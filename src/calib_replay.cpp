// Drive the extrinsic self-calibration observer from a live ROS graph or a bag
// replay. Milestone 6 of CALIBRATION.md: the same ROS-free estimator the gtests
// exercise, fed real depth instead of synthetic.
//
// Deliberately a separate executable rather than a change to the filter node:
// this is how the estimator gets its first contact with real sensor noise, and
// keeping it out of the filter means a bad estimate cannot affect filtering while
// we are still finding out whether it works.
//
//   ros2 run depth_self_filter depth_self_calib_replay --ros-args
//     -p robot_description_file:=/path/robot.urdf
//     -p override_fx:=457.0 -p override_fy:=457.0
//     -r image:=/teleop_camera/depth/image_raw
//     -r camera_info:=/teleop_camera/depth/camera_info
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "depth_self_filter/calibration.hpp"
#include "depth_self_filter/urdf_meshes.hpp"

namespace depth_self_filter
{

class CalibReplayNode : public rclcpp::Node
{
public:
  explicit CalibReplayNode(const rclcpp::NodeOptions & opts)
  : rclcpp::Node("depth_self_calib_replay", opts)
  {
    use_collision_ = declare_parameter("use_collision_meshes", true);
    tf_timeout_s_ = declare_parameter("tf_timeout_s", 0.3);
    time_offset_s_ = declare_parameter("time_offset_s", 0.0);
    near_ = static_cast<float>(declare_parameter("near_m", 0.05));
    far_ = static_cast<float>(declare_parameter("far_m", 6.0));
    stride_ = static_cast<int>(declare_parameter("sample_stride", 3));
    report_every_ = static_cast<int>(declare_parameter("report_every", 100));
    include_links_ = declare_parameter("include_links", std::vector<std::string>{});
    exclude_links_ = declare_parameter("exclude_links", std::vector<std::string>{});

    // Ground-truth experiment: deliberately mis-mount the camera by a KNOWN
    // amount. Every link pose the estimator sees is premultiplied by this, so the
    // correct answer is exactly its inverse — which simulation, unlike a bag,
    // lets us state rather than guess.
    const auto inj = declare_parameter("inject_offset_xyz_rpy", std::vector<double>{});
    if (inj.size() == 6) {
      Eigen::Matrix<double, 6, 1> xi;
      xi << inj[0], inj[1], inj[2], 0, 0, 0;
      T_inject_ = se3_retract(xi);
      T_inject_.linear() =
        (Eigen::AngleAxisd(inj[5], Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(inj[4], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(inj[3], Eigen::Vector3d::UnitX())).toRotationMatrix();
      const Eigen::Matrix<double, 6, 1> want = to_xyz_rpy(T_inject_.inverse());
      RCLCPP_WARN(get_logger(),
        "INJECTED mount error; correct answer is [%.4f %.4f %.4f %.4f %.4f %.4f]",
        want[0], want[1], want[2], want[3], want[4], want[5]);
    }

    ofx_ = declare_parameter("override_fx", 0.0);
    ofy_ = declare_parameter("override_fy", 0.0);
    ocx_ = declare_parameter("override_cx", 0.0);
    ocy_ = declare_parameter("override_cy", 0.0);

    CalibConfig cfg;
    cfg.noise_a = declare_parameter("noise_a", cfg.noise_a);
    cfg.noise_b = declare_parameter("noise_b", cfg.noise_b);
    cfg.prior_sigma_xyz = declare_parameter("prior_sigma_xyz", cfg.prior_sigma_xyz);
    cfg.prior_sigma_rot = declare_parameter("prior_sigma_rot", cfg.prior_sigma_rot);
    cfg.min_model_samples =
      static_cast<std::size_t>(declare_parameter("min_model_samples", 150));
    cfg.max_occluded_fraction =
      declare_parameter("max_occluded_fraction", cfg.max_occluded_fraction);
    cfg.min_usable_fraction =
      declare_parameter("min_usable_fraction", cfg.min_usable_fraction);
    cfg_ = cfg;

    ObserverConfig ocfg;
    ocfg.forget_tau_s = declare_parameter("forget_tau_s", ocfg.forget_tau_s);
    ocfg.jump_sigma = declare_parameter("jump_sigma", ocfg.jump_sigma);
    ocfg.jump_frames = static_cast<int>(declare_parameter("jump_frames", ocfg.jump_frames));

    renderer_ = make_egl_renderer();
    if (!renderer_) renderer_ = make_cpu_renderer();
    {
      std::vector<float> d, p;
      std::vector<RenderItem> none;
      CameraModel probe; probe.width = 4; probe.height = 4;
      probe.fx = probe.fy = 4.f; probe.cx = probe.cy = 2.f;
      have_planes_ = renderer_->render_planes(probe, none, d, p);
    }
    RCLCPP_INFO(get_logger(), "render backend: %s (exact planes: %s)",
      renderer_->name(), have_planes_ ? "yes" : "no — finite-difference fallback");

    observer_ = std::make_unique<ExtrinsicObserver>(*renderer_, cfg_, ocfg);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    std::string urdf = declare_parameter("robot_description", std::string{});
    const std::string urdf_file = declare_parameter("robot_description_file", std::string{});
    if (urdf.empty() && !urdf_file.empty()) {
      std::ifstream f(urdf_file);
      if (f) urdf.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
      else RCLCPP_ERROR(get_logger(), "cannot open %s", urdf_file.c_str());
    }
    if (!urdf.empty()) load_meshes(urdf);

    auto qos = rclcpp::QoS(1).transient_local();
    urdf_sub_ = create_subscription<std_msgs::msg::String>(
      "robot_description", qos,
      [this](const std_msgs::msg::String::SharedPtr m) {
        if (links_.empty() && !m->data.empty()) load_meshes(m->data);
      });

    depth_sub_.subscribe(this, "image", rclcpp::SensorDataQoS().get_rmw_qos_profile());
    info_sub_.subscribe(this, "camera_info", rclcpp::SensorDataQoS().get_rmw_qos_profile());
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), depth_sub_, info_sub_);
    sync_->registerCallback(
      std::bind(&CalibReplayNode::on_frame, this, std::placeholders::_1, std::placeholders::_2));
  }

  double median_err() const
  {
    if (errs_.empty()) return 0.0;
    std::vector<double> v = errs_;
    std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
    return v[v.size() / 2];
  }

  void report(const char * tag) const
  {
    const CalibResult & r = observer_->state();
    const Eigen::Matrix<double, 6, 1> rpy = to_xyz_rpy(r.T_fix);
    std::printf(
      "\n=== %s ===\n"
      "frames used %zu / %zu   residuals %zu   inliers %.1f%%\n"
      "median depth error %.2f mm   median |r|/sigma %.2f   robust scale %.2f\n"
      "jump resets %zu   frames refused for block-displaced residual %zu\n"
      "OVER %zu ACCEPTED FRAMES: mean median-depth-error %.2f mm (worst %.2f, "
      "p50-of-medians %.2f)   mean robust scale %.2f   mean |r|/sigma %.2f   "
      "mean residuals/frame %.0f\n",
      tag, r.frames_used, r.frames_total, r.residual_count, r.inlier_fraction * 100.0,
      r.median_abs_residual_m * 1e3, r.median_normalised_residual, r.robust_scale,
      observer_->jump_resets(), r.frames_rejected_residual,
      acc_n_, acc_n_ ? acc_err_ / acc_n_ * 1e3 : 0.0, worst_err_ * 1e3,
      median_err() * 1e3,
      acc_n_ ? acc_scale_ / acc_n_ : 0.0, acc_n_ ? acc_norm_ / acc_n_ : 0.0,
      acc_n_ ? acc_res_ / acc_n_ : 0.0);
    const char * nm[6] = {"x ", "y ", "z ", "roll ", "pitch", "yaw  "};
    for (int i = 0; i < 6; ++i) {
      const bool ang = i >= 3;
      const double sc = ang ? 180.0 / M_PI : 1e3;
      std::printf("  %s %+8.3f %s   sigma %7.4f   prior_dominance %.3f%s\n",
        nm[i], (ang ? rpy[i] : r.xi[i]) * sc, ang ? "deg" : "mm",
        r.sigma[i] * sc, r.prior_dominance[i],
        r.prior_dominance[i] > 0.9 ? "   <-- NEVER OBSERVED" : "");
    }
    // Per-DoF sigma is a diagonal view of a coupled problem. For a compact target
    // a rotation about x and a translation in y look almost the same, so both can
    // be large while their COMBINATION is tightly determined — which the diagonal
    // cannot show. Report the actual eigenstructure.
    const Eigen::Matrix<double, 6, 6> cov = r.information.inverse();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es(cov);
    std::printf("observability (eigenvectors of the covariance, worst first):\n");
    for (int k = 5; k >= 4; --k) {
      const Eigen::Matrix<double, 6, 1> v = es.eigenvectors().col(k);
      std::printf("  sigma %8.4f along  ", std::sqrt(std::max(0.0, es.eigenvalues()[k])));
      const char * an[6] = {"x", "y", "z", "roll", "pitch", "yaw"};
      for (int i = 0; i < 6; ++i) {
        if (std::fabs(v[i]) > 0.15) std::printf("%+.2f*%s ", v[i], an[i]);
      }
      std::printf("\n");
    }
    // Correlations among the vertical-compensating DoFs.
    const auto corr = [&](int i, int j) {
      return cov(i, j) / std::sqrt(cov(i, i) * cov(j, j));
    };
    std::printf("  corr(y,roll)=%+.4f  corr(x,pitch)=%+.4f  corr(z,pitch)=%+.4f\n",
      corr(1, 3), corr(0, 4), corr(2, 4));
    if (!T_inject_.isApprox(Eigen::Isometry3d::Identity())) {
      const Eigen::Isometry3d truth = T_inject_.inverse();
      const Eigen::Isometry3d err = r.T_fix.inverse() * truth;
      const Eigen::Matrix<double, 6, 1> tr = to_xyz_rpy(truth);
      std::printf("GROUND TRUTH [%.3f %.3f %.3f mm | %.3f %.3f %.3f deg]  "
                  "ERROR %.2f mm / %.3f deg\n",
        tr[0] * 1e3, tr[1] * 1e3, tr[2] * 1e3,
        tr[3] * 180 / M_PI, tr[4] * 180 / M_PI, tr[5] * 180 / M_PI,
        err.translation().norm() * 1e3,
        Eigen::AngleAxisd(err.linear()).angle() * 180 / M_PI);
    }
    std::printf("extrinsic_correction_xyz_rpy: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n",
      rpy[0], rpy[1], rpy[2], rpy[3], rpy[4], rpy[5]);
    std::fflush(stdout);
  }

private:
  using ImageMsg = sensor_msgs::msg::Image;
  using CamInfoMsg = sensor_msgs::msg::CameraInfo;
  using SyncPolicy =
    message_filters::sync_policies::ApproximateTime<ImageMsg, CamInfoMsg>;

  void load_meshes(const std::string & xml)
  {
    links_ = load_link_meshes(xml, use_collision_, include_links_, exclude_links_);
    size_t tris = 0;
    for (const auto & l : links_) tris += l.mesh.indices.size() / 3;
    RCLCPP_INFO(get_logger(), "loaded %zu links, %zu triangles", links_.size(), tris);
  }

  void on_frame(const ImageMsg::ConstSharedPtr & depth, const CamInfoMsg::ConstSharedPtr & info)
  {
    if (links_.empty()) return;
    ++seen_;

    Observation obs;
    obs.cam.width = static_cast<int>(depth->width);
    obs.cam.height = static_cast<int>(depth->height);
    obs.cam.fx = static_cast<float>(ofx_ > 0 ? ofx_ : info->k[0]);
    obs.cam.fy = static_cast<float>(ofy_ > 0 ? ofy_ : info->k[4]);
    obs.cam.cx = static_cast<float>(ocx_ > 0 ? ocx_ : info->k[2]);
    obs.cam.cy = static_cast<float>(ocy_ > 0 ? ocy_ : info->k[5]);
    obs.cam.near = near_;
    obs.cam.far = far_;

    // Link poses exactly as TF reports them; the observer applies its own T_fix.
    const auto stamp = rclcpp::Time(depth->header.stamp) +
      rclcpp::Duration::from_seconds(time_offset_s_);
    size_t tf_fail = 0;
    for (const auto & link : links_) {
      try {
        const auto tf = tf_buffer_->lookupTransform(
          depth->header.frame_id, link.link_name, stamp, tf2::durationFromSec(tf_timeout_s_));
        RenderItem it;
        it.mesh = &link.mesh;
        it.T_cam_link = (T_inject_ * tf2::transformToEigen(tf)).cast<float>();
        if (mesh_in_frustum(obs.cam, link.mesh, it.T_cam_link)) obs.items.push_back(it);
      } catch (const tf2::TransformException &) {
        ++tf_fail;
      }
    }
    if (tf_fail > links_.size() / 2) { ++tf_dropped_; return; }
    if (obs.items.empty()) { ++no_robot_; return; }

    if (!decimate(*depth, obs)) return;

    const double now = rclcpp::Time(depth->header.stamp).seconds();
    const double dt = last_stamp_ > 0.0 ? std::max(0.0, now - last_stamp_) : 0.0;
    last_stamp_ = now;

    if (observer_->update(obs, dt)) {
      // Running aggregates. The observer's own fields describe the LAST frame,
      // which is a misleading summary of a 1500-frame replay.
      const CalibResult & r = observer_->state();
      acc_n_++;
      acc_err_ += r.median_abs_residual_m;
      acc_scale_ += r.robust_scale;
      acc_norm_ += r.median_normalised_residual;
      acc_res_ += static_cast<double>(r.residual_count);
      worst_err_ = std::max(worst_err_, r.median_abs_residual_m);
      errs_.push_back(r.median_abs_residual_m);
    }
    if (report_every_ > 0 && observer_->state().frames_total % report_every_ == 0) {
      report("progress");
    }
  }

  bool decimate(const ImageMsg & img, Observation & obs)
  {
    const int w = static_cast<int>(img.width), h = static_cast<int>(img.height);
    const int s = std::max(1, stride_);
    obs.samples.clear();
    if (img.encoding == "16UC1" || img.encoding == "mono16") {
      const auto * p = reinterpret_cast<const uint16_t *>(img.data.data());
      for (int y = 0; y < h; y += s) {
        for (int x = 0; x < w; x += s) {
          const uint16_t mm = p[static_cast<size_t>(y) * w + x];
          if (mm != 0) obs.samples.push_back(DepthSample{x, y, mm * 1e-3f, 1.f});
        }
      }
      return true;
    }
    if (img.encoding == "32FC1") {
      const auto * p = reinterpret_cast<const float *>(img.data.data());
      for (int y = 0; y < h; y += s) {
        for (int x = 0; x < w; x += s) {
          const float z = p[static_cast<size_t>(y) * w + x];
          if (std::isfinite(z) && z > 0.f) obs.samples.push_back(DepthSample{x, y, z, 1.f});
        }
      }
      return true;
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "unsupported encoding %s", img.encoding.c_str());
    return false;
  }

  bool use_collision_{true};
  double tf_timeout_s_{0.3}, time_offset_s_{0.0};
  float near_{0.05f}, far_{6.f};
  int stride_{3}, report_every_{100};
  double ofx_{0}, ofy_{0}, ocx_{0}, ocy_{0};
  std::vector<std::string> include_links_, exclude_links_;
  CalibConfig cfg_;
  bool have_planes_{false};
  Eigen::Isometry3d T_inject_ = Eigen::Isometry3d::Identity();

  std::vector<LinkGeometry> links_;
  std::unique_ptr<IRenderer> renderer_;
  std::unique_ptr<ExtrinsicObserver> observer_;
  double last_stamp_{-1.0};

public:
  size_t seen_{0}, tf_dropped_{0}, no_robot_{0};
  size_t acc_n_{0};
  double acc_err_{0}, acc_scale_{0}, acc_norm_{0}, acc_res_{0}, worst_err_{0};
  std::vector<double> errs_;

private:
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  message_filters::Subscriber<ImageMsg> depth_sub_;
  message_filters::Subscriber<CamInfoMsg> info_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
};

}  // namespace depth_self_filter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<depth_self_filter::CalibReplayNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  node->report("FINAL");
  std::printf("frames seen %zu, dropped for TF %zu, no robot in view %zu\n",
    node->seen_, node->tf_dropped_, node->no_robot_);
  rclcpp::shutdown();
  return 0;
}
