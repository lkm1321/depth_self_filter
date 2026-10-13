#include "depth_self_filter/depth_self_filter_node.hpp"
#include "depth_self_filter/sphere_mask.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>

#include <tf2_eigen/tf2_eigen.hpp>

namespace depth_self_filter
{
namespace
{
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

// Read the depth image into a float (metres) buffer. Supports 16UC1 (mm) and
// 32FC1 (m). Invalid pixels become NaN.
bool to_metres(const sensor_msgs::msg::Image & img, std::vector<float> & out)
{
  const size_t n = static_cast<size_t>(img.width) * img.height;
  out.assign(n, kNaN);
  if (img.encoding == "16UC1" || img.encoding == "mono16") {
    const auto * p = reinterpret_cast<const uint16_t *>(img.data.data());
    for (size_t i = 0; i < n; ++i) {
      if (p[i] != 0) out[i] = p[i] * 1e-3f;
    }
    return true;
  }
  if (img.encoding == "32FC1") {
    const auto * p = reinterpret_cast<const float *>(img.data.data());
    for (size_t i = 0; i < n; ++i) {
      if (std::isfinite(p[i]) && p[i] > 0.f) out[i] = p[i];
    }
    return true;
  }
  return false;
}

// Grow a boolean mask by `radius` pixels (separable box dilation).
void dilate(std::vector<uint8_t> & m, int w, int h, int radius)
{
  if (radius <= 0) return;
  std::vector<uint8_t> tmp(m.size(), 0);
  for (int y = 0; y < h; ++y) {           // horizontal
    for (int x = 0; x < w; ++x) {
      uint8_t v = 0;
      for (int dx = -radius; dx <= radius && !v; ++dx) {
        const int xx = x + dx;
        if (xx >= 0 && xx < w) v = m[static_cast<size_t>(y) * w + xx];
      }
      tmp[static_cast<size_t>(y) * w + x] = v;
    }
  }
  for (int y = 0; y < h; ++y) {           // vertical
    for (int x = 0; x < w; ++x) {
      uint8_t v = 0;
      for (int dy = -radius; dy <= radius && !v; ++dy) {
        const int yy = y + dy;
        if (yy >= 0 && yy < h) v = tmp[static_cast<size_t>(yy) * w + x];
      }
      m[static_cast<size_t>(y) * w + x] = v;
    }
  }
}

// Grayscale min-dilation of the rendered robot depth: grow the robot's nearest
// surface outward by `radius` px, filling neighbours (and invalid pixels) with
// the closest robot depth in range. This absorbs lateral silhouette / extrinsic
// error WITHOUT swallowing foreground objects — unlike a binary mask dilation —
// because the mask test still requires `measured >= robot - margin`, so an object
// closer than the (grown) robot surface still survives. Invalid pixels are 0.
// min is separable, so we do a horizontal then a vertical pass.
void dilate_robot_depth(std::vector<float> & d, int w, int h, int radius)
{
  if (radius <= 0) return;
  constexpr float kInf = std::numeric_limits<float>::infinity();
  const auto val = [](float x) { return x > 0.f ? x : kInf; };
  std::vector<float> tmp(d.size(), kInf);
  for (int y = 0; y < h; ++y) {           // horizontal min
    for (int x = 0; x < w; ++x) {
      float best = kInf;
      const int lo = std::max(0, x - radius), hi = std::min(w - 1, x + radius);
      for (int xx = lo; xx <= hi; ++xx) best = std::min(best, val(d[static_cast<size_t>(y) * w + xx]));
      tmp[static_cast<size_t>(y) * w + x] = best;
    }
  }
  for (int y = 0; y < h; ++y) {           // vertical min
    for (int x = 0; x < w; ++x) {
      float best = kInf;
      const int lo = std::max(0, y - radius), hi = std::min(h - 1, y + radius);
      for (int yy = lo; yy <= hi; ++yy) best = std::min(best, tmp[static_cast<size_t>(yy) * w + x]);
      d[static_cast<size_t>(y) * w + x] = std::isfinite(best) ? best : 0.f;
    }
  }
}

}  // namespace

DepthSelfFilterNode::DepthSelfFilterNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("depth_self_filter", options)
{
  use_collision_ = declare_parameter("use_collision_meshes", true);
  require_complete_robot_tf_ = declare_parameter("require_complete_robot_tf", false);
  depth_margin_m_ = declare_parameter("depth_margin_m", 0.02);
  robot_dilation_px_ = declare_parameter("robot_dilation_px", 2);
  mask_dilation_px_ = declare_parameter("mask_dilation_px", 4);
  tf_timeout_s_ = declare_parameter("tf_timeout_s", 0.05);
  time_offset_s_ = declare_parameter("time_offset_s", 0.0);
  near_ = static_cast<float>(declare_parameter("near_m", 0.02));
  far_ = static_cast<float>(declare_parameter("far_m", 10.0));
  frustum_cull_ = declare_parameter("frustum_cull", true);
  include_links_ = declare_parameter("include_links", std::vector<std::string>{});
  exclude_links_ = declare_parameter("exclude_links", std::vector<std::string>{});
  std::string urdf_param = declare_parameter("robot_description", std::string{});
  const std::string urdf_file = declare_parameter("robot_description_file", std::string{});
  if (urdf_param.empty() && !urdf_file.empty()) {
    std::ifstream f(urdf_file);
    if (f) {
      urdf_param.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
      RCLCPP_INFO(get_logger(), "loaded URDF from %s (%zu bytes)",
        urdf_file.c_str(), urdf_param.size());
    } else {
      RCLCPP_ERROR(get_logger(), "cannot open robot_description_file: %s", urdf_file.c_str());
    }
  }

  // Renderer: prefer EGL (NVIDIA/GPU), fall back to CPU rasterizer.
  renderer_ = make_egl_renderer();
  if (!renderer_) renderer_ = make_cpu_renderer();
  RCLCPP_INFO(get_logger(), "render backend: %s", renderer_->name());

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  sphere_padding_=declare_parameter("sphere_padding_m",0.005);
  if (!std::isfinite(sphere_padding_) || sphere_padding_<0)
    throw std::invalid_argument("sphere_padding_m must be finite and nonnegative");
  filtering_enabled_=declare_parameter("filtering_enabled",true);
  // Default mutually-exclusive callback group serializes services and frames.
  sphere_service_=create_service<sphere_scene_interfaces::srv::SetSphereSet>(
    "~/set_sphere_set", [this](
      const std::shared_ptr<sphere_scene_interfaces::srv::SetSphereSet::Request> request,
      std::shared_ptr<sphere_scene_interfaces::srv::SetSphereSet::Response> response) {
      *response=sphere_sets_.apply(*request);
    });
  enabled_service_=create_service<std_srvs::srv::SetBool>(
    "~/set_enabled", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                           std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
      filtering_enabled_=request->data;
      response->success=true;
      response->message=filtering_enabled_?"publishing enabled":"frames withheld";
    });

  if (!urdf_param.empty()) {
    load_meshes(urdf_param);
  }
  // Also listen for a latched robot_description (robot_state_publisher).
  auto qos = rclcpp::QoS(1).transient_local();
  urdf_sub_ = create_subscription<std_msgs::msg::String>(
    "robot_description", qos,
    std::bind(&DepthSelfFilterNode::on_robot_description, this, std::placeholders::_1));

  publish_debug_ = declare_parameter("publish_debug", false);
  declare_parameter("focal_scale", 1.0);   // debug: multiply camera_info fx,fy
  declare_parameter("override_fx", 0.0);   // >0 overrides camera_info intrinsics
  declare_parameter("override_fy", 0.0);
  declare_parameter("override_cx", 0.0);
  declare_parameter("override_cy", 0.0);
  pub_ = create_publisher<ImageMsg>("image_filtered", rclcpp::SensorDataQoS());
  if (publish_debug_) {
    robot_depth_pub_ = create_publisher<ImageMsg>("robot_depth", rclcpp::SensorDataQoS());
    sphere_mask_pub_=create_publisher<ImageMsg>("~/sphere_mask",rclcpp::SensorDataQoS());
  }

  depth_sub_.subscribe(this, "image", rclcpp::SensorDataQoS().get_rmw_qos_profile());
  info_sub_.subscribe(this, "camera_info", rclcpp::SensorDataQoS().get_rmw_qos_profile());
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10), depth_sub_, info_sub_);
  sync_->registerCallback(
    std::bind(&DepthSelfFilterNode::on_frame, this,
      std::placeholders::_1, std::placeholders::_2));
}

void DepthSelfFilterNode::on_robot_description(
  const std_msgs::msg::String::SharedPtr msg)
{
  if (!links_.empty() || msg->data.empty()) return;  // already loaded
  load_meshes(msg->data);
}

void DepthSelfFilterNode::load_meshes(const std::string & urdf_xml)
{
  links_ = load_link_meshes(urdf_xml, use_collision_, include_links_, exclude_links_);
  size_t tris = 0;
  for (const auto & l : links_) tris += l.mesh.indices.size() / 3;
  RCLCPP_INFO(get_logger(), "loaded %zu links, %zu triangles", links_.size(), tris);
  for (const auto & l : links_) {
    RCLCPP_INFO(get_logger(), "  link '%s': %zu tris",
      l.link_name.c_str(), l.mesh.indices.size() / 3);
  }
  if (links_.empty()) {
    RCLCPP_WARN(get_logger(), "no link geometry loaded — filter is a pass-through");
  }
}

bool DepthSelfFilterNode::build_items(
  const CameraModel & cam, const std::string & optical_frame,
  const rclcpp::Time & stamp, std::vector<RenderItem> & items)
{
  items.clear();
  const auto lookup_stamp = stamp + rclcpp::Duration::from_seconds(time_offset_s_);
  const auto timeout = tf2::durationFromSec(tf_timeout_s_);
  size_t failures = 0, culled = 0;
  for (const auto & link : links_) {
    Eigen::Isometry3f T_cam_link;
    try {
      const auto tf = tf_buffer_->lookupTransform(
        optical_frame, link.link_name, lookup_stamp, timeout);
      T_cam_link = tf2::transformToEigen(tf).cast<float>();
    } catch (const tf2::TransformException &) {
      ++failures;
      continue;
    }
    // Visibility pre-selection: drop links whose posed bounding box can't fall
    // anywhere in the camera frustum, so the backend never transforms/rasterizes
    // (or, on the GPU path, uploads + issues a draw for) an off-screen link.
    // A culled link is still a successful TF lookup, so it does NOT count as a
    // failure against the "most links placed" quorum below.
    if (frustum_cull_ && !mesh_in_frustum(cam, link.mesh, T_cam_link)) {
      ++culled;
      continue;
    }
    RenderItem it;
    it.mesh = &link.mesh;
    it.T_cam_link = T_cam_link;
    items.push_back(it);
  }
  if (culled > 0) {
    RCLCPP_DEBUG(get_logger(), "frustum cull: %zu/%zu links off-screen, rendering %zu",
      culled, links_.size(), items.size());
  }
  // If we couldn't place most links, the render would be wrong — skip the frame.
  if (require_complete_robot_tf_ && failures) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "withholding depth: %zu robot link transforms unavailable", failures);
    return false;
  }
  return !links_.empty() && failures <= links_.size() / 2;
}

void DepthSelfFilterNode::on_frame(
  const ImageMsg::ConstSharedPtr & depth, const CamInfoMsg::ConstSharedPtr & info)
{
  if (!filtering_enabled_) return;
  std::vector<float> meas;
  if (!to_metres(*depth, meas)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "unsupported depth encoding: %s", depth->encoding.c_str());
    return;
  }

  // Build the camera model up front so it can drive both frustum culling (in
  // build_items) and the render below.
  CameraModel cam;
  cam.width = static_cast<int>(depth->width);
  cam.height = static_cast<int>(depth->height);
  // Intrinsics: use camera_info by default; allow per-axis override for cameras
  // whose recorded camera_info is wrong/placeholder (>0 overrides). focal_scale
  // additionally multiplies fx,fy (debug convenience).
  const double fs = this->get_parameter("focal_scale").as_double();
  const double ofx = this->get_parameter("override_fx").as_double();
  const double ofy = this->get_parameter("override_fy").as_double();
  const double ocx = this->get_parameter("override_cx").as_double();
  const double ocy = this->get_parameter("override_cy").as_double();
  cam.fx = static_cast<float>((ofx > 0 ? ofx : info->k[0]) * fs);
  cam.fy = static_cast<float>((ofy > 0 ? ofy : info->k[4]) * fs);
  cam.cx = static_cast<float>(ocx > 0 ? ocx : info->k[2]);
  cam.cy = static_cast<float>(ocy > 0 ? ocy : info->k[5]);
  cam.near = near_;
  cam.far = far_;

  if (!(cam.fx>0 && cam.fy>0) || !std::isfinite(cam.fx) || !std::isfinite(cam.fy) ||
      !std::isfinite(cam.cx) || !std::isfinite(cam.cy)) return;
  const auto before_spheres=publish_debug_?meas:std::vector<float>{};
  for (const auto & entry:sphere_sets_.entries()) {
    if (entry.second.remove) continue;
    const auto & set=entry.second.set;
    Eigen::Isometry3d camera_from_parent;
    try {
      camera_from_parent=tf2::transformToEigen(tf_buffer_->lookupTransform(
        depth->header.frame_id,set.parent_frame,
        rclcpp::Time(depth->header.stamp)+rclcpp::Duration::from_seconds(time_offset_s_),
        tf2::durationFromSec(tf_timeout_s_)));
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,
        "withholding depth: sphere set '%s' TF unavailable: %s",set.id.c_str(),e.what());
      return;
    }
    const auto & p=set.pose.position; const auto & q=set.pose.orientation;
    Eigen::Isometry3d parent_from_set=Eigen::Isometry3d::Identity();
    parent_from_set.linear()=Eigen::Quaterniond(q.w,q.x,q.y,q.z).normalized().toRotationMatrix();
    parent_from_set.translation()=Eigen::Vector3d(p.x,p.y,p.z);
    const auto camera_from_set=camera_from_parent*parent_from_set;
    for (const auto & sphere:set.spheres) {
      const Eigen::Vector3d point=camera_from_set*Eigen::Vector3d(
        sphere.center.x,sphere.center.y,sphere.center.z);
      mask_sphere(cam,point.cast<float>(),float(sphere.radius+sphere_padding_),meas);
    }
  }
  if (sphere_mask_pub_) {
    auto mask=std::make_unique<ImageMsg>();
    mask->header=depth->header; mask->height=depth->height; mask->width=depth->width;
    mask->encoding="mono8"; mask->step=depth->width; mask->data.resize(meas.size());
    for (size_t i=0;i<meas.size();++i)
      mask->data[i]=std::isfinite(before_spheres[i])&&!std::isfinite(meas[i])?255:0;
    sphere_mask_pub_->publish(std::move(mask));
  }

  // Pass-through if we have no geometry or no TF yet.
  std::vector<RenderItem> items;
  const bool have_robot =
    !links_.empty() &&
    build_items(cam, depth->header.frame_id, rclcpp::Time(depth->header.stamp), items);

  if (require_complete_robot_tf_ && !have_robot) return;

  if (have_robot) {
    if (renderer_->render(cam, items, robot_depth_)) {
      // Grow the rendered robot's near surface to absorb silhouette / calibration
      // error (keeps foreground objects; see dilate_robot_depth).
      dilate_robot_depth(robot_depth_, cam.width, cam.height, robot_dilation_px_);

      const size_t n = meas.size();
      std::vector<uint8_t> mask(n, 0);
      const float margin = static_cast<float>(depth_margin_m_);
      for (size_t i = 0; i < n; ++i) {
        const float r = robot_depth_[i];       // 0 == no robot here
        const float m = meas[i];
        if (r > 0.f && std::isfinite(m) && m >= r - margin) {
          mask[i] = 1;  // observed surface is at/behind the robot -> it's the robot
        }
      }
      dilate(mask, cam.width, cam.height, mask_dilation_px_);
      for (size_t i = 0; i < n; ++i) {
        if (mask[i]) meas[i] = kNaN;
      }

      // Debug: publish the raw rendered robot depth (0 -> NaN for viewers).
      if (publish_debug_ && robot_depth_pub_) {
        auto dbg = std::make_unique<ImageMsg>();
        dbg->header = depth->header;
        dbg->height = depth->height; dbg->width = depth->width;
        dbg->encoding = "32FC1"; dbg->is_bigendian = 0;
        dbg->step = depth->width * sizeof(float);
        dbg->data.resize(static_cast<size_t>(dbg->step) * dbg->height);
        auto * p = reinterpret_cast<float *>(dbg->data.data());
        for (size_t i = 0; i < n; ++i) p[i] = robot_depth_[i] > 0.f ? robot_depth_[i] : kNaN;
        robot_depth_pub_->publish(std::move(dbg));
      }
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "render failed");
      if (require_complete_robot_tf_) return;
    }
  }

  // Publish as 32FC1 (NaN = filtered/invalid).
  auto out = std::make_unique<ImageMsg>();
  out->header = depth->header;
  out->height = depth->height;
  out->width = depth->width;
  out->encoding = "32FC1";
  out->is_bigendian = 0;
  out->step = depth->width * sizeof(float);
  out->data.resize(static_cast<size_t>(out->step) * out->height);
  std::memcpy(out->data.data(), meas.data(), out->data.size());
  pub_->publish(std::move(out));
}

}  // namespace depth_self_filter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(depth_self_filter::DepthSelfFilterNode)
