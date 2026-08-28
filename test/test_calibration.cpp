// Tests for the extrinsic self-calibration estimator (CALIBRATION.md).
//
// The scene is deliberately UNCHOREOGRAPHED: pseudo-random link configurations
// standing in for whatever the robot happened to do, since the design forbids
// asking for calibration poses. Depth is corrupted the way a ToF sensor corrupts
// it — range-dependent noise, flying pixels on silhouettes, dropouts — plus a
// background wall and an occluding object, so the gates and the robust loss are
// exercised rather than assumed.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include <memory>

#include "depth_self_filter/calibration.hpp"

using namespace depth_self_filter;

namespace
{
constexpr double kDeg = M_PI / 180.0;
constexpr float kWallDepth = 2.5f;

CameraModel make_cam()
{
  CameraModel c;
  c.width = 320; c.height = 240;
  c.fx = c.fy = 250.f;
  c.cx = 160.f; c.cy = 120.f;
  c.near = 0.05f; c.far = 6.f;
  return c;
}

Mesh make_box(float hx, float hy, float hz)
{
  Mesh m;
  m.vertices = {
    {-hx, -hy, -hz}, {hx, -hy, -hz}, {hx, hy, -hz}, {-hx, hy, -hz},
    {-hx, -hy,  hz}, {hx, -hy,  hz}, {hx, hy,  hz}, {-hx, hy,  hz}};
  m.indices = {
    0,1,2, 0,2,3,  4,6,5, 4,7,6,      // -z, +z
    0,4,5, 0,5,1,  3,2,6, 3,6,7,      // -y, +y
    0,3,7, 0,7,4,  1,5,6, 1,6,2};     // -x, +x
  compute_bounds(m);
  return m;
}

Eigen::Isometry3d correction(
  double x, double y, double z, double rx_deg, double ry_deg, double rz_deg)
{
  Eigen::Matrix<double, 6, 1> xi;
  xi << x, y, z, rx_deg * kDeg, ry_deg * kDeg, rz_deg * kDeg;
  return se3_retract(xi);
}

/// One arbitrary kinematic configuration: a 3-link chain at a random place and
/// attitude in front of the camera. Nothing here is chosen to be informative.
std::vector<Eigen::Isometry3f> make_config(std::mt19937 & rng)
{
  std::uniform_real_distribution<float> ang(-0.7f, 0.7f);
  std::uniform_real_distribution<float> lat(-0.16f, 0.16f);
  std::uniform_real_distribution<float> depth(-0.15f, 0.45f);

  Eigen::Isometry3f cur = Eigen::Isometry3f::Identity();
  cur.translation() = Eigen::Vector3f(lat(rng), lat(rng), 0.85f + depth(rng));
  cur.linear() =
    (Eigen::AngleAxisf(ang(rng), Eigen::Vector3f::UnitX()) *
     Eigen::AngleAxisf(ang(rng), Eigen::Vector3f::UnitY()) *
     Eigen::AngleAxisf(ang(rng), Eigen::Vector3f::UnitZ())).toRotationMatrix();

  std::vector<Eigen::Isometry3f> out;
  for (int i = 0; i < 3; ++i) {
    out.push_back(cur);
    Eigen::Isometry3f step = Eigen::Isometry3f::Identity();
    step.translation() = Eigen::Vector3f(0.17f, 0.f, 0.f);
    step.linear() =
      (Eigen::AngleAxisf(ang(rng), Eigen::Vector3f::UnitY()) *
       Eigen::AngleAxisf(ang(rng), Eigen::Vector3f::UnitZ())).toRotationMatrix();
    cur = cur * step;
  }
  return out;
}

/// Ground-truth depth by EXACT ray-plane intersection, not the rasterizer's
/// linear-in-screen z. A real sensor sees the true surface; generating the
/// synthetic measurement with the rasterizer's own approximation would make both
/// sides of the comparison wrong in the same way and hide a systematic error of
/// order (z1-z2)^2 / (2(z1+z2)) — a few mm on a slanted face at ~1 m.
std::vector<float> exact_depth(
  IRenderer & r, const CameraModel & cam, const std::vector<RenderItem> & items)
{
  std::vector<float> depth, planes;
  if (!r.render_planes(cam, items, depth, planes)) {
    r.render(cam, items, depth);   // backend without planes: best available
    return depth;
  }
  for (int y = 0; y < cam.height; ++y) {
    for (int x = 0; x < cam.width; ++x) {
      const size_t i = static_cast<size_t>(y) * cam.width + x;
      if (depth[i] <= 0.f) continue;
      const Eigen::Vector3f n(planes[i * 4], planes[i * 4 + 1], planes[i * 4 + 2]);
      const Eigen::Vector3f v(
        (x + 0.5f - cam.cx) / cam.fx, (y + 0.5f - cam.cy) / cam.fy, 1.f);
      const float den = n.dot(v);
      if (std::fabs(den) > 1e-9f) depth[i] = planes[i * 4 + 3] / den;
    }
  }
  return depth;
}

std::vector<RenderItem> pose_items(
  const std::vector<Mesh> & meshes, const std::vector<Eigen::Isometry3f> & poses,
  const Eigen::Isometry3d & pre)
{
  std::vector<RenderItem> items;
  for (size_t i = 0; i < poses.size(); ++i) {
    RenderItem it;
    it.mesh = &meshes[i % meshes.size()];
    it.T_cam_link = (pre * poses[i].cast<double>()).cast<float>();
    items.push_back(it);
  }
  return items;
}

/// Turn a ground-truth render into a plausible measurement: background wall,
/// range-dependent noise, flying pixels along silhouettes, dropouts, and
/// optionally a real object occluding part of the robot.
std::vector<float> corrupt(
  const std::vector<float> & truth, const CameraModel & cam,
  const std::vector<float> * occluder, std::mt19937 & rng)
{
  std::normal_distribution<float> unit(0.f, 1.f);
  std::uniform_real_distribution<float> uni(0.f, 1.f);
  std::uniform_real_distribution<float> fly(0.05f, 0.40f);
  const int w = cam.width, h = cam.height;

  std::vector<float> meas(truth.size(), 0.f);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const size_t i = static_cast<size_t>(y) * w + x;
      float z = truth[i] > 0.f ? truth[i] : kWallDepth;

      // A real object in front of the robot: not noise, a hard occlusion.
      // Composite with z-buffer semantics — the occluder only wins where it is
      // actually nearer. Overriding unconditionally invents measurements that lie
      // BEHIND a robot surface the camera can see, which is physically impossible
      // and which the one-sided occlusion gate is (correctly) not built to catch.
      if (occluder && (*occluder)[i] > 0.f && (*occluder)[i] < z) z = (*occluder)[i];

      if (uni(rng) < 0.10f) { meas[i] = 0.f; continue; }  // dropout

      // Flying pixels concentrate on depth discontinuities.
      bool edge = false;
      for (int dy = -1; dy <= 1 && !edge; ++dy) {
        for (int dx = -1; dx <= 1 && !edge; ++dx) {
          const int xx = std::min(w - 1, std::max(0, x + dx));
          const int yy = std::min(h - 1, std::max(0, y + dy));
          const size_t j = static_cast<size_t>(yy) * w + xx;
          edge = (truth[i] > 0.f) != (truth[j] > 0.f);
        }
      }
      if (edge && uni(rng) < 0.30f) z += fly(rng);

      z += unit(rng) * (0.005f + 0.005f * z * z);  // sigma(z) = a + b z^2
      meas[i] = z;
    }
  }
  return meas;
}

std::vector<DepthSample> decimate(
  const std::vector<float> & meas, const CameraModel & cam, int stride)
{
  std::vector<DepthSample> out;
  for (int y = 0; y < cam.height; y += stride) {
    for (int x = 0; x < cam.width; x += stride) {
      const float z = meas[static_cast<size_t>(y) * cam.width + x];
      if (z > 0.f) out.push_back(DepthSample{x, y, z, 1.f});
    }
  }
  return out;
}

/// Build `count` observations of the robot at random configurations, as seen by a
/// camera that is actually at `T_true` relative to what TF claims.
struct Occluder
{
  const Mesh * mesh = nullptr;      ///< null = none
  Eigen::Vector3f at{0.f, 0.f, 0.6f};
  int every = 0;                    ///< place it on every Nth frame (0 = never)
};

std::vector<Observation> make_stream(
  const std::vector<Mesh> & meshes, const Occluder & occluder,
  const Eigen::Isometry3d & T_true, IRenderer & renderer, int count,
  unsigned seed)
{
  std::mt19937 rng(seed);
  const CameraModel cam = make_cam();
  std::vector<Observation> obs;

  for (int k = 0; k < count; ++k) {
    const auto poses = make_config(rng);

    const std::vector<float> truth =
      exact_depth(renderer, cam, pose_items(meshes, poses, T_true));

    std::vector<float> occ;
    const bool use_occ =
      occluder.mesh && occluder.every > 0 && (k % occluder.every == 0);
    if (use_occ) {
      Eigen::Isometry3f o = Eigen::Isometry3f::Identity();
      o.translation() = occluder.at;
      RenderItem it; it.mesh = occluder.mesh; it.T_cam_link = o;
      occ = exact_depth(renderer, cam, {it});
    }

    Observation ob;
    ob.cam = cam;
    // Poses as TF reports them: uncorrected. Recovering T_true is the whole job.
    ob.items = pose_items(meshes, poses, Eigen::Isometry3d::Identity());
    ob.samples = decimate(corrupt(truth, cam, use_occ ? &occ : nullptr, rng), cam, 3);
    obs.push_back(std::move(ob));
  }
  return obs;
}

/// Print the estimate and its diagnostics. Observed numbers in the test log are
/// worth more than a bare pass: they show how much margin the thresholds have.
void dump(const CalibResult & res, const Eigen::Isometry3d & T_true)
{
  const Eigen::Matrix<double, 6, 1> truth = se3_params(T_true);
  std::fprintf(stderr,
    "  frames=%zu/%zu on_model=%zu residuals=%zu inliers=%.1f%% median|r|=%.1f mm "
    "robust_scale=%.2f outer=%d conv=%d\n",
    res.frames_used, res.frames_total, res.model_sample_count, res.residual_count,
    res.inlier_fraction * 100.0,
    res.median_abs_residual_m * 1e3, res.robust_scale, res.outer_iterations,
    int(res.converged));
  const char * nm[6] = {"tx", "ty", "tz", "rx", "ry", "rz"};
  for (int i = 0; i < 6; ++i) {
    const double sc = i < 3 ? 1e3 : 1.0 / kDeg;              // mm / deg
    std::fprintf(stderr,
      "  %s: est=%+8.3f true=%+8.3f err=%+8.3f sigma=%7.4f prior_dom=%.3f\n",
      nm[i], res.xi[i] * sc, truth[i] * sc, (res.xi[i] - truth[i]) * sc,
      res.sigma[i] * sc, res.prior_dominance[i]);
  }
}

/// A backend that deliberately does NOT supply planes, standing in for the EGL
/// renderer (which would need a second colour attachment). This is the path that
/// actually ships on GPU, so the finite-difference fallback has to be tested.
class NoPlaneRenderer : public IRenderer
{
public:
  explicit NoPlaneRenderer(std::unique_ptr<IRenderer> inner) : inner_(std::move(inner)) {}
  bool render(
    const CameraModel & cam, const std::vector<RenderItem> & items,
    std::vector<float> & depth_out) override
  {
    return inner_->render(cam, items, depth_out);
  }
  const char * name() const override { return "cpu-noplanes"; }

private:
  std::unique_ptr<IRenderer> inner_;
};

}  // namespace

TEST(Calibration, Se3HelpersRoundTrip)
{
  const Eigen::Isometry3d T = correction(0.01, -0.02, 0.03, 1.5, -2.0, 0.7);
  const Eigen::Matrix<double, 6, 1> xi = se3_params(T);
  EXPECT_NEAR((se3_retract(xi).matrix() - T.matrix()).norm(), 0.0, 1e-12);
  const Eigen::Matrix<double, 6, 1> rpy = to_xyz_rpy(T);
  EXPECT_NEAR(rpy[0], 0.01, 1e-12);

  // Near identity the RPY must be near ZERO. Eigen's eulerAngles(2,1,0) reports
  // (pi, pi, pi) here — the same rotation, but useless as a written parameter.
  const Eigen::Matrix<double, 6, 1> tiny =
    to_xyz_rpy(correction(0.001, 0.0, 0.0, 0.02, -0.03, 0.01));
  for (int i = 3; i < 6; ++i) EXPECT_LT(std::fabs(tiny[i]), 0.01) << "rpy[" << i << "]";
  // And it must round-trip through the ROS convention it claims to use.
  const Eigen::Isometry3d T2 = correction(0.0, 0.0, 0.0, 3.0, -7.0, 11.0);
  const Eigen::Matrix<double, 6, 1> r2 = to_xyz_rpy(T2);
  const Eigen::Matrix3d rebuilt =
    (Eigen::AngleAxisd(r2[5], Eigen::Vector3d::UnitZ()) *
     Eigen::AngleAxisd(r2[4], Eigen::Vector3d::UnitY()) *
     Eigen::AngleAxisd(r2[3], Eigen::Vector3d::UnitX())).toRotationMatrix();
  EXPECT_LT((rebuilt - T2.linear()).norm(), 1e-12);
  // Rotation vector and RPY agree to first order at these magnitudes.
  EXPECT_NEAR(rpy[3], xi[3], 2e-3);
  EXPECT_NEAR(rpy[5], xi[5], 2e-3);
}

// Test (1) from CALIBRATION.md: recovery from noisy, unchoreographed poses.
TEST(Calibration, RecoversKnownOffsetFromNoisySelfObservations)
{
  auto renderer = make_cpu_renderer();
  const std::vector<Mesh> meshes = {
    make_box(0.10f, 0.055f, 0.055f),
    make_box(0.085f, 0.05f, 0.05f),
    make_box(0.06f, 0.045f, 0.045f)};
  const Mesh occ_mesh = make_box(0.05f, 0.05f, 0.02f);
  const Occluder occluder{&occ_mesh, {0.10f, 0.08f, 0.55f}, 3};

  const Eigen::Isometry3d T_true = correction(0.012, -0.008, 0.020, 0.5, 1.2, -0.8);
  const auto obs = make_stream(meshes, occluder, T_true, *renderer, 40, 1234u);

  const CalibResult res = solve_extrinsic(obs, *renderer);
  dump(res, T_true);

  const Eigen::Isometry3d err = res.T_fix.inverse() * T_true;
  const double trans_err = err.translation().norm();
  const double rot_err = Eigen::AngleAxisd(err.linear()).angle();

  EXPECT_TRUE(res.converged);
  EXPECT_LT(trans_err, 0.003) << "translation error " << trans_err * 1e3 << " mm";
  EXPECT_LT(rot_err, 0.2 * kDeg) << "rotation error " << rot_err / kDeg << " deg";

  // The data must have moved every DoF well off the prior, or the recovery above
  // was luck rather than evidence.
  for (int i = 0; i < 6; ++i) {
    EXPECT_LT(res.prior_dominance[i], 0.2) << "DoF " << i << " stayed prior-dominated";
  }

  // Reported sigma must bracket the actual error. A calibrator that is
  // confidently wrong is worse than one that stays at nominal, so the error bars
  // are part of the contract, not decoration.
  const Eigen::Matrix<double, 6, 1> truth = se3_params(T_true);
  for (int i = 0; i < 6; ++i) {
    EXPECT_LT(std::fabs(res.xi[i] - truth[i]), 3.0 * res.sigma[i])
      << "DoF " << i << " error exceeds 3 sigma (" << res.sigma[i] << ")";
  }
}

// Heavy occlusion on SOME frames. Those frames are refused outright; the rest
// still carry the answer. This is the common case during manipulation.
TEST(Calibration, DropsOccludedFramesAndRecoversFromTheRest)
{
  auto renderer = make_cpu_renderer();
  const std::vector<Mesh> meshes = {
    make_box(0.10f, 0.055f, 0.055f),
    make_box(0.085f, 0.05f, 0.05f),
    make_box(0.06f, 0.045f, 0.045f)};
  // A wide slab across the middle of the view, well in front of the robot.
  const Mesh occ_mesh = make_box(0.30f, 0.10f, 0.02f);
  const Occluder occluder{&occ_mesh, {0.f, 0.f, 0.65f}, 3};

  const Eigen::Isometry3d T_true = correction(0.012, -0.008, 0.020, 0.5, 1.2, -0.8);
  const auto obs = make_stream(meshes, occluder, T_true, *renderer, 45, 1234u);

  const CalibResult res = solve_extrinsic(obs, *renderer);
  dump(res, T_true);

  EXPECT_EQ(res.frames_total, 45u);
  EXPECT_GT(res.frames_used, 20u);   // the ~2/3 without an occluder
  EXPECT_LT(res.frames_used, 35u);   // but the occluded third must be gone

  const Eigen::Isometry3d err = res.T_fix.inverse() * T_true;
  EXPECT_LT(err.translation().norm(), 0.003)
    << "translation error " << err.translation().norm() * 1e3 << " mm";
  EXPECT_LT(Eigen::AngleAxisd(err.linear()).angle(), 0.2 * kDeg);
}

// Every frame occluded: there is no usable observation, so the estimator must
// return the nominal it started from and say the data was never seen — not a
// plausible-looking number fitted to whatever slipped past.
TEST(Calibration, RefusesWhenEveryFrameIsOccluded)
{
  auto renderer = make_cpu_renderer();
  const std::vector<Mesh> meshes = {
    make_box(0.10f, 0.055f, 0.055f),
    make_box(0.085f, 0.05f, 0.05f),
    make_box(0.06f, 0.045f, 0.045f)};
  const Mesh occ_mesh = make_box(0.45f, 0.35f, 0.02f);
  const Occluder occluder{&occ_mesh, {0.f, 0.f, 0.62f}, 1};

  const Eigen::Isometry3d T_true = correction(0.012, -0.008, 0.020, 0.5, 1.2, -0.8);
  const auto obs = make_stream(meshes, occluder, T_true, *renderer, 20, 1234u);

  const CalibResult res = solve_extrinsic(obs, *renderer);
  dump(res, T_true);

  EXPECT_EQ(res.frames_used, 0u);
  EXPECT_EQ(res.residual_count, 0u);
  EXPECT_LT(res.T_fix.translation().norm(), 1e-9);
  for (int i = 0; i < 6; ++i) EXPECT_NEAR(res.prior_dominance[i], 1.0, 1e-9);
}

// Robot outside the field of view: nothing renders, so there is nothing to
// calibrate against and the frame is not an observation at all.
TEST(Calibration, SkipsFramesWithRobotOutOfView)
{
  auto renderer = make_cpu_renderer();
  const std::vector<Mesh> meshes = {
    make_box(0.10f, 0.055f, 0.055f),
    make_box(0.085f, 0.05f, 0.05f),
    make_box(0.06f, 0.045f, 0.045f)};
  const CameraModel cam = make_cam();
  const Eigen::Isometry3d T_true = correction(0.012, -0.008, 0.020, 0.5, 1.2, -0.8);

  std::mt19937 rng(5u);
  std::vector<Observation> obs;
  const int kFrames = 24, kInView = 12;
  for (int k = 0; k < kFrames; ++k) {
    auto poses = make_config(rng);
    if (k >= kInView) {           // swing the robot well outside the frustum
      for (auto & p : poses) p.translation().x() += 3.0f;
    }
    const std::vector<float> truth =
      exact_depth(*renderer, cam, pose_items(meshes, poses, T_true));
    Observation ob;
    ob.cam = cam;
    ob.items = pose_items(meshes, poses, Eigen::Isometry3d::Identity());
    ob.samples = decimate(corrupt(truth, cam, nullptr, rng), cam, 3);
    obs.push_back(std::move(ob));
  }

  const CalibResult res = solve_extrinsic(obs, *renderer);
  dump(res, T_true);

  EXPECT_EQ(res.frames_total, static_cast<size_t>(kFrames));
  EXPECT_EQ(res.frames_used, static_cast<size_t>(kInView));
  EXPECT_LT((res.T_fix.inverse() * T_true).translation().norm(), 0.003);
}

// Test (2): starvation. A fronto-parallel surface constrains t_z, rx and ry, but
// says nothing about sliding in-plane (t_x, t_y) or rolling about the optical
// axis (rz). Those must stay at the URDF nominal instead of chasing noise.
TEST(Calibration, StarvedDofsStayAtNominal)
{
  auto renderer = make_cpu_renderer();
  const std::vector<Mesh> meshes = {make_box(0.55f, 0.42f, 0.05f)};
  const CameraModel cam = make_cam();

  // A single flat slab square-on to the camera, at a fixed pose.
  Eigen::Isometry3f flat = Eigen::Isometry3f::Identity();
  flat.translation() = Eigen::Vector3f(0.f, 0.f, 1.5f);

  const Eigen::Isometry3d T_true = correction(0.015, -0.010, 0.020, 0.0, 0.0, 0.0);
  std::mt19937 rng(7u);
  std::vector<Observation> obs;
  for (int k = 0; k < 20; ++k) {
    const std::vector<float> truth =
      exact_depth(*renderer, cam, pose_items(meshes, {flat}, T_true));
    Observation ob;
    ob.cam = cam;
    ob.items = pose_items(meshes, {flat}, Eigen::Isometry3d::Identity());
    ob.samples = decimate(corrupt(truth, cam, nullptr, rng), cam, 3);
    obs.push_back(std::move(ob));
  }

  const CalibResult res = solve_extrinsic(obs, *renderer);
  dump(res, T_true);

  // Observable: depth along the optical axis.
  EXPECT_NEAR(res.xi[2], 0.020, 0.003) << "t_z should be recovered";
  EXPECT_LT(res.prior_dominance[2], 0.2);

  // Starved: must stay at nominal (0), NOT at the true 15 mm / -10 mm, and must
  // report itself as prior-dominated rather than as a confident answer.
  for (int i : {0, 1, 5}) {
    EXPECT_LT(std::fabs(res.xi[i]), 0.002) << "DoF " << i << " drifted off nominal";
    EXPECT_GT(res.prior_dominance[i], 0.9) << "DoF " << i << " claims false confidence";
  }
}

// No offset to find and no reason to move: the estimator must not manufacture one.
TEST(Calibration, StaysAtNominalWhenAlreadyCalibrated)
{
  auto renderer = make_cpu_renderer();
  const std::vector<Mesh> meshes = {
    make_box(0.10f, 0.055f, 0.055f),
    make_box(0.085f, 0.05f, 0.05f),
    make_box(0.06f, 0.045f, 0.045f)};
  const Mesh occ_mesh = make_box(0.05f, 0.05f, 0.02f);
  const Occluder occluder{&occ_mesh, {0.10f, 0.08f, 0.55f}, 3};

  const auto obs = make_stream(
    meshes, occluder, Eigen::Isometry3d::Identity(), *renderer, 30, 99u);
  const CalibResult res = solve_extrinsic(obs, *renderer);
  dump(res, Eigen::Isometry3d::Identity());

  EXPECT_LT(res.T_fix.translation().norm(), 0.002);
  EXPECT_LT(Eigen::AngleAxisd(res.T_fix.linear()).angle(), 0.15 * kDeg);
}

TEST(Calibration, NoObservationsLeavesEstimateUntouched)
{
  auto renderer = make_cpu_renderer();
  const CalibResult res = solve_extrinsic({}, *renderer);
  EXPECT_FALSE(res.converged);
  EXPECT_EQ(res.residual_count, 0u);
  EXPECT_TRUE(res.T_fix.isApprox(Eigen::Isometry3d::Identity()));
  // With no data every DoF is pure prior.
  for (int i = 0; i < 6; ++i) EXPECT_NEAR(res.prior_dominance[i], 1.0, 1e-9);
}

// --- Streaming: recursive SE(3) observer --------------------------------------

// The observer must reach the same answer the batch solve does, one frame at a
// time, without ever holding more than a 6x6.
TEST(Observer, ConvergesOnAStreamAndAgreesWithBatch)
{
  auto renderer = make_cpu_renderer();
  const std::vector<Mesh> meshes = {
    make_box(0.10f, 0.055f, 0.055f),
    make_box(0.085f, 0.05f, 0.05f),
    make_box(0.06f, 0.045f, 0.045f)};

  const Eigen::Isometry3d T_true = correction(0.012, -0.008, 0.020, 0.5, 1.2, -0.8);
  const auto stream = make_stream(meshes, Occluder{}, T_true, *renderer, 60, 2024u);

  ObserverConfig ocfg;
  ocfg.forget_tau_s = 0.0;   // accumulate everything, for a like-for-like batch comparison
  ExtrinsicObserver obs(*renderer, CalibConfig(), ocfg);
  for (const auto & frame : stream) obs.update(frame, 0.0);

  const CalibResult & res = obs.state();
  dump(res, T_true);

  const Eigen::Isometry3d err = res.T_fix.inverse() * T_true;
  EXPECT_GT(res.frames_used, 50u);
  EXPECT_LT(err.translation().norm(), 0.003)
    << "translation error " << err.translation().norm() * 1e3 << " mm";
  EXPECT_LT(Eigen::AngleAxisd(err.linear()).angle(), 0.2 * kDeg);

  // And it should land essentially where the batch solve over the same frames does.
  const CalibResult batch = solve_extrinsic(stream, *renderer);
  const Eigen::Isometry3d gap = res.T_fix.inverse() * batch.T_fix;
  EXPECT_LT(gap.translation().norm(), 0.001)
    << "observer/batch gap " << gap.translation().norm() * 1e3 << " mm";
  EXPECT_LT(Eigen::AngleAxisd(gap.linear()).angle(), 0.05 * kDeg);
}

// A mount that physically shifts mid-stream must be re-learned — and without
// giving up the long averaging window that makes the estimate precise. That is
// what step-change detection buys: forgetting alone would force a short tau.
TEST(Observer, TracksAMountShiftMidStream)
{
  auto renderer = make_cpu_renderer();
  const std::vector<Mesh> meshes = {
    make_box(0.10f, 0.055f, 0.055f),
    make_box(0.085f, 0.05f, 0.05f),
    make_box(0.06f, 0.045f, 0.045f)};

  const Eigen::Isometry3d T_a = correction(0.012, -0.008, 0.020, 0.5, 1.2, -0.8);
  const Eigen::Isometry3d T_b = correction(-0.006, 0.011, 0.004, -0.4, 0.3, 0.9);
  const auto before = make_stream(meshes, Occluder{}, T_a, *renderer, 60, 11u);
  const auto after = make_stream(meshes, Occluder{}, T_b, *renderer, 60, 22u);

  // Default tau is 300 s — at dt = 0.1 s that is 3000 frames of memory, far too
  // much inertia to track a step by forgetting alone. The detector handles it.
  ExtrinsicObserver obs(*renderer);

  for (const auto & f : before) obs.update(f, 0.1);
  EXPECT_LT((obs.state().T_fix.inverse() * T_a).translation().norm(), 0.003)
    << "should have settled on the first mount";
  EXPECT_EQ(obs.jump_resets(), 0u) << "no step change happened yet";
  EXPECT_LT(obs.state().median_normalised_residual, 1.0)
    << "a converged fit should sit well under the jump threshold";

  for (const auto & f : after) obs.update(f, 0.1);
  dump(obs.state(), T_b);

  const Eigen::Isometry3d err = obs.state().T_fix.inverse() * T_b;
  EXPECT_GE(obs.jump_resets(), 1u) << "the shift should have been detected";
  EXPECT_LT(err.translation().norm(), 0.003)
    << "did not re-learn the shifted mount: " << err.translation().norm() * 1e3 << " mm";
  EXPECT_LT(Eigen::AngleAxisd(err.linear()).angle(), 0.2 * kDeg);
}

// Frame gates apply identically in streaming: a refused frame returns false and
// leaves the estimate alone.
TEST(Observer, RefusesOccludedAndOutOfViewFrames)
{
  auto renderer = make_cpu_renderer();
  const std::vector<Mesh> meshes = {
    make_box(0.10f, 0.055f, 0.055f),
    make_box(0.085f, 0.05f, 0.05f),
    make_box(0.06f, 0.045f, 0.045f)};
  const Mesh occ_mesh = make_box(0.45f, 0.35f, 0.02f);
  const Occluder blanket{&occ_mesh, {0.f, 0.f, 0.62f}, 1};

  const Eigen::Isometry3d T_true = correction(0.012, -0.008, 0.020, 0.5, 1.2, -0.8);
  const auto blocked = make_stream(meshes, blanket, T_true, *renderer, 15, 33u);

  ExtrinsicObserver obs(*renderer);
  for (const auto & f : blocked) EXPECT_FALSE(obs.update(f, 0.1));

  EXPECT_EQ(obs.state().frames_used, 0u);
  EXPECT_LT(obs.state().T_fix.translation().norm(), 1e-9);
  for (int i = 0; i < 6; ++i) EXPECT_NEAR(obs.state().prior_dominance[i], 1.0, 1e-9);

  // Clean frames afterwards are absorbed normally.
  const auto clean = make_stream(meshes, Occluder{}, T_true, *renderer, 40, 44u);
  for (const auto & f : clean) obs.update(f, 0.1);
  EXPECT_GT(obs.state().frames_used, 30u);
  EXPECT_LT((obs.state().T_fix.inverse() * T_true).translation().norm(), 0.003);
}

// Starvation survives the streaming form: forgetting decays data information, but
// the prior is held outside it, so a never-observed direction stays pinned at
// nominal however long the observer runs.
TEST(Observer, StarvedDofsStayAtNominalForever)
{
  auto renderer = make_cpu_renderer();
  const std::vector<Mesh> meshes = {make_box(0.55f, 0.42f, 0.05f)};
  const CameraModel cam = make_cam();
  Eigen::Isometry3f flat = Eigen::Isometry3f::Identity();
  flat.translation() = Eigen::Vector3f(0.f, 0.f, 1.5f);

  const Eigen::Isometry3d T_true = correction(0.015, -0.010, 0.020, 0.0, 0.0, 0.0);
  std::mt19937 rng(7u);

  ObserverConfig ocfg;
  ocfg.forget_tau_s = 3.0;
  ExtrinsicObserver obs(*renderer, CalibConfig(), ocfg);

  for (int k = 0; k < 60; ++k) {
    const std::vector<float> truth =
      exact_depth(*renderer, cam, pose_items(meshes, {flat}, T_true));
    Observation ob;
    ob.cam = cam;
    ob.items = pose_items(meshes, {flat}, Eigen::Isometry3d::Identity());
    ob.samples = decimate(corrupt(truth, cam, nullptr, rng), cam, 3);
    obs.update(ob, 0.1);
  }
  dump(obs.state(), T_true);

  EXPECT_NEAR(obs.state().xi[2], 0.020, 0.003) << "t_z should be recovered";
  for (int i : {0, 1, 5}) {
    EXPECT_LT(std::fabs(obs.state().xi[i]), 0.002) << "DoF " << i << " drifted off nominal";
    EXPECT_GT(obs.state().prior_dominance[i], 0.9) << "DoF " << i << " claims false confidence";
  }
}

// The EGL backend has no plane output, so the estimator falls back to
// finite-differencing the rendered depth. That path must still work — and the
// gap between the two is worth pinning, because it is what the GPU path gives up
// until someone adds a second colour attachment.
TEST(Calibration, FallsBackToFiniteDifferencesWhenPlanesAreUnavailable)
{
  auto exact = make_cpu_renderer();
  NoPlaneRenderer fallback(make_cpu_renderer());
  const std::vector<Mesh> meshes = {
    make_box(0.10f, 0.055f, 0.055f),
    make_box(0.085f, 0.05f, 0.05f),
    make_box(0.06f, 0.045f, 0.045f)};

  const Eigen::Isometry3d T_true = correction(0.012, -0.008, 0.020, 0.5, 1.2, -0.8);
  // Ground truth is exact geometry either way — a real sensor sees the true
  // surface regardless of which backend we later render it with.
  const auto obs = make_stream(meshes, Occluder{}, T_true, *exact, 40, 1234u);

  const CalibResult with_planes = solve_extrinsic(obs, *exact);
  const CalibResult without = solve_extrinsic(obs, fallback);
  dump(without, T_true);

  EXPECT_LT((with_planes.T_fix.inverse() * T_true).translation().norm(), 0.001);

  // Still converges, but carries the rasterizer's linear-in-screen z bias.
  const double err = (without.T_fix.inverse() * T_true).translation().norm();
  EXPECT_LT(err, 0.004) << "fallback should still converge: " << err * 1e3 << " mm";
  EXPECT_GT(err, (with_planes.T_fix.inverse() * T_true).translation().norm())
    << "exact planes should not be worse than finite differences";
}

// A frame displaced AS A BLOCK — a TF glitch, a bad stamp — has a small residual
// spread despite a huge offset. The robust scale is a MAD about the median, so it
// inflates to fit, the Tukey window widens, and every sample in a garbage frame
// scores as an inlier. Found on real ToF data: frames with a 1 m median error
// passing at ~60% inliers. The Tukey loss guards spread; only an explicit offset
// gate guards this.
TEST(Observer, RejectsBlockDisplacedFrames)
{
  auto renderer = make_cpu_renderer();
  const std::vector<Mesh> meshes = {
    make_box(0.10f, 0.055f, 0.055f),
    make_box(0.085f, 0.05f, 0.05f),
    make_box(0.06f, 0.045f, 0.045f)};

  const Eigen::Isometry3d T_true = correction(0.012, -0.008, 0.020, 0.5, 1.2, -0.8);
  auto stream = make_stream(meshes, Occluder{}, T_true, *renderer, 60, 4242u);

  // Settle on clean frames first.
  ExtrinsicObserver obs(*renderer);
  for (int k = 0; k < 40; ++k) obs.update(stream[k], 0.1);
  const Eigen::Isometry3d settled = obs.state().T_fix;
  ASSERT_LT((settled.inverse() * T_true).translation().norm(), 0.003);

  // Stretch whole frames in depth, which is what a bad TF or a wrong depth scale
  // actually looks like: a large offset that also has a large SPREAD. Both parts
  // matter. A uniform shift keeps the spread tiny, so the Tukey loss rejects every
  // sample and the frame dies earlier; and a uniformly NEARER frame is caught by
  // the occlusion gate. Only offset-with-spread widens the robust window enough
  // for a garbage frame to score inliers — the case seen on real ToF data
  // (median |r|/sigma 19.5 at robust scale 13.5, ~60% inliers).
  // Note the offset alone is not enough even when stretched: sigma(z) grows as
  // z^2, so a pure scale error leaves |r|/sigma nearly CONSTANT across the frame
  // and the robust scale stays ~1. Real garbage frames carry independent scatter
  // on top, which is what inflates the MAD and opens the window.
  std::mt19937 bad_rng(1u);
  std::normal_distribution<float> scatter(0.f, 0.15f);
  for (int k = 40; k < 60; ++k) {
    Observation bad = stream[k];
    for (auto & s : bad.samples) s.z = s.z * 1.6f + scatter(bad_rng);
    EXPECT_FALSE(obs.update(bad, 0.1)) << "block-displaced frame " << k << " was accepted";
  }

  // Every bad frame is refused (asserted above); the gates are defence in depth,
  // so some are claimed earlier by the occlusion or Tukey paths. What matters here
  // is that the offset gate catches the ones that get past those.
  EXPECT_GE(obs.state().frames_rejected_residual, 5u)
    << "the block-displacement gate never fired";
  // The estimate must be untouched by them.
  const Eigen::Isometry3d after = obs.state().T_fix;
  EXPECT_LT((settled.inverse() * after).translation().norm(), 1e-6);
  EXPECT_LT((after.inverse() * T_true).translation().norm(), 0.003);
}
