// Extrinsic self-calibration. See CALIBRATION.md for the design.
//
// Residual (point-to-plane, projective association):
//
//   r_i(xi) = n_i . ( Q_i - retract(xi) P_i )
//   dr/dxi  = [ -n_i^T , -(P_i x n_i)^T ]
//
// P_i and n_i come from the RENDER (exact); only Q_i is measured. Nothing here
// differentiates the measured depth image, which is what lets the estimator work
// on very noisy input: zero-mean depth noise averages out of the normal
// equations instead of biasing them.
//
// Two front-ends share that machinery: a batch solve over a set of observations,
// and a recursive SE(3) observer for streaming. They accumulate the same
// quantities; the observer just never stores the frames.
#include "depth_self_filter/calibration.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace depth_self_filter
{
namespace
{

/// A gated model/measurement pair for one pixel, in the camera frame at the pose
/// used for the render that produced it.
///
/// The model side is a PLANE, not a point: `n . X = d`. The measurement side is
/// the raw scalar depth. Nothing here back-projects the measurement — the noisy
/// quantity stays the scalar the sensor actually produced, which is what makes
/// its variance exactly sigma(z)^2 with no projection factor to get wrong.
struct Corr
{
  Eigen::Vector3d n;      ///< unit plane normal, facing the camera
  double d;               ///< plane offset: n . X = d
  Eigen::Vector3d v;      ///< pixel ray, normalised so v.z == 1
  double z_meas;          ///< measured depth, metres
  double inv_sigma;       ///< 1 / sigma(z) from the sensor noise model
  double confidence;      ///< amplitude/confidence weight
};

struct FrameStats
{
  std::size_t offered = 0;
  std::size_t on_model = 0;
  std::size_t occluded = 0;
};

/// Robust normal equations for a correspondence set.
struct NormalEqs
{
  Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 1> g = Eigen::Matrix<double, 6, 1>::Zero();
  std::size_t inliers = 0;
  double scale = 1.0;         ///< MAD scale of noise-normalised residuals (centred)
  double median_abs = 0.0;    ///< median |r| in metres
  double median_norm_abs = 0.0;  ///< median |r| / sigma, UNcentred: how many sigma off
};

Eigen::Vector3d backproject(const CameraModel & cam, int u, int v, double z)
{
  // Pixel centres, matching the rasterizer's (x + 0.5, y + 0.5) sampling.
  const double x = (u + 0.5 - cam.cx) / cam.fx * z;
  const double y = (v + 0.5 - cam.cy) / cam.fy * z;
  return Eigen::Vector3d(x, y, z);
}

/// True when any pixel within `radius` of (u,v) is background or jumps in depth,
/// i.e. (u,v) sits on or near a silhouette / self-occlusion boundary. Those are
/// exactly where flying pixels and mixed returns live in the measurement, and
/// where the rendered normal is meaningless.
bool near_depth_edge(
  const std::vector<float> & depth, int w, int h, int u, int v, int radius,
  double jump_m)
{
  const double d0 = depth[static_cast<size_t>(v) * w + u];
  for (int dy = -radius; dy <= radius; ++dy) {
    const int yy = v + dy;
    if (yy < 0 || yy >= h) return true;
    for (int dx = -radius; dx <= radius; ++dx) {
      const int xx = u + dx;
      if (xx < 0 || xx >= w) return true;
      const double d = depth[static_cast<size_t>(yy) * w + xx];
      if (d <= 0.0) return true;                    // background: silhouette
      if (std::fabs(d - d0) > jump_m) return true;  // depth step
    }
  }
  return false;
}

/// Surface normal from the rendered depth by central differences on the
/// back-projected neighbourhood, oriented towards the camera. Callers must have
/// already ruled out depth edges, so the neighbours are on the same surface.
Eigen::Vector3d rendered_normal(
  const CameraModel & cam, const std::vector<float> & depth, int w, int u, int v)
{
  const auto at = [&](int x, int y) {
    return backproject(cam, x, y, depth[static_cast<size_t>(y) * w + x]);
  };
  const Eigen::Vector3d du = at(u + 1, v) - at(u - 1, v);
  const Eigen::Vector3d dv = at(u, v + 1) - at(u, v - 1);
  Eigen::Vector3d n = du.cross(dv);
  const double norm = n.norm();
  if (norm < 1e-12) return Eigen::Vector3d::Zero();
  n /= norm;
  // Optical frame: +z forward. A surface we can see faces back at the camera.
  if (n.dot(at(u, v)) > 0.0) n = -n;
  return n;
}

double median_of(std::vector<double> & v)
{
  if (v.empty()) return 0.0;
  const size_t mid = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + mid, v.end());
  return v[mid];
}

/// Render ONE observation at `T_fix` and build its gated correspondence set.
///
/// Acceptance is per FRAME, not per pixel: a frame where the robot is barely in
/// view, or where something real is standing in front of it, is discarded whole.
/// Returns false when the frame is refused, in which case `out` is meaningless.
bool associate_frame(
  const Observation & o, IRenderer & renderer, const Eigen::Isometry3d & T_fix,
  const CalibConfig & cfg, std::vector<Corr> & out, FrameStats & fs)
{
  out.clear();
  fs = FrameStats{};
  if (o.cam.width <= 0 || o.cam.height <= 0) return false;
  fs.offered = o.samples.size();

  std::vector<RenderItem> items;
  items.reserve(o.items.size());
  for (const auto & it : o.items) {
    RenderItem c = it;
    c.T_cam_link = (T_fix * it.T_cam_link.cast<double>()).cast<float>();
    items.push_back(c);
  }

  // Prefer the analytic plane straight from the rasterizer. Falling back to
  // finite-differencing the depth image costs accuracy twice over: the normal
  // becomes a numerical estimate, and z_pred inherits the rasterizer's
  // linear-in-screen z interpolation, which is an approximation even for a plane.
  std::vector<float> depth, planes;
  const bool have_planes = renderer.render_planes(o.cam, items, depth, planes);
  if (!have_planes && !renderer.render(o.cam, items, depth)) return false;

  const int radius = std::max(1, cfg.edge_reject_px);
  const int w = o.cam.width, h = o.cam.height;

  for (const auto & s : o.samples) {
    if (s.z <= 0.f || !std::isfinite(s.z)) continue;
    if (s.u < radius || s.u >= w - radius || s.v < radius || s.v >= h - radius) continue;

    const size_t idx = static_cast<size_t>(s.v) * w + s.u;
    const double zr = depth[idx];
    if (zr <= 0.0) continue;  // no robot rendered here
    ++fs.on_model;

    // Occlusion is one-sided, and it is a property of the frame rather than of
    // the pixel: tested first so the fraction below is measured against every
    // on-model sample, not just the ones that survived the other gates.
    if (s.z < zr - cfg.occlusion_margin_m) { ++fs.occluded; continue; }

    // Still gated on render depth edges, but now only to protect the MEASUREMENT:
    // silhouettes are where flying pixels and mixed returns live. With planes it
    // is no longer also propping up a finite-difference stencil.
    if (near_depth_edge(depth, w, h, s.u, s.v, radius, cfg.edge_depth_jump_m)) continue;

    // Pixel ray, scaled so v.z == 1; depth along it is exactly the z coordinate.
    const Eigen::Vector3d v(
      (s.u + 0.5 - o.cam.cx) / o.cam.fx,
      (s.v + 0.5 - o.cam.cy) / o.cam.fy,
      1.0);

    Eigen::Vector3d n;
    double d = 0.0;
    if (have_planes) {
      n = Eigen::Vector3d(planes[idx * 4 + 0], planes[idx * 4 + 1], planes[idx * 4 + 2]);
      if (n.squaredNorm() < 0.5) continue;
      d = planes[idx * 4 + 3];
    } else {
      n = rendered_normal(o.cam, depth, w, s.u, s.v);
      if (n.squaredNorm() < 0.5) continue;
      d = n.dot(zr * v);   // plane through the back-projected rendered point
    }

    // Grazing incidence: unreliable on every depth sensor, and it is also where
    // the ray-plane intersection becomes ill-conditioned (n . v -> 0).
    const double den = n.dot(v);
    if (std::fabs(den) / v.norm() < cfg.grazing_min_cos) continue;

    const double sigma = cfg.noise_a + cfg.noise_b * s.z * s.z;
    if (!(sigma > 0.0)) continue;
    out.push_back(Corr{n, d, v, s.z, 1.0 / sigma, std::max(0.0, double(s.confidence))});
  }

  // Frame gates. Not in view, occluded, or too little usable -> skip the frame
  // entirely rather than fit to a biased slice of the robot.
  const double on_model = static_cast<double>(fs.on_model);
  if (fs.on_model < cfg.min_model_samples) return false;
  if (static_cast<double>(fs.occluded) > cfg.max_occluded_fraction * on_model) return false;
  if (static_cast<double>(out.size()) < cfg.min_usable_fraction * on_model) return false;
  return true;
}

/// Robust (IRLS) normal equations for `corrs`, evaluated after applying
/// `T_local` to the model points they were associated with.
NormalEqs build_normal_equations(
  const std::vector<Corr> & corrs, const Eigen::Isometry3d & T_local,
  const CalibConfig & cfg)
{
  NormalEqs ne;
  if (corrs.empty()) return ne;

  // Residual in DEPTH space: intersect the pixel ray with the (perturbed) model
  // plane and compare against the measured depth.
  //
  //   n' = R n,  d' = d + n'.t,  z_pred = d' / (n'.v),  r = z_meas - z_pred
  //
  // This is the point-to-plane residual divided by (n.v) — the same information,
  // but expressed in the units the sensor actually measures, so Var(r) is exactly
  // sigma(z)^2. The point-to-plane form needs a (n.v)^2 factor in its variance to
  // say the same thing, and getting that factor wrong silently mis-weights every
  // slanted surface. At the identity perturbation this reduces to z_meas - z_render.
  const Eigen::Matrix3d R_local = T_local.linear();
  const Eigen::Vector3d t_local = T_local.translation();

  const auto predict = [&](const Corr & c, Eigen::Vector3d & n_out, double & den_out) {
    n_out = R_local * c.n;
    den_out = n_out.dot(c.v);
    return (c.d + n_out.dot(t_local)) / den_out;   // z_pred
  };

  std::vector<double> r_all(corrs.size(), 0.0);
  std::vector<double> e_signed(corrs.size(), 0.0);
  std::vector<uint8_t> ok(corrs.size(), 0);

  for (size_t i = 0; i < corrs.size(); ++i) {
    Eigen::Vector3d n1; double den;
    const double z_pred = predict(corrs[i], n1, den);
    if (std::fabs(den) < 1e-9 || !std::isfinite(z_pred)) continue;
    r_all[i] = corrs[i].z_meas - z_pred;
    e_signed[i] = r_all[i] * corrs[i].inv_sigma;
    ok[i] = 1;
  }

  // Robust scale = MAD of the noise-normalised residuals ABOUT THEIR MEDIAN.
  //
  // Centring matters. median(|e|) would conflate two different things: residuals
  // spread wide because the sensor is noisier than the model (a real reason to
  // deflate the information), and residuals displaced together because the
  // estimate is currently wrong (a transient, and precisely the signal we are
  // trying to fit). Using the uncentred form, a 3-sigma systematic offset reads
  // as scale ~ 4.4, which both widens the Tukey window and deflates the frame's
  // information by ~20x — so the observer suppresses new evidence exactly when it
  // is most wrong, and a mount shift takes hundreds of frames to be re-learned
  // instead of tens. Centred, an offset leaves the scale at ~1 where it belongs.
  {
    std::vector<double> tmp;
    tmp.reserve(corrs.size());
    for (size_t i = 0; i < corrs.size(); ++i) if (ok[i]) tmp.push_back(e_signed[i]);
    if (tmp.empty()) return ne;
    const double med = median_of(tmp);
    std::vector<double> centred;
    centred.reserve(tmp.size());
    for (size_t i = 0; i < corrs.size(); ++i) {
      if (ok[i]) centred.push_back(std::fabs(e_signed[i] - med));
    }
    // Floored at 1: the data is never allowed to claim it is tighter than the
    // noise model says, which stops the weights collapsing onto a spurious minimum.
    ne.scale = std::max(1.0, 1.4826 * median_of(centred));
  }

  // The Tukey window stays centred on ZERO — the target is a zero residual, not
  // the current median — while its width comes from the centred scale above.
  const double k = cfg.tukey_k * ne.scale;
  for (size_t i = 0; i < corrs.size(); ++i) {
    if (!ok[i]) continue;
    const Corr & c = corrs[i];
    const double e = e_signed[i];
    if (std::fabs(e) >= k) continue;              // Tukey: zero weight
    const double u = e / k;
    const double tukey = (1.0 - u * u) * (1.0 - u * u);
    const double W = tukey * c.confidence * c.inv_sigma * c.inv_sigma;
    if (W <= 0.0) continue;
    ++ne.inliers;

    Eigen::Vector3d n1; double den;
    const double z_pred = predict(c, n1, den);
    Eigen::Matrix<double, 6, 1> J;
    J.head<3>() = -n1 / den;
    J.tail<3>() = z_pred * n1.cross(c.v) / den;
    ne.H.noalias() += W * J * J.transpose();
    ne.g.noalias() += W * J * r_all[i];
  }

  std::vector<double> abs_r;
  abs_r.reserve(corrs.size());
  for (size_t i = 0; i < corrs.size(); ++i) if (ok[i]) abs_r.push_back(std::fabs(r_all[i]));
  ne.median_abs = median_of(abs_r);
  // Uncentred: a frame displaced as a block is exactly what this must see.
  abs_r.clear();
  for (size_t i = 0; i < corrs.size(); ++i) if (ok[i]) abs_r.push_back(std::fabs(e_signed[i]));
  ne.median_norm_abs = median_of(abs_r);
  return ne;
}

void make_prior(
  const CalibConfig & cfg, Eigen::Matrix<double, 6, 1> & sigma,
  Eigen::Matrix<double, 6, 6> & H)
{
  sigma << cfg.prior_sigma_xyz, cfg.prior_sigma_xyz, cfg.prior_sigma_xyz,
           cfg.prior_sigma_rot, cfg.prior_sigma_rot, cfg.prior_sigma_rot;
  H = sigma.array().square().inverse().matrix().asDiagonal();
}

void fill_uncertainty(
  CalibResult & res, const Eigen::Matrix<double, 6, 6> & H_total,
  const Eigen::Matrix<double, 6, 1> & prior_sigma)
{
  res.information = H_total;
  const Eigen::Matrix<double, 6, 6> cov = H_total.inverse();
  for (int i = 0; i < 6; ++i) {
    res.sigma[i] = std::sqrt(std::max(0.0, cov(i, i)));
    res.prior_dominance[i] = res.sigma[i] / prior_sigma[i];
  }
}

}  // namespace

Eigen::Isometry3d se3_retract(const Eigen::Matrix<double, 6, 1> & xi)
{
  const Eigen::Vector3d t = xi.head<3>();
  const Eigen::Vector3d w = xi.tail<3>();
  const double theta = w.norm();
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = (theta < 1e-12)
    ? Eigen::Matrix3d::Identity()
    : Eigen::Matrix3d(Eigen::AngleAxisd(theta, w / theta));
  T.translation() = t;
  return T;
}

Eigen::Matrix<double, 6, 1> se3_params(const Eigen::Isometry3d & T)
{
  const Eigen::AngleAxisd aa(T.linear());
  Eigen::Matrix<double, 6, 1> xi;
  xi.head<3>() = T.translation();
  xi.tail<3>() = aa.angle() * aa.axis();
  return xi;
}

Eigen::Matrix<double, 6, 1> to_xyz_rpy(const Eigen::Isometry3d & T)
{
  // ROS convention: R = Rz(yaw) * Ry(pitch) * Rx(roll), extracted directly.
  //
  // NOT Eigen's eulerAngles(2,1,0): that returns angles in a canonical range and
  // near identity happily reports (pi, pi, pi) — the same rotation, but nonsense
  // to write into a params file a human will read. Observed on real data.
  const Eigen::Matrix3d & R = T.linear();
  const double sy = std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
  Eigen::Matrix<double, 6, 1> out;
  out.head<3>() = T.translation();
  if (sy > 1e-9) {
    out[3] = std::atan2(R(2, 1), R(2, 2));   // roll
    out[4] = std::atan2(-R(2, 0), sy);       // pitch
    out[5] = std::atan2(R(1, 0), R(0, 0));   // yaw
  } else {                                    // gimbal lock: pitch = +/-90 deg
    out[3] = std::atan2(-R(1, 2), R(1, 1));
    out[4] = std::atan2(-R(2, 0), sy);
    out[5] = 0.0;
  }
  return out;
}

CalibResult solve_extrinsic(
  const std::vector<Observation> & obs, IRenderer & renderer,
  const CalibConfig & cfg, const Eigen::Isometry3d & T_init)
{
  CalibResult res;
  res.T_fix = T_init;

  // MAP prior, centred on the URDF nominal (identity). Unobserved directions are
  // held here rather than being driven by noise, which is what makes the estimate
  // safe to apply before the robot has happened to move through informative poses.
  Eigen::Matrix<double, 6, 1> prior_sigma;
  Eigen::Matrix<double, 6, 6> H_prior;
  make_prior(cfg, prior_sigma, H_prior);
  res.information = H_prior;
  res.sigma = prior_sigma;

  std::vector<Corr> corrs, frame;

  for (int outer = 0; outer < cfg.max_outer; ++outer) {
    const Eigen::Isometry3d T_render = res.T_fix;
    res.outer_iterations = outer + 1;

    corrs.clear();
    res.frames_total = obs.size();
    res.frames_used = 0;
    res.sample_count = 0;
    res.model_sample_count = 0;
    for (const auto & o : obs) {
      FrameStats fs;
      const bool ok = associate_frame(o, renderer, T_render, cfg, frame, fs);
      res.sample_count += fs.offered;
      if (!ok) continue;
      ++res.frames_used;
      res.model_sample_count += fs.on_model;
      corrs.insert(corrs.end(), frame.begin(), frame.end());
    }
    if (corrs.size() < 6) break;

    // Inner IRLS: correspondences frozen, weights and step re-estimated.
    Eigen::Isometry3d T_local = Eigen::Isometry3d::Identity();
    NormalEqs ne;
    for (int inner = 0; inner < cfg.max_inner; ++inner) {
      ne = build_normal_equations(corrs, T_local, cfg);
      if (ne.inliers < 6) break;

      // MAP normal equations. xi_cur is the total correction from nominal, so the
      // prior pulls the accumulated estimate, not just this step.
      const Eigen::Matrix<double, 6, 1> xi_cur = se3_params(T_local * T_render);
      const Eigen::Matrix<double, 6, 6> H = ne.H + H_prior;
      const Eigen::Matrix<double, 6, 1> step =
        H.ldlt().solve(-(ne.g + H_prior * xi_cur));
      if (!step.allFinite()) break;
      T_local = se3_retract(step) * T_local;
      if (step.norm() < cfg.converge_xi_norm) break;
    }

    res.T_fix = T_local * T_render;
    res.xi = se3_params(res.T_fix);
    res.residual_count = corrs.size();
    res.robust_scale = ne.scale;
    res.median_abs_residual_m = ne.median_abs;
    res.median_normalised_residual = ne.median_norm_abs;
    res.inlier_fraction = res.model_sample_count
      ? static_cast<double>(ne.inliers) / static_cast<double>(res.model_sample_count) : 0.0;
    // Report covariance from the noise-weighted information, deflated when the
    // residuals are wider than the noise model predicted, so sigma stays honest.
    fill_uncertainty(res, ne.H / (ne.scale * ne.scale) + H_prior, prior_sigma);

    if (se3_params(T_local).norm() < cfg.converge_xi_norm) {
      res.converged = true;
      break;
    }
  }

  return res;
}

// --- Streaming: recursive SE(3) observer --------------------------------------

ExtrinsicObserver::ExtrinsicObserver(
  IRenderer & renderer, const CalibConfig & cfg, const ObserverConfig & ocfg)
: renderer_(renderer), cfg_(cfg), ocfg_(ocfg)
{
  make_prior(cfg_, prior_sigma_, H_prior_);
  reset();
}

void ExtrinsicObserver::reset(const Eigen::Isometry3d & T_init)
{
  H_data_.setZero();
  inconsistent_run_ = 0;
  state_ = CalibResult{};
  state_.T_fix = T_init;
  state_.xi = se3_params(T_init);
  fill_uncertainty(state_, H_prior_, prior_sigma_);
}

// Step-change detection. A whole frame displaced together is a mount that moved,
// not noise; once that persists, the accumulated information describes a camera
// pose that no longer exists and is worth less than nothing.
void ExtrinsicObserver::note_consistency(double median_norm_abs)
{
  if (median_norm_abs > ocfg_.jump_sigma) {
    ++inconsistent_run_;
    if (inconsistent_run_ >= std::max(1, ocfg_.jump_frames)) {
      H_data_.setZero();     // keep T_fix and the prior; drop the stale evidence
      inconsistent_run_ = 0;
      ++jump_resets_;
    }
  } else {
    inconsistent_run_ = 0;
  }
}

bool ExtrinsicObserver::update(const Observation & obs, double dt_s)
{
  ++state_.frames_total;

  // Time update: let old evidence fade so a mount that physically shifts is
  // re-learned rather than outvoted by history. The prior is deliberately not
  // part of H_data_, so forgetting can never erode the anchor that holds
  // never-observed directions at nominal.
  if (dt_s > 0.0 && ocfg_.forget_tau_s > 0.0) {
    H_data_ *= std::exp(-dt_s / ocfg_.forget_tau_s);
  }

  std::vector<Corr> corrs;
  FrameStats fs;

  // Iterated update: re-associate at the improved estimate, but commit the
  // measurement's information only once — it is one frame however many times we
  // relinearize against it.
  Eigen::Matrix<double, 6, 6> dH = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 1> delta_cum = Eigen::Matrix<double, 6, 1>::Zero();
  NormalEqs ne;
  bool accepted = false;
  Eigen::Matrix<double, 6, 1> last_step = Eigen::Matrix<double, 6, 1>::Zero();

  for (int it = 0; it < std::max(1, ocfg_.iterations); ++it) {
    if (!associate_frame(obs, renderer_, state_.T_fix, cfg_, corrs, fs)) {
      if (it == 0) {
        state_.sample_count = fs.offered;
        return false;  // frame refused; estimate untouched
      }
      break;           // moved out of the gates mid-iteration: keep what we have
    }
    ne = build_normal_equations(corrs, Eigen::Isometry3d::Identity(), cfg_);
    if (ne.inliers < 6) break;
    // Block-displaced frame: do not let it move the estimate, but do let the
    // step detector see it — a mount that really moved looks like this too, and
    // must not be locked out permanently.
    if (ne.median_norm_abs > cfg_.max_frame_normalised_residual) {
      note_consistency(ne.median_norm_abs);
      state_.median_normalised_residual = ne.median_norm_abs;
      state_.median_abs_residual_m = ne.median_abs;
      state_.robust_scale = ne.scale;
      ++state_.frames_rejected_residual;
      return false;
    }
    accepted = true;
    dH = ne.H / (ne.scale * ne.scale);

    // Information-form MAP update. H_data_ is the posterior of everything absorbed
    // so far, centred where this frame's update began, so it pulls back by
    // H_data_ * delta_cum; the
    // prior is centred on the URDF nominal, so it pulls by H_prior_ * xi_current.
    const Eigen::Matrix<double, 6, 6> H_total = H_data_ + dH + H_prior_;
    const Eigen::Matrix<double, 6, 1> rhs =
      -(ne.g / (ne.scale * ne.scale) + H_data_ * delta_cum +
        H_prior_ * se3_params(state_.T_fix));
    const Eigen::Matrix<double, 6, 1> step = H_total.ldlt().solve(rhs);
    if (!step.allFinite()) break;

    state_.T_fix = se3_retract(step) * state_.T_fix;
    delta_cum += step;
    last_step = step;
    if (step.norm() < cfg_.converge_xi_norm) break;
  }

  if (!accepted) {
    state_.sample_count = fs.offered;
    return false;
  }

  note_consistency(ne.median_norm_abs);

  H_data_ += dH;
  ++state_.frames_used;

  state_.xi = se3_params(state_.T_fix);
  state_.sample_count = fs.offered;
  state_.model_sample_count = fs.on_model;
  state_.residual_count = corrs.size();
  state_.robust_scale = ne.scale;
  state_.median_abs_residual_m = ne.median_abs;
  state_.median_normalised_residual = ne.median_norm_abs;
  state_.inlier_fraction = fs.on_model
    ? static_cast<double>(ne.inliers) / static_cast<double>(fs.on_model) : 0.0;
  state_.outer_iterations = std::max(1, ocfg_.iterations);
  state_.converged = last_step.norm() < cfg_.converge_xi_norm;
  fill_uncertainty(state_, H_data_ + H_prior_, prior_sigma_);
  return true;
}

}  // namespace depth_self_filter
