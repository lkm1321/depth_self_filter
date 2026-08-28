// Extrinsic self-calibration: estimate the 6-DoF correction between the camera
// optical frame as TF declares it and where the camera actually is, from
// observations of the robot's own body. See CALIBRATION.md.
//
// ROS-free by design (like the renderers) so the estimator can be tested in CI
// with no GPU, no bag and no ROS graph.
#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Geometry>

#include "depth_self_filter/renderer.hpp"

namespace depth_self_filter
{

/// One decimated measured-depth sample from an accepted self-observation.
struct DepthSample
{
  int u = 0, v = 0;         ///< pixel column, row
  float z = 0.f;            ///< measured depth in metres (> 0)
  float confidence = 1.f;   ///< optional amplitude/confidence weight, (0, 1]
};

/// One self-observation: the robot's own body seen at a known configuration.
///
/// `items` carry the link poses exactly as TF reported them, with NO correction
/// applied — the estimator composes its current T_fix when it re-renders. That
/// is why correspondences are not stored: freezing them would freeze the data
/// association that existed when the observation was admitted. Meshes are
/// borrowed (RenderItem holds a raw pointer) and must outlive the observation.
struct Observation
{
  CameraModel cam;
  std::vector<RenderItem> items;
  std::vector<DepthSample> samples;
};

struct CalibConfig
{
  // --- Prior (MAP). Gaussian on the correction, centred on the URDF nominal
  // (identity). Its job is to leave unobserved directions at nominal instead of
  // letting noise drive them, so the estimate is safe to apply from frame one.
  double prior_sigma_xyz = 0.02;    ///< m; expected mounting error
  double prior_sigma_rot = 0.035;   ///< rad (~2 deg)

  // --- Sensor noise model: sigma(z) = noise_a + noise_b * z^2 (ToF-like).
  double noise_a = 0.005;           ///< m
  double noise_b = 0.005;           ///< m per m^2

  // --- Per-sample gates (see CALIBRATION.md "Observation intake").
  int edge_reject_px = 3;           ///< reject within this many px of a render depth edge
  double edge_depth_jump_m = 0.05;  ///< what counts as a depth discontinuity
  double occlusion_margin_m = 0.05; ///< measured this much nearer than model => occluded
  double grazing_min_cos = 0.2;     ///< reject |n . ray| below this

  // --- Frame-level gates. If the robot is not meaningfully in view, or is being
  // occluded by something real, the frame is DROPPED rather than salvaged. A
  // partially usable frame contributes a spatially truncated, leverage-skewed
  // slice of the robot, which biases the fit while still looking well-conditioned
  // — worse than simply waiting for a clean frame, and passive operation supplies
  // plenty of those.
  std::size_t min_model_samples = 150;  ///< too little robot in view => not an observation
  double max_occluded_fraction = 0.15;  ///< drop the frame once this much of it is occluded
  double min_usable_fraction = 0.40;    ///< of on-model samples surviving every gate
  /// Reject a frame whose residuals are displaced AS A BLOCK by more than this
  /// many sigma (median |r|/sigma). Necessary because the robust scale is a MAD
  /// about the median: a frame that is entirely wrong — a TF glitch, a bad stamp —
  /// has a small spread despite a huge offset, so the Tukey window widens to
  /// accommodate it and every sample in a garbage frame scores as an inlier.
  /// Observed on real ToF data: frames with a 1 m median error passing at 60%
  /// inliers and dragging the estimate. The Tukey loss guards spread; this guards
  /// offset. A SUSTAINED offset is a mount change, not a bad frame, so rejected
  /// frames still feed step-change detection.
  double max_frame_normalised_residual = 6.0;

  // --- Robust loss / solver.
  double tukey_k = 4.685;           ///< Tukey constant, in MAD-scaled sigma units
  int max_outer = 12;               ///< re-render / re-associate iterations
  int max_inner = 4;                ///< IRLS Gauss-Newton steps per outer iteration
  /// Stop when the outer step falls below this (mixed m / rad norm). Re-association
  /// is discrete — pixels enter and leave the gates as the estimate moves — so the
  /// fixed point is not exact and there is a limit cycle at small amplitude.
  /// 1e-4 is 0.1 mm / 0.006 deg, far below the sigma this estimator can report.
  double converge_xi_norm = 1e-4;
};

/// Estimate and diagnostics. `xi` orders translation first: (x, y, z, rx, ry, rz)
/// with the rotation as a rotation vector.
struct CalibResult
{
  Eigen::Isometry3d T_fix = Eigen::Isometry3d::Identity();
  Eigen::Matrix<double, 6, 1> xi = Eigen::Matrix<double, 6, 1>::Zero();

  Eigen::Matrix<double, 6, 6> information =
    Eigen::Matrix<double, 6, 6>::Zero();          ///< posterior: H_data + H_prior
  Eigen::Matrix<double, 6, 1> sigma =
    Eigen::Matrix<double, 6, 1>::Zero();          ///< sqrt(diag(information^-1))
  /// sigma_posterior / sigma_prior per DoF, in (0, 1]. Near 1 means the data said
  /// nothing about that direction and it is still sitting at the URDF nominal —
  /// the honest answer for a passive system that never saw the right poses.
  Eigen::Matrix<double, 6, 1> prior_dominance =
    Eigen::Matrix<double, 6, 1>::Ones();

  std::size_t frames_total = 0;        ///< observations offered
  std::size_t frames_used = 0;         ///< observations that passed the frame gates
  std::size_t frames_rejected_residual = 0;  ///< refused for a block-displaced residual
  std::size_t sample_count = 0;        ///< samples offered across all observations
  std::size_t model_sample_count = 0;  ///< of those, on rendered robot in USED frames
  std::size_t residual_count = 0;      ///< survived every gate on the final iteration
  /// Non-zero robust weight / model_sample_count. Deliberately relative to the
  /// samples that landed on the robot, not to every sample offered: most samples
  /// are background and their exclusion says nothing about observation health.
  /// This is the quantity the frame-level sanity gate thresholds on.
  double inlier_fraction = 0.0;
  double median_abs_residual_m = 0.0;
  /// Median |r| / sigma(z) — how many sigma the fit is off, uncentred. Near 0.7
  /// when converged; a sustained high value means the estimate no longer explains
  /// the data, which is what step-change detection keys on.
  double median_normalised_residual = 0.0;
  double robust_scale = 0.0;         ///< MAD scale of noise-normalised residuals (~1 if the noise model fits)
  int outer_iterations = 0;
  bool converged = false;
};

/// Solve for T_fix over `obs`. `renderer` is used to re-associate each outer
/// iteration; `T_init` warm-starts (identity = the URDF nominal).
CalibResult solve_extrinsic(
  const std::vector<Observation> & obs,
  IRenderer & renderer,
  const CalibConfig & cfg = CalibConfig(),
  const Eigen::Isometry3d & T_init = Eigen::Isometry3d::Identity());

// --- Streaming form ----------------------------------------------------------

struct ObserverConfig
{
  /// Information time constant. Data information decays by exp(-dt/tau) between
  /// frames, so the observer forgets at a controlled rate and a mount that
  /// physically shifts is re-learned instead of being outvoted by history.
  /// <= 0 disables forgetting (information accumulates without bound).
  double forget_tau_s = 300.0;
  /// Re-association passes per frame (iterated-EKF style). 1 is a plain EKF
  /// update; 2 costs one extra render and handles the nonlinearity better.
  int iterations = 2;

  // --- Step-change detection.
  //
  // Forgetting alone trades precision against tracking: after tau/dt frames of
  // accumulated information a single frame can only move the estimate by ~dt/tau,
  // so a mount that is physically knocked takes several multiples of tau to be
  // re-learned. Shortening tau to track faster throws away the averaging that
  // makes the estimate precise in the first place.
  //
  // A step change is distinguishable from noise: it displaces the whole frame's
  // residuals together, which is exactly `median_normalised_residual`. When that
  // stays high for several consecutive frames, the accumulated information is
  // discarded so new evidence takes over immediately — long tau AND fast
  // re-learning, instead of a compromise between them.
  double jump_sigma = 1.5;   ///< frame is inconsistent above this median |r|/sigma
  int jump_frames = 4;       ///< consecutive inconsistent frames => drop the memory
};

/// Recursive SE(3) observer — the streaming form of solve_extrinsic.
///
/// Keeps the estimate and a 6x6 information matrix instead of the frames. Each
/// accepted frame contributes `dH = sum W J^T J` and `dg = sum W J^T r`, exactly
/// the quantities the batch solver accumulates, so the two agree; the observer
/// simply never stores what it has already absorbed. Cost per frame is one render
/// (shared with the filter's own), the residual pass, and a 6x6 solve.
///
/// The error is parameterised as a LEFT perturbation, `T_true = retract(xi)*T_hat`
/// — the same convention the residual Jacobian is derived against. That choice is
/// what makes the update transport-free: an update `T_hat <- retract(d)*T_hat`
/// shifts the error coordinates by `-d` and leaves the covariance unchanged to
/// first order, so no adjoint appears anywhere below.
///
/// The prior is NOT folded into the accumulated information. It is re-evaluated
/// against the nominal every step, so a direction the data never illuminates
/// keeps exactly the prior's uncertainty however long the observer runs, instead
/// of slowly drifting as forgetting eats an anchor that was only applied once.
class ExtrinsicObserver
{
public:
  explicit ExtrinsicObserver(
    IRenderer & renderer,
    const CalibConfig & cfg = CalibConfig(),
    const ObserverConfig & ocfg = ObserverConfig());

  /// Absorb one frame. `dt_s` is the elapsed time since the previous call, used
  /// only for forgetting. Returns false if the frame was refused by the frame
  /// gates (robot not in view, occluded, too little usable) — the estimate is
  /// then untouched apart from the time update.
  bool update(const Observation & obs, double dt_s = 0.0);

  /// Current estimate and diagnostics. `converged` here means "the last update
  /// moved the estimate less than converge_xi_norm", i.e. the observer is at
  /// rest, not that a batch solve terminated.
  const CalibResult & state() const { return state_; }

  void reset(const Eigen::Isometry3d & T_init = Eigen::Isometry3d::Identity());

  /// Feed the step-change detector without committing information.
  void note_consistency(double median_norm_abs);

  /// How many times step-change detection has discarded the accumulated
  /// information. A climbing count means the mount is moving, or the noise model
  /// is wrong enough to look like it.
  std::size_t jump_resets() const { return jump_resets_; }

private:
  IRenderer & renderer_;
  CalibConfig cfg_;
  ObserverConfig ocfg_;
  Eigen::Matrix<double, 6, 6> H_prior_;
  Eigen::Matrix<double, 6, 1> prior_sigma_;
  Eigen::Matrix<double, 6, 6> H_data_;
  CalibResult state_;
  int inconsistent_run_ = 0;
  std::size_t jump_resets_ = 0;
};

// --- Small SE(3) helpers, shared with the tests -------------------------------

/// Retraction used throughout: rotate by exp(rotvec) about the camera origin,
/// then translate. `xi` is (translation, rotvec). This is the perturbation the
/// residual Jacobian in calibration.cpp is derived against.
Eigen::Isometry3d se3_retract(const Eigen::Matrix<double, 6, 1> & xi);

/// Inverse of se3_retract: (translation, rotvec) of `T`.
Eigen::Matrix<double, 6, 1> se3_params(const Eigen::Isometry3d & T);

/// Convert a correction to the (x, y, z, roll, pitch, yaw) convention the node
/// parameter `extrinsic_correction_xyz_rpy` uses. Distinct from se3_params,
/// which returns a rotation vector — identical only to first order.
Eigen::Matrix<double, 6, 1> to_xyz_rpy(const Eigen::Isometry3d & T);

}  // namespace depth_self_filter
