// Common types and the renderer interface shared by the CPU and EGL backends.
#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Geometry>

namespace depth_self_filter
{

/// A triangle mesh expressed in its link frame (URDF collision/visual origin and
/// scale are baked into the vertices at load time). Positions are metres.
struct Mesh
{
  std::vector<Eigen::Vector3f> vertices;  ///< xyz in link frame
  std::vector<uint32_t> indices;          ///< 3 per triangle, into `vertices`

  /// Axis-aligned bounding box of `vertices` in the link frame, used for cheap
  /// frustum culling before rasterization. Only meaningful when `bounds_valid`;
  /// call compute_bounds() after populating vertices.
  Eigen::Vector3f aabb_min = Eigen::Vector3f::Zero();
  Eigen::Vector3f aabb_max = Eigen::Vector3f::Zero();
  bool bounds_valid = false;
};

/// Recompute `mesh.aabb_min/max` from its vertices and set `bounds_valid`. An
/// empty mesh leaves `bounds_valid` false (nothing to bound).
void compute_bounds(Mesh & mesh);

/// Pinhole model taken from sensor_msgs/CameraInfo. Optical frame convention
/// (REP 103): +x right, +y down, +z forward. Depth is +z in metres.
struct CameraModel
{
  int width = 0;
  int height = 0;
  float fx = 0.f, fy = 0.f, cx = 0.f, cy = 0.f;
  float near = 0.02f;  ///< metres; geometry closer than this is clipped
  float far = 10.0f;   ///< metres; geometry farther than this is ignored
};

/// One link to draw: its mesh plus the pose that maps link-frame points into the
/// camera optical frame (T_cam_link, from TF at the image timestamp).
struct RenderItem
{
  const Mesh * mesh = nullptr;
  Eigen::Isometry3f T_cam_link = Eigen::Isometry3f::Identity();
};

/// Backend-agnostic robot-depth renderer. Implementations produce, for the given
/// camera and posed links, a per-pixel nearest-robot-surface depth image.
class IRenderer
{
public:
  virtual ~IRenderer() = default;

  /// Render the robot depth. `depth_out` is resized to width*height, row-major.
  /// Pixels with no robot geometry are set to 0 (invalid). Returns false on a
  /// backend error (the node then falls back / drops the frame).
  virtual bool render(
    const CameraModel & cam,
    const std::vector<RenderItem> & items,
    std::vector<float> & depth_out) = 0;

  /// Optional: render depth AND the plane of the winning surface per pixel, in
  /// the camera optical frame, packed as 4 floats (nx, ny, nz, d) with `n` unit,
  /// facing the camera, and `n . X = d` for points X on that surface. Pixels with
  /// no geometry get all zeros.
  ///
  /// This exists so a consumer can use the analytic geometry instead of
  /// finite-differencing the depth image to recover normals the rasterizer
  /// already had exactly — and so it can intersect the ray with the true plane
  /// rather than trusting the interpolated z (which this rasterizer computes
  /// linearly in screen space, an approximation for a plane).
  ///
  /// Returns false if the backend cannot supply planes; callers MUST fall back.
  /// The EGL backend currently does not (it would need a second colour
  /// attachment), so it keeps this default.
  virtual bool render_planes(
    const CameraModel & cam,
    const std::vector<RenderItem> & items,
    std::vector<float> & depth_out,
    std::vector<float> & plane_out)
  {
    (void)cam; (void)items; (void)depth_out; (void)plane_out;
    return false;
  }

  /// Human-readable backend name for logging ("egl", "cpu").
  virtual const char * name() const = 0;
};

/// Conservative visibility test: could `mesh` (via its link-frame AABB, posed by
/// `T_cam_link` into the camera optical frame) project anywhere inside `cam`'s
/// view frustum? Returns true if it might be visible, false only when the whole
/// box is provably outside (behind near, past far, or fully off-frame). Meshes
/// without valid bounds are treated as potentially visible (never culled). Used
/// to pre-select which links to rasterize each frame.
bool mesh_in_frustum(
  const CameraModel & cam, const Mesh & mesh,
  const Eigen::Isometry3f & T_cam_link);

/// Factory: try to build the EGL backend (if compiled in and a device is
/// available); returns nullptr if unavailable so the caller can fall back.
std::unique_ptr<IRenderer> make_egl_renderer();
/// Always-available software rasterizer.
std::unique_ptr<IRenderer> make_cpu_renderer();

}  // namespace depth_self_filter
