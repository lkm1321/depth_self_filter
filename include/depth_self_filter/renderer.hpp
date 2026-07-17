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
};

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

  /// Human-readable backend name for logging ("egl", "cpu").
  virtual const char * name() const = 0;
};

/// Factory: try to build the EGL backend (if compiled in and a device is
/// available); returns nullptr if unavailable so the caller can fall back.
std::unique_ptr<IRenderer> make_egl_renderer();
/// Always-available software rasterizer.
std::unique_ptr<IRenderer> make_cpu_renderer();

}  // namespace depth_self_filter
