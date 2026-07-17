// Per-mesh bounding-box computation and a conservative AABB-vs-frustum test used
// to pre-select which posed links are worth rasterizing each frame. Culling here
// is backend-agnostic (the node calls it before building RenderItems), so both
// the CPU and EGL paths skip the same off-screen links.
#include "depth_self_filter/renderer.hpp"

#include <array>
#include <limits>

namespace depth_self_filter
{

void compute_bounds(Mesh & mesh)
{
  mesh.bounds_valid = false;
  if (mesh.vertices.empty()) return;
  constexpr float kInf = std::numeric_limits<float>::infinity();
  Eigen::Vector3f lo(kInf, kInf, kInf), hi(-kInf, -kInf, -kInf);
  for (const auto & v : mesh.vertices) {
    lo = lo.cwiseMin(v);
    hi = hi.cwiseMax(v);
  }
  mesh.aabb_min = lo;
  mesh.aabb_max = hi;
  mesh.bounds_valid = true;
}

bool mesh_in_frustum(
  const CameraModel & cam, const Mesh & mesh,
  const Eigen::Isometry3f & T_cam_link)
{
  // No bounds (or a degenerate camera) -> can't safely cull; keep the mesh.
  if (!mesh.bounds_valid || cam.width <= 0 || cam.height <= 0) return true;

  // The eight AABB corners, posed into the camera optical frame.
  const Eigen::Vector3f & lo = mesh.aabb_min;
  const Eigen::Vector3f & hi = mesh.aabb_max;
  std::array<Eigen::Vector3f, 8> corners;
  for (int i = 0; i < 8; ++i) {
    const Eigen::Vector3f c(
      (i & 1) ? hi.x() : lo.x(),
      (i & 2) ? hi.y() : lo.y(),
      (i & 4) ? hi.z() : lo.z());
    corners[i] = T_cam_link * c;
  }

  // Six frustum half-spaces, each written so a point is INSIDE when the value is
  // >= 0. The four side planes pass through the optical centre (a point projects
  // to 0 <= u < width, 0 <= v < height); near/far are z slabs. Optical frame is
  // +x right, +y down, +z forward (REP 103).
  //
  // Conservative cull: if every corner lies outside one plane (all values < 0),
  // the whole box is outside the frustum. This never culls a visible box; it may
  // occasionally keep a box that's actually outside (e.g. spanning a corner),
  // which is harmless — it just rasterizes to nothing.
  const float w = static_cast<float>(cam.width);
  const float h = static_cast<float>(cam.height);
  const auto all_outside = [&](auto plane) {
    for (const auto & p : corners) {
      if (plane(p) >= 0.f) return false;  // this corner is inside -> not cullable
    }
    return true;
  };

  if (all_outside([&](const Eigen::Vector3f & p) { return p.z() - cam.near; })) return false;
  if (all_outside([&](const Eigen::Vector3f & p) { return cam.far - p.z(); })) return false;
  if (all_outside([&](const Eigen::Vector3f & p) { return cam.fx * p.x() + cam.cx * p.z(); })) return false;          // u >= 0
  if (all_outside([&](const Eigen::Vector3f & p) { return (w - cam.cx) * p.z() - cam.fx * p.x(); })) return false;    // u <= width
  if (all_outside([&](const Eigen::Vector3f & p) { return cam.fy * p.y() + cam.cy * p.z(); })) return false;          // v >= 0
  if (all_outside([&](const Eigen::Vector3f & p) { return (h - cam.cy) * p.z() - cam.fy * p.y(); })) return false;    // v <= height

  return true;
}

}  // namespace depth_self_filter
