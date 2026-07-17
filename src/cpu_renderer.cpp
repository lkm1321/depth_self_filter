// Software triangle rasterizer producing a robot depth image. Always available;
// used for CI/no-GPU and as the automatic fallback when EGL init fails.
//
// Convention: points arrive in the camera optical frame (+x right, +y down,
// +z forward). We project with the pinhole model and keep, per pixel, the
// smallest positive z (nearest surface). No shading — depth only.
#include "depth_self_filter/renderer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace depth_self_filter
{
namespace
{

class CpuRenderer : public IRenderer
{
public:
  bool render(
    const CameraModel & cam,
    const std::vector<RenderItem> & items,
    std::vector<float> & depth_out) override
  {
    const int w = cam.width, h = cam.height;
    if (w <= 0 || h <= 0) return false;
    depth_out.assign(static_cast<size_t>(w) * h,
      std::numeric_limits<float>::infinity());

    for (const auto & item : items) {
      if (!item.mesh) continue;
      const Mesh & m = *item.mesh;
      const Eigen::Isometry3f & T = item.T_cam_link;
      for (size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const Eigen::Vector3f p0 = T * m.vertices[m.indices[t + 0]];
        const Eigen::Vector3f p1 = T * m.vertices[m.indices[t + 1]];
        const Eigen::Vector3f p2 = T * m.vertices[m.indices[t + 2]];
        raster_triangle(cam, p0, p1, p2, w, h, depth_out);
      }
    }

    // Convert "infinity" sentinel to 0 (invalid) for the caller's convention.
    for (float & d : depth_out) {
      if (!std::isfinite(d) || d > cam.far) d = 0.f;
    }
    return true;
  }

  const char * name() const override { return "cpu"; }

private:
  // Project a camera-frame point to pixel coords; returns false if behind near.
  static bool project(
    const CameraModel & cam, const Eigen::Vector3f & p,
    float & u, float & v, float & z)
  {
    z = p.z();
    if (z <= cam.near) return false;
    u = cam.fx * (p.x() / z) + cam.cx;
    v = cam.fy * (p.y() / z) + cam.cy;
    return true;
  }

  static void raster_triangle(
    const CameraModel & cam,
    const Eigen::Vector3f & a, const Eigen::Vector3f & b,
    const Eigen::Vector3f & c, int w, int h, std::vector<float> & depth)
  {
    // v1: skip triangles crossing the near plane rather than clipping them.
    // A small margin of missed silhouette pixels is absorbed by the node's
    // depth margin + mask dilation. TODO(v2): near-plane clip for close links.
    float ua, va, za, ub, vb, zb, uc, vc, zc;
    if (!project(cam, a, ua, va, za) ||
        !project(cam, b, ub, vb, zb) ||
        !project(cam, c, uc, vc, zc)) {
      return;
    }

    const float minxf = std::floor(std::min({ua, ub, uc}));
    const float maxxf = std::ceil(std::max({ua, ub, uc}));
    const float minyf = std::floor(std::min({va, vb, vc}));
    const float maxyf = std::ceil(std::max({va, vb, vc}));
    const int minx = std::max(0, static_cast<int>(minxf));
    const int maxx = std::min(w - 1, static_cast<int>(maxxf));
    const int miny = std::max(0, static_cast<int>(minyf));
    const int maxy = std::min(h - 1, static_cast<int>(maxyf));
    if (minx > maxx || miny > maxy) return;

    // Edge-function area; reject degenerate/back-facing-collapsed triangles.
    const float area = edge(ua, va, ub, vb, uc, vc);
    if (std::fabs(area) < 1e-6f) return;
    const float inv_area = 1.0f / area;

    for (int y = miny; y <= maxy; ++y) {
      const float py = y + 0.5f;
      for (int x = minx; x <= maxx; ++x) {
        const float px = x + 0.5f;
        float w0 = edge(ub, vb, uc, vc, px, py) * inv_area;
        float w1 = edge(uc, vc, ua, va, px, py) * inv_area;
        float w2 = edge(ua, va, ub, vb, px, py) * inv_area;
        // Accept regardless of winding (either all >=0 or all <=0).
        const bool inside = (w0 >= 0 && w1 >= 0 && w2 >= 0) ||
                            (w0 <= 0 && w1 <= 0 && w2 <= 0);
        if (!inside) continue;
        // Interpolate depth. Linear-in-screen interpolation of z is a small
        // approximation vs perspective-correct 1/z; adequate at pixel scale.
        const float z = w0 * za + w1 * zb + w2 * zc;
        if (z <= cam.near) continue;
        float & dst = depth[static_cast<size_t>(y) * w + x];
        if (z < dst) dst = z;
      }
    }
  }

  static float edge(float ax, float ay, float bx, float by, float cx, float cy)
  {
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
  }
};

}  // namespace

std::unique_ptr<IRenderer> make_cpu_renderer()
{
  return std::make_unique<CpuRenderer>();
}

}  // namespace depth_self_filter
