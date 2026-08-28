// Standalone GPU/EGL smoke test — no ROS required.
//
// Renders an ASYMMETRIC scene (a quad in the upper-left of the image) so that a
// vertical or horizontal flip is detectable — a centered quad would hide it.
// Checks that (a) the geometry lands in the expected image quadrant, and (b) the
// EGL backend agrees with the CPU rasterizer reference. With `--require-egl` it
// exits non-zero unless the EGL backend initialized.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "depth_self_filter/renderer.hpp"

using namespace depth_self_filter;

namespace
{
CameraModel make_cam()
{
  CameraModel c;
  c.width = 128; c.height = 128;
  c.fx = c.fy = 128.f; c.cx = 64.f; c.cy = 64.f;
  c.near = 0.01f; c.far = 10.f;
  return c;
}

// Quad centered up-and-left in the optical frame: X<0 (left), Y<0 (up), so it
// projects to the UPPER-LEFT of the image (small u, small v).
Mesh make_corner_quad()
{
  const float z = 2.0f, cxo = -0.2f, cyo = -0.3f, hs = 0.15f;
  Mesh m;
  m.vertices = {
    {cxo - hs, cyo - hs, z}, {cxo + hs, cyo - hs, z},
    {cxo + hs, cyo + hs, z}, {cxo - hs, cyo + hs, z}};
  m.indices = {0, 1, 2, 0, 2, 3};
  return m;
}

// A quad TILTED about y, so depth varies across it and its plane is neither
// fronto-parallel nor axis-aligned — a fronto-parallel test surface would pass
// even with a badly wrong normal.
Mesh make_tilted_quad()
{
  const float tilt = 0.5f;   // rad about +y
  const float c = std::cos(tilt), s = std::sin(tilt);
  const float z0 = 1.6f, hs = 0.35f;
  Mesh m;
  for (int i = 0; i < 4; ++i) {
    const float lx = (i == 0 || i == 3) ? -hs : hs;
    const float ly = (i < 2) ? -hs : hs;
    // rotate (lx, ly, 0) about y, then push to z0
    m.vertices.push_back({c * lx, ly, -s * lx + z0});
  }
  m.indices = {0, 1, 2, 0, 2, 3};
  return m;
}

// Mean (row, col) of valid pixels.
bool centroid(const std::vector<float> & d, int w, int h, float & row, float & col)
{
  double sr = 0, sc = 0; long n = 0;
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x)
      if (d[static_cast<size_t>(y) * w + x] > 0.f) { sr += y; sc += x; ++n; }
  if (n == 0) return false;
  row = static_cast<float>(sr / n); col = static_cast<float>(sc / n);
  return true;
}
}  // namespace

int main(int argc, char ** argv)
{
  const bool require_egl = argc > 1 && std::strcmp(argv[1], "--require-egl") == 0;

  const CameraModel cam = make_cam();
  Mesh quad = make_corner_quad();
  std::vector<RenderItem> items{{&quad, Eigen::Isometry3f::Identity()}};

  // CPU reference (image convention, matches the depth-image row order).
  auto cpu = make_cpu_renderer();
  std::vector<float> dcpu;
  cpu->render(cam, items, dcpu);
  float cr, cc;
  if (!centroid(dcpu, cam.width, cam.height, cr, cc)) {
    std::fprintf(stderr, "FAIL: CPU produced no geometry\n"); return 5;
  }
  const bool cpu_quadrant_ok = cr < cam.height / 2.f && cc < cam.width / 2.f;
  std::printf("cpu   centroid=(row %.1f, col %.1f) expect top-left (<64,<64) %s\n",
    cr, cc, cpu_quadrant_ok ? "OK" : "WRONG");
  if (!cpu_quadrant_ok) return 6;

  auto egl = make_egl_renderer();
  if (require_egl && !egl) {
    std::fprintf(stderr, "FAIL: EGL backend unavailable\n"); return 2;
  }
  if (!egl) { std::printf("egl   unavailable — CPU-only OK\n"); return 0; }

  std::vector<float> degl;
  if (!egl->render(cam, items, degl)) {
    std::fprintf(stderr, "FAIL: EGL render() returned false\n"); return 3;
  }
  float er, ec;
  if (!centroid(degl, cam.width, cam.height, er, ec)) {
    std::fprintf(stderr, "FAIL: EGL produced no geometry\n"); return 4;
  }
  // A vertical flip would put the centroid at row ~ (128 - cr); horizontal at
  // col ~ (128 - cc). Require the EGL centroid to match the CPU reference.
  const float drow = std::fabs(er - cr), dcol = std::fabs(ec - cc);
  const bool egl_quadrant_ok = er < cam.height / 2.f && ec < cam.width / 2.f;
  std::printf("egl   centroid=(row %.1f, col %.1f)  |dr|=%.1f |dc|=%.1f  quadrant %s\n",
    er, ec, drow, dcol, egl_quadrant_ok ? "OK" : "WRONG(flip?)");

  if (!egl_quadrant_ok || drow > 2.f || dcol > 2.f) {
    std::fprintf(stderr, "FAIL: EGL disagrees with CPU reference (flip/offset)\n");
    return 7;
  }
  // --- Plane output: exact geometry instead of finite-differenced depth --------
  //
  // The estimator in calibration.cpp uses render_planes() to intersect each pixel
  // ray with the true surface plane. On GPU that comes out of a second colour
  // attachment and a dFdx/dFdy face normal, which is a different derivation from
  // the CPU rasterizer's per-triangle plane — so it needs checking against the
  // reference on real hardware, not assuming.
  Mesh tilted = make_tilted_quad();
  std::vector<RenderItem> titems{{&tilted, Eigen::Isometry3f::Identity()}};

  std::vector<float> dtc, ptc, dte, pte;
  const bool cpu_planes = cpu->render_planes(cam, titems, dtc, ptc);
  const bool egl_planes = egl->render_planes(cam, titems, dte, pte);
  std::printf("plane support: cpu=%s egl=%s\n",
    cpu_planes ? "yes" : "no", egl_planes ? "yes" : "no");
  if (!cpu_planes) { std::fprintf(stderr, "FAIL: CPU must supply planes\n"); return 8; }
  if (!egl_planes) {
    std::printf("egl   no plane output — estimator falls back to finite differences\n");
    std::printf("OK\n");
    return 0;
  }

  // Compare only where BOTH backends drew geometry and neither pixel sits on a
  // silhouette (rasterization differs by a pixel at edges, and the GPU's 2x2
  // derivative quads are meaningless there).
  double worst_ang = 0.0, worst_d = 0.0, worst_z = 0.0;
  long compared = 0;
  const int w = cam.width, h = cam.height;
  for (int y = 1; y < h - 1; ++y) {
    for (int x = 1; x < w - 1; ++x) {
      const size_t i = static_cast<size_t>(y) * w + x;
      if (dtc[i] <= 0.f || dte[i] <= 0.f) continue;
      bool interior = true;
      for (int dy = -1; dy <= 1 && interior; ++dy)
        for (int dx = -1; dx <= 1 && interior; ++dx) {
          const size_t j = static_cast<size_t>(y + dy) * w + (x + dx);
          interior = dtc[j] > 0.f && dte[j] > 0.f;
        }
      if (!interior) continue;

      const Eigen::Vector3f nc(ptc[i * 4], ptc[i * 4 + 1], ptc[i * 4 + 2]);
      const Eigen::Vector3f ne(pte[i * 4], pte[i * 4 + 1], pte[i * 4 + 2]);
      if (nc.squaredNorm() < 0.5f || ne.squaredNorm() < 0.5f) continue;
      const double cosang = std::min(1.f, std::max(-1.f, nc.dot(ne)));
      worst_ang = std::max(worst_ang, std::acos(cosang) * 180.0 / M_PI);
      worst_d = std::max<double>(worst_d, std::fabs(ptc[i * 4 + 3] - pte[i * 4 + 3]));

      // What the estimator actually consumes: the ray-plane intersection depth.
      const Eigen::Vector3f v((x + 0.5f - cam.cx) / cam.fx,
                              (y + 0.5f - cam.cy) / cam.fy, 1.f);
      const float zc = ptc[i * 4 + 3] / nc.dot(v);
      const float ze = pte[i * 4 + 3] / ne.dot(v);
      worst_z = std::max<double>(worst_z, std::fabs(zc - ze));
      ++compared;
    }
  }
  std::printf("plane egl-vs-cpu over %ld px: max angle=%.4f deg, max |dd|=%.3f mm, "
              "max |dz|=%.3f mm\n", compared, worst_ang, worst_d * 1e3, worst_z * 1e3);
  if (compared < 1000) {
    std::fprintf(stderr, "FAIL: too few comparable pixels (%ld)\n", compared); return 9;
  }
  // Tolerances: the two derivations differ only by float rounding on a planar
  // surface. Anything larger means the GPU normal is genuinely wrong.
  if (worst_ang > 0.5 || worst_z > 1e-3) {
    std::fprintf(stderr, "FAIL: EGL planes disagree with CPU reference\n"); return 10;
  }

  std::printf("OK\n");
  return 0;
}
