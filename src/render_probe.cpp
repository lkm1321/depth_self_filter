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
  std::printf("OK\n");
  return 0;
}
