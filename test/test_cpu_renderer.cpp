// Sanity tests for the CPU rasterizer: a known triangle/quad should render at
// the expected depth, and pixels outside it should stay invalid.
#include <gtest/gtest.h>

#include "depth_self_filter/renderer.hpp"

using namespace depth_self_filter;

namespace
{
CameraModel make_cam(int w, int h)
{
  CameraModel c;
  c.width = w; c.height = h;
  c.fx = c.fy = 100.f;
  c.cx = w / 2.f; c.cy = h / 2.f;
  c.near = 0.01f; c.far = 10.f;
  return c;
}

// A quad facing the camera at z = Z, spanning a chunk around the principal point.
Mesh make_quad(float half, float z)
{
  Mesh m;
  m.vertices = {{-half, -half, z}, {half, -half, z}, {half, half, z}, {-half, half, z}};
  m.indices = {0, 1, 2, 0, 2, 3};
  return m;
}
}  // namespace

TEST(CpuRenderer, RendersQuadAtExpectedDepth)
{
  auto r = make_cpu_renderer();
  const auto cam = make_cam(64, 64);
  const float z = 2.0f;
  Mesh quad = make_quad(0.3f, z);
  std::vector<RenderItem> items{{&quad, Eigen::Isometry3f::Identity()}};

  std::vector<float> depth;
  ASSERT_TRUE(r->render(cam, items, depth));
  ASSERT_EQ(depth.size(), 64u * 64u);

  // Center pixel is inside the quad -> depth ~ z.
  const float center = depth[32 * 64 + 32];
  EXPECT_NEAR(center, z, 1e-3f);

  // A corner is outside the projected quad -> invalid (0).
  EXPECT_FLOAT_EQ(depth[0], 0.f);
}

TEST(CpuRenderer, EmptySceneIsAllInvalid)
{
  auto r = make_cpu_renderer();
  const auto cam = make_cam(16, 16);
  std::vector<float> depth;
  ASSERT_TRUE(r->render(cam, {}, depth));
  for (float d : depth) EXPECT_FLOAT_EQ(d, 0.f);
}

// Pins the image-row convention: an up-and-left quad in the optical frame must
// land in the TOP-LEFT of the output (small row, small col). A vertical flip
// would put it in the bottom half — this is the regression the render_probe
// EGL-vs-CPU check guards on the GPU path.
TEST(CpuRenderer, AsymmetricQuadLandsTopLeft)
{
  auto r = make_cpu_renderer();
  const auto cam = make_cam(128, 128);  // fx=fy=100, cx=cy=64
  const float z = 2.0f;
  Mesh m;
  const float cxo = -0.2f, cyo = -0.3f, hs = 0.12f;  // X<0 left, Y<0 up
  m.vertices = {{cxo - hs, cyo - hs, z}, {cxo + hs, cyo - hs, z},
                {cxo + hs, cyo + hs, z}, {cxo - hs, cyo + hs, z}};
  m.indices = {0, 1, 2, 0, 2, 3};
  std::vector<RenderItem> items{{&m, Eigen::Isometry3f::Identity()}};

  std::vector<float> depth;
  ASSERT_TRUE(r->render(cam, items, depth));
  double sr = 0, sc = 0; long n = 0;
  for (int y = 0; y < 128; ++y)
    for (int x = 0; x < 128; ++x)
      if (depth[y * 128 + x] > 0.f) { sr += y; sc += x; ++n; }
  ASSERT_GT(n, 0);
  EXPECT_LT(sr / n, 64.0);  // top half
  EXPECT_LT(sc / n, 64.0);  // left half
}

TEST(CpuRenderer, NearerSurfaceWins)
{
  auto r = make_cpu_renderer();
  const auto cam = make_cam(32, 32);
  Mesh far = make_quad(0.4f, 3.0f);
  Mesh near = make_quad(0.4f, 1.5f);
  std::vector<RenderItem> items{
    {&far, Eigen::Isometry3f::Identity()},
    {&near, Eigen::Isometry3f::Identity()}};
  std::vector<float> depth;
  ASSERT_TRUE(r->render(cam, items, depth));
  EXPECT_NEAR(depth[16 * 32 + 16], 1.5f, 1e-3f);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
