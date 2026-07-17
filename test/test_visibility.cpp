// Tests for the AABB frustum culling used to pre-select visible links.
// The contract is conservative: a mesh that could be visible must never be
// culled (no false "outside"); a mesh provably outside should be culled.
#include <gtest/gtest.h>

#include "depth_self_filter/renderer.hpp"

using namespace depth_self_filter;

namespace
{
CameraModel make_cam()
{
  CameraModel c;
  c.width = 640; c.height = 480;
  c.fx = c.fy = 500.f;
  c.cx = 320.f; c.cy = 240.f;
  c.near = 0.05f; c.far = 10.f;
  return c;
}

// A small unit-ish cube centred at the origin of the link frame.
Mesh make_cube(float half)
{
  Mesh m;
  m.vertices = {{-half,-half,-half},{half,-half,-half},{half,half,-half},{-half,half,-half},
                {-half,-half, half},{half,-half, half},{half,half, half},{-half,half, half}};
  m.indices = {0,1,2, 0,2,3, 4,6,5, 4,7,6};  // (partial; irrelevant to bounds)
  compute_bounds(m);
  return m;
}

Eigen::Isometry3f at(float x, float y, float z)
{
  Eigen::Isometry3f t = Eigen::Isometry3f::Identity();
  t.translation() = Eigen::Vector3f(x, y, z);
  return t;
}
}  // namespace

TEST(Visibility, ComputeBoundsCoversVertices)
{
  Mesh m = make_cube(0.25f);
  ASSERT_TRUE(m.bounds_valid);
  EXPECT_FLOAT_EQ(m.aabb_min.x(), -0.25f);
  EXPECT_FLOAT_EQ(m.aabb_max.z(),  0.25f);
}

TEST(Visibility, EmptyMeshHasNoBounds)
{
  Mesh m;
  compute_bounds(m);
  EXPECT_FALSE(m.bounds_valid);
  // No bounds -> never culled (conservative).
  EXPECT_TRUE(mesh_in_frustum(make_cam(), m, Eigen::Isometry3f::Identity()));
}

TEST(Visibility, InFrontAndCentredIsVisible)
{
  const Mesh m = make_cube(0.1f);
  EXPECT_TRUE(mesh_in_frustum(make_cam(), m, at(0.f, 0.f, 1.0f)));
}

TEST(Visibility, BehindCameraIsCulled)
{
  const Mesh m = make_cube(0.1f);
  // Whole box behind the near plane (negative z).
  EXPECT_FALSE(mesh_in_frustum(make_cam(), m, at(0.f, 0.f, -1.0f)));
}

TEST(Visibility, BeyondFarIsCulled)
{
  const Mesh m = make_cube(0.1f);
  EXPECT_FALSE(mesh_in_frustum(make_cam(), m, at(0.f, 0.f, 50.0f)));
}

TEST(Visibility, FarOffToTheSideIsCulled)
{
  const Mesh m = make_cube(0.1f);
  // At z=1, the right frustum edge is x = (width-cx)/fx = 320/500 = 0.64 m.
  // Place the little box well past it.
  EXPECT_FALSE(mesh_in_frustum(make_cam(), m, at(3.0f, 0.f, 1.0f)));
}

TEST(Visibility, StraddlingTheEdgeIsKept)
{
  const Mesh m = make_cube(0.5f);
  // Centre just outside the right edge but the 0.5 m half-extent reaches back in.
  EXPECT_TRUE(mesh_in_frustum(make_cam(), m, at(0.7f, 0.f, 1.0f)));
}

TEST(Visibility, DisabledCameraKeepsEverything)
{
  CameraModel bad = make_cam();
  bad.width = 0;  // degenerate -> cannot cull safely
  const Mesh m = make_cube(0.1f);
  EXPECT_TRUE(mesh_in_frustum(bad, m, at(0.f, 0.f, -100.0f)));
}
