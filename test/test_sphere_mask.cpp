#include <gtest/gtest.h>
#include "depth_self_filter/sphere_mask.hpp"
using namespace depth_self_filter;

TEST(SphereMask, KeepsForegroundAndBackground) {
  CameraModel c; c.width=5; c.height=3; c.fx=10; c.fy=10; c.cx=2; c.cy=1;
  std::vector<float> d(15,2.f);
  d[5]=1.f; d[6]=1.8f; d[7]=2.f; d[8]=2.2f; d[9]=3.f;
  mask_sphere(c,{0,0,2},0.3,d);
  EXPECT_FLOAT_EQ(d[5],1.f);
  EXPECT_TRUE(std::isnan(d[6])); EXPECT_TRUE(std::isnan(d[7]));
  EXPECT_TRUE(std::isnan(d[8])); EXPECT_FLOAT_EQ(d[9],3.f);
}
TEST(SphereMask, OffscreenAndBehindCamera) {
  CameraModel c; c.width=5; c.height=3; c.fx=10; c.fy=10; c.cx=2; c.cy=1;
  std::vector<float> d(15,2.f), expected=d;
  mask_sphere(c,{100,0,2},0.3,d); mask_sphere(c,{0,0,-2},0.3,d);
  EXPECT_EQ(d,expected);
}
TEST(SphereMask, CameraInsideSphere) {
  CameraModel c; c.width=3; c.height=3; c.fx=10; c.fy=10; c.cx=1; c.cy=1;
  std::vector<float> d(9,0.1f);
  mask_sphere(c,{0,0,0},0.3,d);
  for (float z:d) EXPECT_TRUE(std::isnan(z));
}
