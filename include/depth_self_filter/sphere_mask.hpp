#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include "depth_self_filter/renderer.hpp"

namespace depth_self_filter {

// Mask only measured points inside the cover, not the entire ray behind it.
// Bounding the projected enclosing cube limits work to each sphere's image ROI.
inline void mask_sphere(const CameraModel & cam, const Eigen::Vector3f & center,
                        float radius, std::vector<float> & depth) {
  if (!(cam.fx > 0 && cam.fy > 0) || cam.width <= 0 || cam.height <= 0 ||
      depth.size() != static_cast<size_t>(cam.width) * cam.height ||
      !center.allFinite() || !std::isfinite(radius) || radius <= 0)
    throw std::invalid_argument("invalid sphere mask geometry/camera");
  if (center.z() + radius <= 0) return;
  int x0=0, y0=0, x1=cam.width-1, y1=cam.height-1;
  if (center.z() > radius) {
    float u0=std::numeric_limits<float>::infinity(), v0=u0, u1=-u0, v1=-u0;
    for (int i=0; i<8; ++i) {
      const float x=center.x()+((i&1)?radius:-radius);
      const float y=center.y()+((i&2)?radius:-radius);
      const float z=center.z()+((i&4)?radius:-radius);
      const float u=cam.fx*x/z+cam.cx, v=cam.fy*y/z+cam.cy;
      u0=std::min(u0,u); u1=std::max(u1,u);
      v0=std::min(v0,v); v1=std::max(v1,v);
    }
    if (u1<0 || v1<0 || u0>cam.width-1 || v0>cam.height-1) return;
    x0=static_cast<int>(std::max(0.f,std::floor(u0)));
    y0=static_cast<int>(std::max(0.f,std::floor(v0)));
    x1=static_cast<int>(std::min(float(cam.width-1),std::ceil(u1)));
    y1=static_cast<int>(std::min(float(cam.height-1),std::ceil(v1)));
  }
  for (int y=y0; y<=y1; ++y) for (int x=x0; x<=x1; ++x) {
    auto & z=depth[static_cast<size_t>(y)*cam.width+x];
    if (!std::isfinite(z) || z<=0) continue;
    const Eigen::Vector3f point((x-cam.cx)*z/cam.fx,(y-cam.cy)*z/cam.fy,z);
    if ((point-center).squaredNorm()<=radius*radius)
      z=std::numeric_limits<float>::quiet_NaN();
  }
}
}  // namespace depth_self_filter
