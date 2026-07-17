// Ablation: how much render time does AABB frustum culling actually save?
//
// Loads a real robot (URDF + collision meshes) through the production loader,
// reconstructs each link's pose in the camera optical frame via zero-config
// forward kinematics (camera = a chosen optical link), then measures per-frame
// render time on each available backend with culling OFF (all links) vs ON (only
// links whose AABB is inside the frustum). Also reports the cull-test overhead so
// the net win is honest. A yaw sweep shows how the win scales with how much of
// the robot is off-screen.
//
// Usage: frustum_bench <urdf> <mesh_stage_dir> [camera_link]
//   urdf            expanded URDF (mesh filenames remapped to mesh_stage by base)
//   mesh_stage_dir  dir holding the collider .dae files (by basename)
//   camera_link     optical-frame link to sit the camera at (default tof_camera_optical)
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include <urdf/model.h>

#include "depth_self_filter/renderer.hpp"
#include "depth_self_filter/urdf_meshes.hpp"

using namespace depth_self_filter;
using Clock = std::chrono::steady_clock;

namespace
{
// Read a whole file into a string.
std::string slurp(const std::string & path)
{
  std::ifstream f(path);
  return std::string(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
}

// Rewrite every mesh filename="..." to file://<stage>/<basename>, so the meshes
// resolve against a flat local staging dir (mirrors the bag test's preprocess).
std::string remap_meshes(std::string xml, const std::string & stage)
{
  std::string out;
  size_t i = 0;
  const std::string key = "filename=\"";
  while (true) {
    size_t p = xml.find(key, i);
    if (p == std::string::npos) { out += xml.substr(i); break; }
    size_t s = p + key.size();
    size_t e = xml.find('"', s);
    std::string val = xml.substr(s, e - s);
    std::string base = val.substr(val.find_last_of('/') + 1);
    // The reachy torso references a *_no_cone collider we alias to the plain one.
    if (base == "torso_collider_no_cone.dae") base = "torso_collider.dae";
    out += xml.substr(i, s - i);
    out += "file://" + stage + "/" + base;
    i = e;
  }
  return out;
}

Eigen::Isometry3f origin_iso(const urdf::Pose & p)
{
  Eigen::Isometry3f t = Eigen::Isometry3f::Identity();
  t.translation() = Eigen::Vector3f(p.position.x, p.position.y, p.position.z);
  t.linear() = Eigen::Quaternionf(
    p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z).toRotationMatrix();
  return t;
}

// Forward kinematics at joint configuration `q` (radians / metres; joints absent
// from the map are at 0). Each joint contributes its fixed parent->child origin
// times its axis motion. Enough to place links in realistic poses vs the camera.
std::map<std::string, Eigen::Isometry3f> fk(
  const urdf::Model & model, const std::map<std::string, double> & q)
{
  std::map<std::string, Eigen::Isometry3f> pose;
  const auto root = model.getRoot();
  if (!root) return pose;
  pose[root->name] = Eigen::Isometry3f::Identity();
  std::vector<urdf::LinkConstSharedPtr> stack{root};
  while (!stack.empty()) {
    auto link = stack.back(); stack.pop_back();
    const Eigen::Isometry3f Tp = pose[link->name];
    for (const auto & j : link->child_joints) {
      Eigen::Isometry3f motion = Eigen::Isometry3f::Identity();
      auto it = q.find(j->name);
      const double v = it == q.end() ? 0.0 : it->second;
      if (v != 0.0 && (j->type == urdf::Joint::REVOLUTE ||
                       j->type == urdf::Joint::CONTINUOUS)) {
        const Eigen::Vector3f ax(j->axis.x, j->axis.y, j->axis.z);
        motion.linear() = Eigen::AngleAxisf(static_cast<float>(v), ax.normalized()).toRotationMatrix();
      } else if (v != 0.0 && j->type == urdf::Joint::PRISMATIC) {
        const Eigen::Vector3f ax(j->axis.x, j->axis.y, j->axis.z);
        motion.translation() = static_cast<float>(v) * ax.normalized();
      }
      const Eigen::Isometry3f Tc =
        Tp * origin_iso(j->parent_to_joint_origin_transform) * motion;
      pose[j->child_link_name] = Tc;
      if (auto c = model.getLink(j->child_link_name)) stack.push_back(c);
    }
  }
  return pose;
}

struct PosedLink
{
  const Mesh * mesh;
  Eigen::Isometry3f T_cam_link;
  size_t tris;
};

// Median of a vector of timings (ms).
double median(std::vector<double> v)
{
  std::sort(v.begin(), v.end());
  return v.empty() ? 0.0 : v[v.size() / 2];
}

// Time `iters` renders of `items`, return median ms/frame.
double time_render(IRenderer & r, const CameraModel & cam,
  const std::vector<RenderItem> & items, int iters)
{
  std::vector<float> depth;
  r.render(cam, items, depth);  // warm up (FBO alloc, first upload)
  std::vector<double> t;
  t.reserve(iters);
  for (int k = 0; k < iters; ++k) {
    const auto a = Clock::now();
    r.render(cam, items, depth);
    const auto b = Clock::now();
    t.push_back(std::chrono::duration<double, std::milli>(b - a).count());
  }
  return median(std::move(t));
}

struct Scene { const char * name; std::map<std::string, double> q; };
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::fprintf(stderr, "usage: %s <urdf> <mesh_stage_dir> [camera_link]\n", argv[0]);
    return 2;
  }
  const std::string urdf_path = argv[1];
  const std::string stage = argv[2];
  const std::string cam_link = argc > 3 ? argv[3] : "tof_camera_optical";

  const std::string xml = remap_meshes(slurp(urdf_path), stage);

  // Real production load path: merged per-link collision meshes with AABBs.
  const auto links = load_link_meshes(xml, /*use_collision=*/true, {}, {});
  urdf::Model model;
  if (!model.initString(xml)) { std::fprintf(stderr, "urdf parse failed\n"); return 1; }

  CameraModel cam;
  cam.width = 640; cam.height = 480;
  cam.fx = cam.fy = 457.f; cam.cx = 320.f; cam.cy = 240.f;
  cam.near = 0.02f; cam.far = 10.f;

  size_t total_tris = 0;
  for (const auto & lg : links) total_tris += lg.mesh.indices.size() / 3;
  std::printf("Robot: %zu links, %zu triangles. Camera '%s', %dx%d fx=%.0f near=%.2f far=%.1f\n",
    links.size(), total_tris, cam_link.c_str(), cam.width, cam.height, cam.fx, cam.near, cam.far);
  std::printf("A body-mounted depth camera sees only a fraction of the robot; the rest is\n"
              "behind/beside the sensor. 'off_ms' renders every link; 'on_ms' renders only the\n"
              "in-frustum subset (+ the cull test). Times are median ms/frame.\n\n");

  // Available backends.
  std::vector<std::pair<std::string, std::unique_ptr<IRenderer>>> backends;
  if (auto egl = make_egl_renderer()) backends.emplace_back("egl", std::move(egl));
  backends.emplace_back("cpu", make_cpu_renderer());

  // Representative joint poses: arms at the sides (home) vs raised into the
  // workspace in front of the head camera (manipulation / teleop, like the bag).
  const std::vector<Scene> scenes = {
    {"home  (arms down)", {}},
    {"reach (arms fwd) ", {
      {"r_shoulder_pitch", -1.4}, {"r_elbow_pitch", -1.2},
      {"l_shoulder_pitch", -1.4}, {"l_elbow_pitch", -1.2}}},
  };

  for (const auto & sc : scenes) {
    const auto pose = fk(model, sc.q);
    if (!pose.count(cam_link)) {
      std::fprintf(stderr, "camera link '%s' not in URDF\n", cam_link.c_str()); return 1;
    }
    const Eigen::Isometry3f T_cam_base = pose.at(cam_link).inverse();

    std::vector<PosedLink> posed;
    for (const auto & lg : links) {
      if (!pose.count(lg.link_name)) continue;
      posed.push_back({&lg.mesh, T_cam_base * pose.at(lg.link_name),
                       lg.mesh.indices.size() / 3});
    }

    // Partition: visible vs culled, and how many are culled purely for being
    // behind the sensor (AABB entirely at z <= near) — the body-mounted effect.
    std::vector<RenderItem> all, vis;
    size_t tris_all = 0, tris_vis = 0, behind = 0;
    for (const auto & p : posed) {
      all.push_back({p.mesh, p.T_cam_link});
      tris_all += p.tris;
      if (mesh_in_frustum(cam, *p.mesh, p.T_cam_link)) {
        vis.push_back({p.mesh, p.T_cam_link}); tris_vis += p.tris;
      } else {
        // Is the whole box behind the near plane in the camera frame?
        bool all_behind = true;
        for (int c = 0; c < 8 && all_behind; ++c) {
          const Eigen::Vector3f corner(
            (c & 1) ? p.mesh->aabb_max.x() : p.mesh->aabb_min.x(),
            (c & 2) ? p.mesh->aabb_max.y() : p.mesh->aabb_min.y(),
            (c & 4) ? p.mesh->aabb_max.z() : p.mesh->aabb_min.z());
          if ((p.T_cam_link * corner).z() > cam.near) all_behind = false;
        }
        if (all_behind) ++behind;
      }
    }

    // Cull-test cost: run the whole per-frame pre-select many times, amortize.
    const int cull_reps = 5000;
    const auto ca = Clock::now();
    volatile int keep = 0;
    for (int rep = 0; rep < cull_reps; ++rep)
      for (const auto & p : posed)
        keep += mesh_in_frustum(cam, *p.mesh, p.T_cam_link) ? 1 : 0;
    const auto cb = Clock::now();
    const double cull_ms =
      std::chrono::duration<double, std::milli>(cb - ca).count() / cull_reps;
    (void)keep;

    if (std::getenv("DSF_BENCH_DEBUG")) {
      std::printf("  [debug %s] per-link camera-frame centroid:\n", sc.name);
      size_t li = 0;
      for (const auto & lg : links) {
        if (!pose.count(lg.link_name)) continue;
        const auto & p = posed[li++];
        Eigen::Vector3f c = Eigen::Vector3f::Zero();
        for (const auto & v : lg.mesh.vertices) c += v;
        c /= static_cast<float>(std::max<size_t>(1, lg.mesh.vertices.size()));
        const Eigen::Vector3f cc = p.T_cam_link * c;
        std::printf("    %-24s z=%+6.2f x=%+6.2f y=%+6.2f  vis=%d\n",
          lg.link_name.c_str(), cc.z(), cc.x(), cc.y(),
          mesh_in_frustum(cam, *p.mesh, p.T_cam_link));
      }
    }
    std::printf("Pose %s: %zu/%zu links in frustum (%zu behind sensor), "
      "%zu/%zu triangles rendered. cull test %.4f ms/frame\n",
      sc.name, vis.size(), posed.size(), behind, tris_vis, tris_all, cull_ms);
    std::printf("  %-5s | %9s | %9s | %8s | %7s\n",
      "bknd", "off_ms", "on_ms", "saved", "speedup");
    std::printf("  ------|-----------|-----------|----------|--------\n");
    for (auto & [bname, rptr] : backends) {
      const int iters = (bname == "cpu") ? 100 : 500;
      const double off_ms = time_render(*rptr, cam, all, iters);
      const double on_ms = time_render(*rptr, cam, vis, iters) + cull_ms;
      std::printf("  %-5s | %9.4f | %9.4f | %7.4f%% | %6.2fx\n",
        bname.c_str(), off_ms, on_ms, 100.0 * (off_ms - on_ms) / off_ms, off_ms / on_ms);
    }
    std::printf("\n");
  }
  return 0;
}
