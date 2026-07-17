#include "depth_self_filter/urdf_meshes.hpp"

#include <algorithm>
#include <cmath>

#include <assimp/Importer.hpp>
#include <assimp/config.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <resource_retriever/retriever.hpp>
#include <urdf/model.h>

namespace depth_self_filter
{
namespace
{

// URDF Pose (origin) -> Eigen::Isometry3f.
Eigen::Isometry3f to_iso(const urdf::Pose & p)
{
  Eigen::Isometry3f t = Eigen::Isometry3f::Identity();
  t.translation() = Eigen::Vector3f(p.position.x, p.position.y, p.position.z);
  t.linear() = Eigen::Quaternionf(
    p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z).toRotationMatrix();
  return t;
}

// Append `src` transformed by `xf` (with per-axis scale) into `dst`.
void append(Mesh & dst, const std::vector<Eigen::Vector3f> & v,
  const std::vector<uint32_t> & idx, const Eigen::Isometry3f & xf,
  const Eigen::Vector3f & scale)
{
  const uint32_t base = static_cast<uint32_t>(dst.vertices.size());
  for (const auto & p : v) {
    dst.vertices.push_back(xf * p.cwiseProduct(scale));
  }
  for (uint32_t i : idx) dst.indices.push_back(base + i);
}

// ---- Primitive tessellation (link-local, unit-ish; scaled by append) --------
void make_box(const urdf::Box & b,
  std::vector<Eigen::Vector3f> & v, std::vector<uint32_t> & idx)
{
  const float x = b.dim.x * 0.5f, y = b.dim.y * 0.5f, z = b.dim.z * 0.5f;
  v = {{-x,-y,-z},{x,-y,-z},{x,y,-z},{-x,y,-z},
       {-x,-y, z},{x,-y, z},{x,y, z},{-x,y, z}};
  idx = {0,1,2, 0,2,3, 4,6,5, 4,7,6, 0,4,5, 0,5,1,
         1,5,6, 1,6,2, 2,6,7, 2,7,3, 3,7,4, 3,4,0};
}

void make_sphere(const urdf::Sphere & s,
  std::vector<Eigen::Vector3f> & v, std::vector<uint32_t> & idx)
{
  const int stacks = 12, slices = 16;
  const float r = s.radius;
  for (int i = 0; i <= stacks; ++i) {
    const float phi = M_PI * i / stacks;
    for (int j = 0; j <= slices; ++j) {
      const float th = 2.f * M_PI * j / slices;
      v.emplace_back(r * std::sin(phi) * std::cos(th),
                     r * std::sin(phi) * std::sin(th), r * std::cos(phi));
    }
  }
  const int row = slices + 1;
  for (int i = 0; i < stacks; ++i) {
    for (int j = 0; j < slices; ++j) {
      const uint32_t a = i * row + j, b = a + row;
      idx.insert(idx.end(), {a, b, a + 1, a + 1, b, b + 1});
    }
  }
}

void make_cylinder(const urdf::Cylinder & c,
  std::vector<Eigen::Vector3f> & v, std::vector<uint32_t> & idx)
{
  const int slices = 24;
  const float r = c.radius, hz = c.length * 0.5f;
  for (int j = 0; j <= slices; ++j) {
    const float th = 2.f * M_PI * j / slices;
    v.emplace_back(r * std::cos(th), r * std::sin(th), -hz);
    v.emplace_back(r * std::cos(th), r * std::sin(th),  hz);
  }
  for (int j = 0; j < slices; ++j) {
    const uint32_t a = 2 * j;
    idx.insert(idx.end(), {a, a + 1, a + 2, a + 1, a + 3, a + 2});
  }
  // caps
  const uint32_t cb = static_cast<uint32_t>(v.size());
  v.emplace_back(0, 0, -hz); v.emplace_back(0, 0, hz);
  for (int j = 0; j < slices; ++j) {
    idx.insert(idx.end(), {cb, static_cast<uint32_t>(2 * j), static_cast<uint32_t>(2 * j + 2)});
    idx.insert(idx.end(), {cb + 1, static_cast<uint32_t>(2 * j + 3), static_cast<uint32_t>(2 * j + 1)});
  }
}

// Load a mesh resource (package:// / file://) via resource_retriever + assimp.
bool load_mesh_resource(const std::string & uri,
  std::vector<Eigen::Vector3f> & v, std::vector<uint32_t> & idx)
{
  resource_retriever::Retriever retriever;
  resource_retriever::MemoryResource res;
  try {
    res = retriever.get(uri);
  } catch (const std::exception &) {
    return false;
  }
  Assimp::Importer importer;
  // COLLADA meshes carry per-node <matrix> transforms (placement/rotation of the
  // geometry within the file) and an <up_axis>. We must bake the node transforms
  // into the vertices (PreTransformVertices) or the geometry is displaced/rotated
  // — the wrong shape. ROS/URDF meshes are Z-up in the link frame, so we tell
  // assimp NOT to rotate the scene into its Y-up convention.
  importer.SetPropertyInteger(AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION, 1);
  // Pass the file extension as a hint so assimp selects the right importer.
  const auto dot = uri.find_last_of('.');
  const std::string hint = dot == std::string::npos ? "" : uri.substr(dot + 1);
  const aiScene * scene = importer.ReadFileFromMemory(
    res.data.get(), res.size,
    aiProcess_Triangulate | aiProcess_PreTransformVertices, hint.c_str());
  if (!scene) return false;
  for (unsigned mi = 0; mi < scene->mNumMeshes; ++mi) {
    const aiMesh * am = scene->mMeshes[mi];
    const uint32_t base = static_cast<uint32_t>(v.size());
    for (unsigned i = 0; i < am->mNumVertices; ++i) {
      v.emplace_back(am->mVertices[i].x, am->mVertices[i].y, am->mVertices[i].z);
    }
    for (unsigned f = 0; f < am->mNumFaces; ++f) {
      const aiFace & face = am->mFaces[f];
      if (face.mNumIndices != 3) continue;
      idx.push_back(base + face.mIndices[0]);
      idx.push_back(base + face.mIndices[1]);
      idx.push_back(base + face.mIndices[2]);
    }
  }
  return !v.empty();
}

// Turn one urdf::Geometry into local verts/idx + its scale vector.
bool geometry_to_mesh(const urdf::Geometry & g,
  std::vector<Eigen::Vector3f> & v, std::vector<uint32_t> & idx,
  Eigen::Vector3f & scale)
{
  scale = Eigen::Vector3f::Ones();
  switch (g.type) {
    case urdf::Geometry::BOX:
      make_box(static_cast<const urdf::Box &>(g), v, idx); return true;
    case urdf::Geometry::SPHERE:
      make_sphere(static_cast<const urdf::Sphere &>(g), v, idx); return true;
    case urdf::Geometry::CYLINDER:
      make_cylinder(static_cast<const urdf::Cylinder &>(g), v, idx); return true;
    case urdf::Geometry::MESH: {
      const auto & m = static_cast<const urdf::Mesh &>(g);
      scale = Eigen::Vector3f(m.scale.x, m.scale.y, m.scale.z);
      return load_mesh_resource(m.filename, v, idx);
    }
    default:
      return false;
  }
}

bool allowed(const std::string & name,
  const std::vector<std::string> & include,
  const std::vector<std::string> & exclude)
{
  if (std::find(exclude.begin(), exclude.end(), name) != exclude.end()) return false;
  if (include.empty()) return true;
  return std::find(include.begin(), include.end(), name) != include.end();
}

}  // namespace

std::vector<LinkGeometry> load_link_meshes(
  const std::string & urdf_xml, bool use_collision,
  const std::vector<std::string> & include,
  const std::vector<std::string> & exclude)
{
  std::vector<LinkGeometry> out;
  urdf::Model model;
  if (!model.initString(urdf_xml)) return out;

  for (const auto & kv : model.links_) {
    const auto & link = kv.second;
    if (!link || !allowed(link->name, include, exclude)) continue;

    // Collect the geometry list (a link may have several).
    std::vector<urdf::CollisionSharedPtr> cols;
    std::vector<urdf::VisualSharedPtr> vis;
    if (use_collision) {
      if (link->collision) cols.push_back(link->collision);
      for (const auto & c : link->collision_array) if (c) cols.push_back(c);
    } else {
      if (link->visual) vis.push_back(link->visual);
      for (const auto & c : link->visual_array) if (c) vis.push_back(c);
    }

    LinkGeometry lg;
    lg.link_name = link->name;
    auto add_one = [&](const urdf::GeometrySharedPtr & geom, const urdf::Pose & origin) {
      if (!geom) return;
      std::vector<Eigen::Vector3f> v;
      std::vector<uint32_t> idx;
      Eigen::Vector3f scale;
      if (geometry_to_mesh(*geom, v, idx, scale)) {
        append(lg.mesh, v, idx, to_iso(origin), scale);
      }
    };
    for (const auto & c : cols) add_one(c->geometry, c->origin);
    for (const auto & c : vis) add_one(c->geometry, c->origin);

    if (!lg.mesh.indices.empty()) out.push_back(std::move(lg));
  }
  return out;
}

}  // namespace depth_self_filter
