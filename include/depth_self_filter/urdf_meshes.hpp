// Load per-link collision/visual geometry from a URDF into link-frame meshes.
#pragma once

#include <map>
#include <string>
#include <vector>

#include "depth_self_filter/renderer.hpp"

namespace depth_self_filter
{

struct LinkGeometry
{
  std::string link_name;
  Mesh mesh;  ///< all of the link's geometries merged, in link frame
};

/// Parse `urdf_xml` and build one merged mesh per link (origin + scale baked in).
/// If `use_collision` is true, collision geometry is used (recommended), else
/// visual. `include`/`exclude` optionally restrict which links are loaded
/// (empty include = all links). Mesh resources are fetched via resource_retriever
/// so package:// and file:// URIs both work.
std::vector<LinkGeometry> load_link_meshes(
  const std::string & urdf_xml,
  bool use_collision,
  const std::vector<std::string> & include,
  const std::vector<std::string> & exclude);

}  // namespace depth_self_filter
