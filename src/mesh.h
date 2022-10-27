#ifndef __PLYMESH_H__
#define __PLYMESH_H__

#include <span>
#include <string>

#include "envoy.h"

EVY_NAMESPACE_BEGIN
class GResource;

struct TriangleMesh {
  std::size_t         num_indices, num_vertices;
  std::span<uint32_t> indices;
  std::span<vec3>     vertex_positions;
  std::span<vec3>     vertex_normals;
  std::span<vec2>     vertex_uv;
};

TriangleMesh MakeTriangleMesh(const std::string &path, GResource &resource);

EVY_NAMESPACE_END

#endif