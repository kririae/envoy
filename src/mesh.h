#ifndef __PLYMESH_H__
#define __PLYMESH_H__

#include <span>
#include <string>

#include "common.h"

EVY_NAMESPACE_BEGIN

class GResource;

struct TriangleMesh {
  std::size_t         num_indices, num_vertices;
  std::span<uint32_t> indices;
  std::span<Vector3f> vertex_positions;
  std::span<Vector3f> vertex_normals;
  std::span<Vector2f> vertex_uv;
};

TriangleMesh MakeTriangleMesh(const std::string &path, GResource &resource);

EVY_NAMESPACE_END

#endif