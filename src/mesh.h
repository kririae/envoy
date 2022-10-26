#ifndef __PLYMESH_H__
#define __PLYMESH_H__

#include <miniply.h>

#include <span>
#include <string>

#include "envoy.h"
#include "resource_manager.h"

EVY_NAMESPACE_BEGIN

struct TriangleMesh {
  std::size_t         num_indices, num_vertices;
  std::span<uint32_t> indices;
  std::span<float3>   vertex_positions;
  std::span<float3>   vertex_normals;
  std::span<float2>   vertex_uv;
};

TriangleMesh MakeTriangleMesh(const std::string &path, GResource &resource);

EVY_NAMESPACE_END

#endif