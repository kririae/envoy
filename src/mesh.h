#ifndef __PLYMESH_H__
#define __PLYMESH_H__

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_selectors.hpp>
#include <boost/graph/graph_traits.hpp>
#include <cstddef>
#include <span>
#include <string>

#include "envoy_common.h"

EVY_NAMESPACE_BEGIN

class GResource;

/**
 * The naive triangle mesh data structure
 */
struct TriangleMesh {
  std::size_t         num_indices, num_vertices;
  std::span<uint32_t> indices;
  std::span<Vec3f>    vertex_positions;
  std::span<Vec3f>    vertex_normals;
  std::span<Vec2f>    vertex_uv;
};

struct SubTriangleMesh {
  std::size_t         num_indices, num_vertices;
  std::span<uint32_t> indices;
  TriangleMesh       *triangle_mesh;
};

class SurfaceMesh {
public:
  using graph_type =
      boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS>;

private:
};

TriangleMesh MakeTriangleMesh(const std::string &path, GResource &resource);

EVY_NAMESPACE_END

#endif