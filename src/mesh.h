#ifndef __PLYMESH_H__
#define __PLYMESH_H__

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_selectors.hpp>
#include <boost/graph/graph_traits.hpp>
#include <cstddef>
#include <optional>
#include <span>
#include <string>

#include "envoy_common.h"
#include "pages.h"

EVY_NAMESPACE_BEGIN

class GResource;

/**
 * The naive triangle mesh data structure
 */
struct TriangleMesh {
  std::size_t         num_indices, num_vertices;
  std::span<uint32_t> indices;
  std::span<Vec3f>    vertex_positions;

  /* optional properties of mesh */
  std::optional<std::span<Vec3f>> vertex_normals{std::nullopt};
  std::optional<std::span<Vec2f>> vertex_uv{std::nullopt};
};

struct SubTriangleMesh {
  std::span<uint32_t> indices;
  TriangleMesh       *triangle_mesh;
};

class SurfaceMesh {
public:
  using graph_type =
      boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS>;

private:
};

/**
 * @brief Load the .ply mesh from `path` specified in relative to assets/(for
 * now).
 *
 * @param path The relative path in assets/
 * @param resource The memory resource instance
 * @return TriangleMesh
 */
TriangleMesh MakeTriangleMesh(const std::string &path, GResource &resource);
std::span<SysPage<TriangleV>> MakeTrianglePages(const TriangleMesh &mesh,
                                                GResource          &resource);

EVY_NAMESPACE_END

#endif