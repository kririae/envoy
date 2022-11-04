#ifndef __PACKER_H__
#define __PACKER_H__

#include "envoy_common.h"
#include "geometry.h"
#include "intersector.h"
#include "mesh.h"
#include "pages.h"
#include "resource_manager.h"

EVY_NAMESPACE_BEGIN

TriangleV            CastIndicesToTriangleV(const std::span<uint32_t> indices,
                                            const std::span<Vec3f>   &positions);
std::span<TriangleV> CastMeshToTriangleV(TriangleMesh mesh,
                                         GResource   &resource);

struct BvhRayHit;

struct TriangleVPack /* : public SysPage<TriangleV> */ {
public:
  using triangle_type               = TriangleV;
  static constexpr std::size_t size = 1; /* ??? */

  TriangleVPack() = default;
  TriangleVPack(const std::span<TriangleV> &triangles);
  BBox3f bound() const { return m_bound; }
  bool   intersect(BvhRayHit &rayhit);

  std::size_t   m_num_valid_triangles;
  BBox3f        m_bound;
  triangle_type m_triangles[size];
};

std::span<TriangleVPack> MakeTrianglePacksFromZOrderCurve(TriangleMesh &mesh,
                                                          GResource &resource);
std::span<TriangleVPack> MakeTrianglePacksFromBVH(TriangleMesh &mesh,
                                                  GResource    &resource);

EVY_NAMESPACE_END

#endif
