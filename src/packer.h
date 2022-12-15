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

  std::size_t                     m_num_valid_triangles;
  BBox3f                          m_bound;
  std::array<triangle_type, size> m_triangles;
};

std::span<TriangleVPack> MakeTrianglePacksFromZOrderCurve(TriangleMesh &mesh,
                                                          GResource &resource);
std::span<TriangleVPack> MakeTrianglePacksFromBVH(TriangleMesh &mesh,
                                                  GResource    &resource);

EVY_FORCEINLINE uint32_t LeftShift3(uint32_t x) {
  if (x == (1 << 10)) --x;
  x = (x | (x << 16)) & 0b00000011000000000000000011111111;
  x = (x | (x << 8)) & 0b00000011000000001111000000001111;
  x = (x | (x << 4)) & 0b00000011000011000011000011000011;
  x = (x | (x << 2)) & 0b00001001001001001001001001001001;
  return x;
}

EVY_FORCEINLINE uint32_t EncodeMorton3(const Vec3f &v) {
  return (LeftShift3(v.z) << 2) | (LeftShift3(v.y) << 1) | LeftShift3(v.x);
}

EVY_NAMESPACE_END

#endif
