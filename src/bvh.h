#ifndef __BVH_H__
#define __BVH_H__

#include <embree3/rtcore.h>
#include <embree3/rtcore_geometry.h>

#include "envoy.h"
#include "envoy_common.h"
#include "geometry.h"
#include "intersector.h"
#include "mesh.h"
#include "pages.h"
#include "resource_manager.h"

EVY_NAMESPACE_BEGIN
namespace detail_ {
TriangleV            CastIndicesToTriangleV(const std::span<uint32_t> indices,
                                            const std::span<Vec3f>   &positions);
std::span<TriangleV> CastMeshToTriangleV(TriangleMesh mesh,
                                         GResource   &resource);
}  // namespace detail_

struct alignas(16) BvhRayHit {
  Vec3f ray_o{0.0};                              /* init */
  Vec3f ray_d{0.0};                              /* init */
  float tnear{0.0};                              /* init */
  float tfar{std::numeric_limits<float>::max()}; /* init, modified */

  Vec3f hit_ng{0.0}; /* modified */
  Vec3f hit_ns{0.0}; /* modified */
  bool  hit{false};  /* modified */
};

struct TriangleVPack /* : public SysPage<TriangleV> */ {
public:
  using triangle_type               = TriangleV;
  static constexpr std::size_t size = 1; /* ??? */

  EVY_FORCEINLINE        TriangleVPack() = default;
  EVY_FORCEINLINE        TriangleVPack(const std::span<TriangleV> &triangles);
  EVY_FORCEINLINE BBox3f bound() const { return m_bound; }
  EVY_FORCEINLINE bool   intersect(BvhRayHit &rayhit);

  std::size_t   m_num_valid_triangles;
  BBox3f        m_bound;
  triangle_type m_triangles[size];
};

std::span<TriangleVPack> MakeTrianglePacksFromZOrderCurve(TriangleMesh &mesh,
                                                          GResource &resource);

class BvhBase {
public:
  virtual ~BvhBase()                          = default;
  virtual void   build()                      = 0;
  virtual BBox3f getBound() const             = 0;
  virtual bool   intersect(BvhRayHit &rayhit) = 0;
};

class SerialBvh : public BvhBase {
public:
  SerialBvh(TriangleMesh &mesh, GResource &resource)
      : m_mesh(mesh),
        m_resource(resource),
        m_triangles(detail_::CastMeshToTriangleV(mesh, resource)) {}
  ~SerialBvh() override = default;
  BBox3f getBound() const override { return m_bound; }
  void   build() override;
  bool   intersect(BvhRayHit &rayhit) override;

private:
  BBox3f               m_bound;
  std::span<TriangleV> m_triangles;

  TriangleMesh &m_mesh;
  GResource    &m_resource;
};

class BaseBvh : public BvhBase {
public:
  struct Node {
    BBox3f bound;
    Node  *left{nullptr}, *right{nullptr};

    std::span<TriangleVPack> pack;
  };

  BaseBvh(TriangleMesh &mesh, GResource &resource);
  ~BaseBvh() override = default;

  BBox3f getBound() const override { return m_root->bound; }
  void   build() override;
  bool   intersect(BvhRayHit &rayhit) override;

private:
  Node                    *m_root;
  std::span<TriangleVPack> m_packs;
  TriangleMesh            &m_mesh;
  GResource               &m_resource;

  Node *recursiveBuilder(const std::span<TriangleVPack> &packs, int depth);
  bool  recursiveIntersect(Node *node, BvhRayHit &rayhit);
};

class EmbreeBvh : public BvhBase {
public:
  EmbreeBvh(const TriangleMesh &mesh, GResource &resource);
  ~EmbreeBvh() override;

  BBox3f getBound() const override;
  void   build() override;
  bool   intersect(BvhRayHit &rayhit) override;

private:
  RTCDevice    m_device;
  RTCScene     m_scene;
  RTCGeometry  m_geom;
  unsigned int m_geomID;
};

EVY_NAMESPACE_END
#endif
