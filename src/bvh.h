#ifndef __BVH_H__
#define __BVH_H__

#include <embree3/rtcore.h>
#include <embree3/rtcore_geometry.h>

#include <array>

#include "envoy.h"
#include "envoy_common.h"
#include "geometry.h"
#include "intersector.h"
#include "mesh.h"
#include "packer.h"
#include "pages.h"
#include "resource_manager.h"

EVY_NAMESPACE_BEGIN

struct alignas(16) BvhRayHit {
  Vec3f ray_o{0.0};                              /* init */
  Vec3f ray_d{0.0};                              /* init */
  float tnear{0.0};                              /* init */
  float tfar{std::numeric_limits<float>::max()}; /* init, modified */

  Vec3f hit_ng{0.0}; /* modified */
  Vec3f hit_ns{0.0}; /* modified */
  bool  hit{false};  /* modified */
};

class BvhBase {
public:
  virtual ~BvhBase()                          = default;
  virtual void   build()                      = 0;
  virtual BBox3f getBound() const             = 0;
  virtual bool   intersect(BvhRayHit &rayhit) = 0;

  /* print the corresponding build info */
  virtual void build_info();
};

class SerialBvh : public BvhBase {
public:
  SerialBvh(TriangleMesh &mesh, GResource &resource)
      : m_mesh(mesh),
        m_resource(resource),
        m_triangles(CastMeshToTriangleV(mesh, resource)) {}
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
  bool  nonRecursiveIntersect(Node *node, BvhRayHit &rayhit);
};

class RadixBvh : public BvhBase {
public:
  struct Node {
    BBox3f bound{};
    Node  *left{nullptr}, *right{nullptr};

    std::span<TriangleVPack> pack{};

    int     flag{}, direction{}, split_point{}, n_triangles{}, parent{};
    uint8_t left_node_type : 1 {};   // 0: internal; 1: leaf
    uint8_t right_node_type : 1 {};  // 0: internal; 1: leaf
  };

  struct RadixTriangleVPack : public TriangleVPack {
    uint32_t morton_code, parent;
  };

  RadixBvh(TriangleMesh &mesh, GResource &resource);
  ~RadixBvh() override;

  BBox3f getBound() const override;
  void   build() override;
  bool   intersect(BvhRayHit &rayhit) override;

private:
  BBox3f                        m_bound;
  std::span<TriangleVPack>      m_packs;
  std::span<RadixTriangleVPack> m_radix_packs; /* converted from m_packs */
  TriangleMesh                 &m_mesh;
  GResource                    &m_resource;

  /* implementation-specific nodes */
  std::span<Node> m_internal_nodes;

  /* pre-processing for radix bvh */
  void prestage();

  /* build the BVH in parallel */
  void parallelBuilder();

  /* BVH intersect helper */
  bool recursiveIntersect(Node *node, BvhRayHit &rayhit);

  /* defined in the original paper */
  EVY_FORCEINLINE int delta(int i, int j) {
    if (j < 0 || j >= m_radix_packs.size()) return -1;
    if (m_radix_packs[i].morton_code == m_radix_packs[j].morton_code)
      return __builtin_clz(i ^ j) + 32;
    return __builtin_clz(m_radix_packs[i].morton_code ^
                         m_radix_packs[j].morton_code);
  }
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
