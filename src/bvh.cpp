#include "bvh.h"

#include <algorithm>
#include <iostream>
#include <span>

#include "envoy_common.h"
#include "geometry.h"
#include "intersector.h"
#include "math_aliases.h"
#include "mesh.h"
#include "packer.h"

EVY_NAMESPACE_BEGIN

namespace detail_ {
EVY_FORCEINLINE void EmbreeErrorFunction([[maybe_unused]] void *userPtr,
                                         enum RTCError error, const char *str) {
  Error("embree error {}: {}", error, str);
}

EVY_FORCEINLINE RTCDevice EmbreeInitializeDevice() {
  RTCDevice device = rtcNewDevice(nullptr);
  if (!device)
    Error("embree error %d: cannot create device\n",
          rtcGetDeviceError(nullptr));
  rtcSetDeviceErrorFunction(device, EmbreeErrorFunction, nullptr);
  return device;
}

auto SurfaceArea(const BBox3f &x) {
  auto sides = x.upper - x.lower;
  return 2 * (sides.x * sides.y + sides.y * sides.z + sides.z * sides.z);
}

auto Center(const TriangleVPack &x) {
  const auto bound = x.bound();
  return (bound.lower + bound.upper) / 2;
}
}  // namespace detail_

void SerialBvh::build() {
  m_bound = m_triangles[0].all_bound();
  for (auto &triangle : m_triangles) {
    m_bound = BBox3f::merge(m_bound, triangle.all_bound());
  }
}

bool SerialBvh::intersect(BvhRayHit &rayhit) {
  bool  hit  = false;
  float tfar = rayhit.tfar;
  for (const auto &triangle : m_triangles) {
    float thit;
    Vec3f ng;
    bool  inter = PlueckerTriangleIntersect1(triangle, rayhit.ray_o,
                                             rayhit.ray_d, thit, ng);
    if (!inter) continue;
    if (thit > tfar) continue;
    hit           = true;
    tfar          = thit;
    rayhit.hit    = hit;
    rayhit.tfar   = tfar;
    rayhit.hit_ng = rayhit.hit_ns = ng;
  }

  return hit;
}

BaseBvh::BaseBvh(TriangleMesh &mesh, GResource &resource)
    : m_mesh(mesh),
      m_resource(resource),
      m_packs(envoy::MakeTrianglePacksFromBVH(mesh, resource)) {}

void BaseBvh::build() {
  m_root = recursiveBuilder(m_packs, 0);
}

bool BaseBvh::intersect(BvhRayHit &rayhit) {
  return recursiveIntersect(m_root, rayhit);
}

BaseBvh::Node *BaseBvh::recursiveBuilder(const std::span<TriangleVPack> &packs,
                                         int depth) {
  using tpack_iterator = std::span<TriangleVPack>::iterator;
  if (packs.empty()) return nullptr;
  assert(packs.data() != nullptr);

  auto bound = packs[0].bound();
  for (auto &pack : packs) bound = BBox3f::merge(bound, pack.bound());
  Node *node = m_resource.alloc<Node>();

  if (packs.size() <= 1) {
    assert(packs.size() != 0);
    node->bound = bound;
    node->pack  = packs;
    return node;
  }

  assert(packs.size() >= 2);
  const int      dim      = depth % 3;
  tpack_iterator mid_pack = packs.begin() + packs.size() / 2;

#if 0
  std::nth_element(
      packs.begin(), mid_pack, packs.end(),
      [dim](const TriangleVPack &a, const TriangleVPack &b) -> bool {
        return detail_::Center(a)[dim] < detail_::Center(b)[dim];
      });
  float mid = detail_::Center(*mid_pack)[dim];
  mid_pack  = std::partition(packs.begin(), packs.end(),
                             [mid, dim](const TriangleVPack &a) -> bool {
                              return detail_::Center(a)[dim] < mid;
                            });  // std::partition
#endif

  constexpr int                  num_buckets = 16;
  std::pair<BBox3f, std::size_t> buckets[num_buckets], prefix[num_buckets],
      suffix[num_buckets];
  std::array<float, num_buckets> cost;

  for (auto &pack : packs) {
    int b = std::floor(num_buckets * ((detail_::Center(pack) - bound.lower) /
                                      (bound.upper - bound.lower))[dim]);
    b     = std::clamp(b, 0, num_buckets - 1);
    buckets[b].first = BBox3f::merge(buckets[b].first, pack.bound());
    buckets[b].second++;
  }

  // Run inclusive_sum in two directions
  prefix[0] = buckets[0];
  for (int i = 1; i < num_buckets; ++i) {
    prefix[i]       = prefix[i - 1];
    prefix[i].first = BBox3f::merge(prefix[i].first, buckets[i].first);
    prefix[i].second += buckets[i].second;
  }

  suffix[num_buckets - 1] = buckets[num_buckets - 1];
  for (int i = num_buckets - 2; i >= 0; --i) {
    suffix[i]       = suffix[i + 1];
    suffix[i].first = BBox3f::merge(suffix[i].first, buckets[i].first);
    suffix[i].second += buckets[i].second;
  }

  float min_cost       = std::numeric_limits<float>::max();
  int   min_cost_index = 0;
  for (int i = 1; i < num_buckets - 1; ++i) {
    cost[i] =
        1.0 / 8 +
        (detail_::SurfaceArea(prefix[i].first) * prefix[i].second +
         detail_::SurfaceArea(suffix[i + 1].first) * suffix[i + 1].second) /
            detail_::SurfaceArea(bound);
    if (cost[i] < min_cost) {
      min_cost       = cost[i];
      min_cost_index = i;
    }
  }

  float no_partition_cost = packs.size();
  if (min_cost < no_partition_cost) {
    mid_pack = std::partition(
        packs.begin(), packs.end(),
        [&bound, dim, min_cost_index,
         num_buckets](const TriangleVPack &t) -> bool {
          int b = std::floor(num_buckets * ((detail_::Center(t) - bound.lower) /
                                            (bound.upper - bound.lower))[dim]);
          b     = std::clamp(b, 0, num_buckets - 1);
          return b <= min_cost_index;
        });
  } else {
    node->bound = bound;
    node->pack  = packs;
    return node;
  }

  const auto packs_left  = std::span{packs.begin(), mid_pack};
  const auto packs_right = std::span{mid_pack, packs.end()};
  node->bound            = bound;
  node->left =
      packs_left.empty() ? nullptr : recursiveBuilder(packs_left, depth + 1);
  node->right =
      packs_right.empty() ? nullptr : recursiveBuilder(packs_right, depth + 1);
  return node;
}

bool BaseBvh::recursiveIntersect(Node *node, BvhRayHit &rayhit) {
  if (node == nullptr) return false;
  float tnear, tfar;
  bool  inter =
      BoundIntersect1(node->bound, rayhit.ray_o, rayhit.ray_d, tnear, tfar);

  if (!inter || rayhit.tfar < tnear) {
    return false;
  } else if (!node->pack.empty()) {
    bool hit = false;
    for (auto &pack : node->pack) hit |= pack.intersect(rayhit);
    return hit;
  } else {
    bool res1 = recursiveIntersect(node->left, rayhit);
    bool res2 = recursiveIntersect(node->right, rayhit);
    return res1 || res2;
  }
}

EmbreeBvh::EmbreeBvh(const TriangleMesh &mesh, GResource &resource) {
  m_device = detail_::EmbreeInitializeDevice();
  m_scene  = rtcNewScene(m_device);
  m_geom   = rtcNewGeometry(m_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  std::span<float> vertices{
      static_cast<float *>(rtcSetNewGeometryBuffer(
          m_geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3,
          3 * sizeof(float), mesh.num_vertices)),
      3 * mesh.num_vertices};
  std::span<uint32_t> indices{
      static_cast<uint32_t *>(rtcSetNewGeometryBuffer(
          m_geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3,
          3 * sizeof(uint32_t), mesh.num_indices / 3)),
      mesh.num_indices};

  // Still, I have to use std::memcpy
  std::memcpy(vertices.data(), mesh.vertex_positions.data(),
              3 * sizeof(float) * mesh.num_vertices);
  std::memcpy(indices.data(), mesh.indices.data(),
              sizeof(int) * mesh.num_indices);
}

EmbreeBvh::~EmbreeBvh() {
  Info("Embree BVH is destructed");
}

BBox3f EmbreeBvh::getBound() const {
  auto *bounds =
      static_cast<RTCBounds *>(std::aligned_alloc(16, sizeof(RTCBounds)));
  rtcGetSceneBounds(m_scene, bounds);
  BBox3f result{
      Vec3f{bounds->lower_x, bounds->lower_y, bounds->lower_z},
      Vec3f{bounds->upper_x, bounds->upper_y, bounds->upper_z}
  };
  std::free(bounds);
  return result;
}

void EmbreeBvh::build() {
  rtcSetGeometryBuildQuality(m_geom, RTC_BUILD_QUALITY_LOW);
  rtcCommitGeometry(m_geom);
  m_geomID = rtcAttachGeometry(m_scene, m_geom);
  rtcReleaseGeometry(m_geom);
  rtcCommitScene(m_scene);  // start building
}

bool EmbreeBvh::intersect(BvhRayHit &rayhit) {
  RTCIntersectContext context;
  rtcInitIntersectContext(&context);

  RTCRayHit r_rayhit;
  r_rayhit.ray.org_x     = rayhit.ray_o.x;
  r_rayhit.ray.org_y     = rayhit.ray_o.y;
  r_rayhit.ray.org_z     = rayhit.ray_o.z;
  r_rayhit.ray.dir_x     = rayhit.ray_d.x;
  r_rayhit.ray.dir_y     = rayhit.ray_d.y;
  r_rayhit.ray.dir_z     = rayhit.ray_d.z;
  r_rayhit.ray.tnear     = rayhit.tnear;
  r_rayhit.ray.tfar      = rayhit.tfar;
  r_rayhit.ray.mask      = -1;
  r_rayhit.ray.flags     = 0;
  r_rayhit.hit.geomID    = RTC_INVALID_GEOMETRY_ID;
  r_rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

  rtcIntersect1(m_scene, &context, &r_rayhit);

  if (r_rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
    rayhit.tfar   = r_rayhit.ray.tfar;
    rayhit.hit_ng = rayhit.hit_ns = Normalize(
        Vec3f{r_rayhit.hit.Ng_x, r_rayhit.hit.Ng_y, r_rayhit.hit.Ng_z});
    rayhit.hit = true;
    return true;
  } else {
    return false;
  }
}

EVY_NAMESPACE_END