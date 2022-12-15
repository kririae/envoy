#include "bvh.h"

#include <algorithm>
#include <array>
#include <iostream>
#include <span>

#include "envoy_common.h"
#include "geometry.h"
#include "intersector.h"
#include "math_aliases.h"
#include "mesh.h"
#include "packer.h"
#include "putils.hpp"

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
  // return recursiveIntersect(m_root, rayhit);
  return nonRecursiveIntersect(m_root, rayhit);
}

BaseBvh::Node *BaseBvh::recursiveBuilder(const std::span<TriangleVPack> &packs,
                                         int depth) {
  using tpack_iterator = std::span<TriangleVPack>::iterator;
  if (packs.empty()) return nullptr;
  assert(packs.data() != nullptr);

  auto bound = packs[0].bound();
  for (auto &pack : packs) bound = BBox3f::merge(bound, pack.bound());
  Node *node = m_resource.alloc<Node, 32>();

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

bool BaseBvh::nonRecursiveIntersect(Node *node, BvhRayHit &rayhit) {
  constexpr int STACK_SIZE = 64;
  if (node == nullptr) return false;

  struct Item {
    Node *node;
    float tnear;
  };

  // Let's traverse on this local stack
  std::array<Item, STACK_SIZE> stack{};

  int stack_ptr      = 0;
  stack[++stack_ptr] = Item{node, 0};

  // Manual DFS traversal
  while (stack_ptr > 0) {
    // pop the first element
    Item it = stack[stack_ptr--];
    if (it.node == nullptr) continue;

    if (rayhit.tfar < it.tnear) {
      continue;
    } else if (!it.node->pack.empty()) {
      // Intersecting leaf node
      for (auto &pack : it.node->pack) pack.intersect(rayhit);
    } else {
      // Perform two intersections
      float left_tnear, right_tnear, tfar;
      bool  left_inter  = BoundIntersect1(it.node->left->bound, rayhit.ray_o,
                                          rayhit.ray_d, left_tnear, tfar);
      bool  right_inter = BoundIntersect1(it.node->right->bound, rayhit.ray_o,
                                          rayhit.ray_d, right_tnear, tfar);
      if (left_inter && right_inter) {
        auto left  = it.node->left;
        auto right = it.node->right;
        if (left_tnear < right_tnear) {
          std::swap(left_inter, right_inter);
          std::swap(left, right);
        }

        // Now, traverse right first
        stack[++stack_ptr] = Item{it.node->left, left_tnear};
        stack[++stack_ptr] = Item{it.node->right, right_tnear};
      } else if (left_inter) {
        stack[++stack_ptr] = Item{it.node->left, left_tnear};
      } else if (right_inter) {
        stack[++stack_ptr] = Item{it.node->right, right_tnear};
      }
    }

    assert(stack_ptr <= STACK_SIZE);
  }

  return rayhit.hit;
}

RadixBvh::RadixBvh(TriangleMesh &mesh, GResource &resource)
    : m_mesh(mesh),
      m_resource(resource),
      m_packs(envoy::MakeTrianglePacksFromBVH(mesh, resource)) {}

RadixBvh::~RadixBvh() {
  // TODO: memory overhead for simple implementation
  // m_resource.dealloc(m_radix_packs.data());
}

BBox3f RadixBvh::getBound() const {
  return m_bound;
}

void RadixBvh::build() {
  prestage();
  parallelBuilder();
  auto vec = m_internal_nodes[0].bound.lower;
  Info("build info: {} {} {}", vec.x, vec.y, vec.z);
}

bool RadixBvh::intersect(BvhRayHit &rayhit) {
  return recursiveIntersect(&m_internal_nodes[0], rayhit);
}

void RadixBvh::prestage() {
  const std::size_t m_n_triangles = m_packs.size();

  // Then build m_radix_packs from m_packs
  m_radix_packs = std::span{
      m_resource.alloc<RadixTriangleVPack[]>(m_n_triangles), m_n_triangles};

  m_internal_nodes =
      std::span{m_resource.alloc<Node[]>(m_n_triangles - 1), m_n_triangles - 1};

  m_bound = tbb::parallel_reduce(
      tbb::blocked_range<std::size_t>(0, m_n_triangles), BBox3f{},
      [&](const tbb::blocked_range<std::size_t> &r, BBox3f init) -> BBox3f {
        for (std::size_t i = r.begin(); i != r.end(); ++i)
          init = BBox3f::merge(init, m_packs[i].bound());
        return init;
      },
      [](const BBox3f &b1, const BBox3f &b2) -> BBox3f {
        return BBox3f::merge(b1, b2);
      });

  Vec3f edges = m_bound.upper - m_bound.lower;
  ParallelForLinear(0, m_n_triangles, [&](std::size_t i) {
    const auto bound                       = m_packs[i].bound();
    Vec3f      centroid                    = (bound.lower + bound.upper) / 2;
    Vec3f      n_coord                     = (centroid - m_bound.lower) / edges;
    m_radix_packs[i].morton_code           = EncodeMorton3(n_coord * (1 << 10));
    m_radix_packs[i].m_num_valid_triangles = m_packs[i].m_num_valid_triangles;
    m_radix_packs[i].m_bound               = m_packs[i].m_bound;
    m_radix_packs[i].m_triangles           = m_packs[i].m_triangles;
  });

  // TODO: replace this implementation with
  // https://github.com/google/highway/tree/master/hwy/contrib/sort
  tbb::parallel_sort(
      m_radix_packs.begin(), m_radix_packs.end(),
      [](const RadixTriangleVPack &a, const RadixTriangleVPack &b) -> bool {
        return a.morton_code < b.morton_code;
      });
}

void RadixBvh::parallelBuilder() {
  const std::size_t m_n_triangles = m_packs.size();

  ParallelForLinear(0, m_n_triangles - 1, [&](std::size_t i) {
    Node &internal_node     = m_internal_nodes[i];
    internal_node.direction = Sign(delta(i, i + 1) - delta(i, i - 1));
    const int d             = internal_node.direction;
    assert(d != 0);

    // move toward the sibling node
    int delta_min = delta(i, i - d);
    int l_max     = 2;
    while (delta(i, i + l_max * d) > delta_min) /* obtain the maximum l_max */
      l_max <<= 1;

    // Moving rightward
    int l = 0;
    for (int t = l_max / 2; t >= 1; t >>= 1)
      if (delta(i, i + (l + t) * d) > delta_min) l += t;
    int j = i + l * d;  // the other bound, inclusive

    int delta_node = delta(i, j);
    int s          = 0;
    for (int t = Next2Pow(l); t >= 1; t >>= 1)
      if (delta(i, i + (s + t) * d) > delta_node) s += t;

    // Finally decide the split point, always chose the left one, neglecting the
    // internal_node.direction
    internal_node.split_point = i + s * d + std::min(d, 0);
    const auto split_point    = internal_node.split_point;
    if (std::min(static_cast<int>(i), j) == split_point)
      internal_node.left_node_type = 1;
    if (std::max(static_cast<int>(i), j) == split_point + 1)
      internal_node.right_node_type = 1;

    // Final pass, calculate the parent
    if (internal_node.left_node_type)
      m_radix_packs[split_point].parent = i;
    else
      m_internal_nodes[split_point].parent = i;

    if (internal_node.right_node_type)
      m_radix_packs[split_point + 1].parent = i;
    else
      m_internal_nodes[split_point + 1].parent = i;
  });

  ParallelForLinear(0, m_n_triangles, [&](std::size_t i) {
    int    current_index  = m_radix_packs[i].parent;
    int    previous_index = i;
    BBox3f subnode_bound  = m_radix_packs[i].bound();
    assert(0 <= current_index && current_index < m_n_triangles - 1);
    while (true) {
      auto &internal_node = m_internal_nodes[current_index];
      // Increment the flag by one, and fetch the original value
      int previous_flag = __sync_fetch_and_add(&internal_node.flag, 1);
      if (previous_flag == 0)
        break;
      else if (previous_flag == 1) {
        BBox3f other_bound{};
        int    other_num = 0;
        if (previous_index == internal_node.split_point) {
          other_bound =
              internal_node.right_node_type == 1
                  ? m_radix_packs[internal_node.split_point + 1].bound()
                  : m_internal_nodes[internal_node.split_point + 1].bound;
        } else {
          assert(previous_index == internal_node.split_point + 1);
          other_bound = internal_node.left_node_type == 1
                          ? m_radix_packs[internal_node.split_point].bound()
                          : m_internal_nodes[internal_node.split_point].bound;
        }

        internal_node.bound = BBox3f{};
        internal_node.bound = BBox3f::merge(internal_node.bound, other_bound);
        internal_node.bound = BBox3f::merge(internal_node.bound, subnode_bound);
      }

      subnode_bound = internal_node.bound;
      if (current_index == 0 && m_internal_nodes[0].parent == 0) break;
      previous_index = current_index;
      current_index  = m_internal_nodes[current_index].parent;
    }
  });
}

bool RadixBvh::recursiveIntersect(Node *node, BvhRayHit &rayhit) {
  if (node == nullptr) return false;
  float tnear, tfar;
  bool  inter =
      BoundIntersect1(node->bound, rayhit.ray_o, rayhit.ray_d, tnear, tfar);

  if (!inter || rayhit.tfar < tnear) {
    return false;
  } else {
    int                                        n_packs = 0;
    static std::array<RadixTriangleVPack *, 2> packs{};
    if (node->left_node_type)
      packs[n_packs++] = &m_radix_packs[node->split_point];
    if (node->right_node_type)
      packs[n_packs++] = &m_radix_packs[node->split_point + 1];

    bool hit = false, res_left = false, res_right = false;
    if (n_packs == 1) hit |= packs[0]->intersect(rayhit);
    if (n_packs == 2) hit |= packs[1]->intersect(rayhit);
    if (!node->left_node_type)
      res_left =
          recursiveIntersect(&m_internal_nodes[node->split_point], rayhit);
    if (!node->right_node_type)
      res_right =
          recursiveIntersect(&m_internal_nodes[node->split_point + 1], rayhit);
    return res_left || res_right || hit;
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
