#include "bvh.h"

#include <oneapi/tbb/parallel_sort.h>

#include <iostream>
#include <span>

#include "envoy.h"
#include "envoy_common.h"
#include "geometry.h"
#include "math_aliases.h"

EVY_NAMESPACE_BEGIN

namespace detail_ {
TriangleV CastIndicesToTriangleV(const std::span<uint32_t> indices,
                                 const std::span<Vec3f>   &positions) {
  const int num_indices = indices.size();
  assert(num_indices % 3 == 0);

  Vec3vf             v[3];
  const int          num_triangles = num_indices / 3;
  constexpr uint32_t stride        = TriangleV::size;

  auto get_vfloats = [num_indices, positions, indices](int indices_offset) {
    std::array<std::array<float, stride>, 3> vfloats;
    for (int i_indices = indices_offset, i_triangles = 0;
         i_indices < num_indices; i_indices += 3, ++i_triangles) {
      for (int k = 0; k < 3; ++k)
        vfloats[k][i_triangles] = positions[indices[i_indices]][k];
    }
    return vfloats;
  };

  // vfloats[{x, y, z}][{1, ... simd_width}]
  for (int i = 0; i < 3; ++i) {
    const auto vfloats = get_vfloats(i);
    v[i].x             = vfloat::load_unaligned(vfloats[0].data());
    v[i].y             = vfloat::load_unaligned(vfloats[1].data());
    v[i].z             = vfloat::load_unaligned(vfloats[2].data());
  }

  return TriangleV(
      LeadingMask<float>(num_triangles, TriangleV::vfloat_type::arch_type{}),
      v[0], v[1], v[2]);
}

std::span<TriangleV> CastMeshToTriangleV(TriangleMesh mesh,
                                         GResource   &resource) {
  constexpr uint32_t stride = TriangleV::size;
  std::size_t        num_triangle_v =
      std::ceil(static_cast<double>(mesh.num_indices) / 3.0 / stride);
  Info("mesh.num_triangles:        {}", num_triangle_v * stride);
  Info("mesh.num_packed_triangles: {}", num_triangle_v);
  Info("packed triangle width:     {}", stride);
  std::span<TriangleV> result =
      std::span{resource.alloc<TriangleV[]>(num_triangle_v), num_triangle_v};
  for (std::size_t i_indices = 0, i_triangle_v = 0;
       i_indices < mesh.num_indices;
       i_indices += (stride * 3), ++i_triangle_v) {
    const std::size_t num_indices =
        std::min<std::size_t>(stride * 3, mesh.num_indices - i_indices);
    assert(num_indices % 3 == 0);
    result[i_triangle_v] = CastIndicesToTriangleV(
        mesh.indices.subspan(i_indices, num_indices), mesh.vertex_positions);
  }

  return result;
}

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
}  // namespace detail_

TriangleVPack::TriangleVPack(const std::span<TriangleV> &triangles) {
  assert(size >= triangles.size());
  m_num_valid_triangles = triangles.size();
  std::memcpy(m_triangles, triangles.data(),
              sizeof(TriangleV) * m_num_valid_triangles);
  m_bound = triangles[0].all_bound();
  for (auto &triangle : triangles) {
    m_bound = BBox3f::merge(m_bound, triangle.all_bound());
  }
}

bool TriangleVPack::intersect(BvhRayHit &rayhit) {
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

std::span<TriangleVPack> MakeTrianglePacksFromZOrderCurve(TriangleMesh &mesh,
                                                          GResource &resource) {
  const std::size_t trianglev_pack_size = TriangleVPack::size;
  const std::size_t num_triangles       = mesh.num_indices / 3;
  const std::size_t num_triangle_pack =
      std::ceil(static_cast<double>(num_triangles) / trianglev_pack_size);

  // Alloc the result
  std::span<TriangleVPack> result = std::span{
      resource.alloc<TriangleVPack[]>(num_triangle_pack), num_triangle_pack};
  std::span<Vec3i> indices_triangle_view{
      reinterpret_cast<Vec3i *>(mesh.indices.data()), mesh.num_indices / 3};

  // TODO: optimize this sort
  // Sort the triangles by their morton code
  tbb::parallel_sort(
      indices_triangle_view.begin(), indices_triangle_view.end(),
      [&mesh](const Vec3i &x, const Vec3i &y) -> bool {
        return detail_::EncodeMorton3(mesh.vertex_positions[x.x]) <
               detail_::EncodeMorton3(mesh.vertex_positions[y.x]);
      });
}

void SerialBvh::build() {
  m_bound = triangles[0].all_bound();
  for (auto &triangle : triangles) {
    m_bound = BBox3f::merge(m_bound, triangle.all_bound());
  }
}

bool SerialBvh::intersect(BvhRayHit &rayhit) {
  bool  hit  = false;
  float tfar = rayhit.tfar;
  for (const auto &triangle : triangles) {
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

EVY_NAMESPACE_END