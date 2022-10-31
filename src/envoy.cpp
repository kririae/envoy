#include "bvh.h"
#include "envoy_common.h"
#include "geometry.h"
#include "intersector.h"
#include "lru.h"
#include "mesh.h"
#include "resource_manager.h"
#include "stats.h"

using namespace envoy;

// @note from
// https://github.com/facebookincubator/velox/blob/main/velox/common/base/SimdUtil-inl.h
template <typename T, typename A>
xsimd::batch_bool<T, A> leadingMask(int n, const A &) {
  constexpr int     N     = xsimd::batch_bool<T, A>::size;
  static const auto kMemo = ({
    std::array<xsimd::batch_bool<T, A>, N> memo;
    bool                                   tmp[N]{};
    for (int i = 0; i < N; ++i) {
      memo[i] = xsimd::batch_bool<T, A>::load_unaligned(tmp);
      tmp[i]  = true;
    }
    memo;
  });
  return (n >= N) ? xsimd::batch_bool<T, A>(true) : kMemo[n];
}

inline static TriangleV CastIndicesToTriangleV(
    const std::span<uint32_t> indices, const std::span<Vec3f> &positions) {
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
      leadingMask<float>(num_triangles, TriangleV::vfloat_type::arch_type{}),
      v[0], v[1], v[2]);
}

inline static std::span<TriangleV> CastMeshToTriangleV(TriangleMesh mesh,
                                                       GResource   &resource) {
  constexpr uint32_t stride = TriangleV::size;
  std::size_t        num_triangle_v =
      std::ceil(static_cast<double>(mesh.num_indices) / 3.0 / stride);
  Info("mesh.num_triangles {}", num_triangle_v * stride);
  Info("mesh.num_packed_triangles {}", num_triangle_v);
  Info("packed triangle width {}", stride);
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

int main() {
  GResource resource;
  auto      mesh = MakeTriangleMesh(__get_asset_path("sphere.ply"), resource);
  auto      triangles = CastMeshToTriangleV(mesh, resource);

  auto bvh = SerialBvh(triangles, resource);
  bvh.build();
  BvhRayHit rayhit;
  rayhit.ray_o = Vec3f{0.0};
  rayhit.ray_d = Vec3f{1.0, 0.0, 0.0};
  bool _       = bvh.intersect(rayhit);
  Info("{} {} {}", _, rayhit.tfar, rayhit.hit_ng.x);
  stats::Report();

  return 0;
}
