#include "packer.h"

#include <oneapi/tbb/parallel_sort.h>

#include "bvh.h"

EVY_NAMESPACE_BEGIN

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

EVY_FORCEINLINE static uint32_t LeftShift3(uint32_t x) {
  if (x == (1 << 10)) --x;
  x = (x | (x << 16)) & 0b00000011000000000000000011111111;
  x = (x | (x << 8)) & 0b00000011000000001111000000001111;
  x = (x | (x << 4)) & 0b00000011000011000011000011000011;
  x = (x | (x << 2)) & 0b00001001001001001001001001001001;
  return x;
}

EVY_FORCEINLINE static uint32_t EncodeMorton3(const Vec3f &v) {
  return (LeftShift3(v.z) << 2) | (LeftShift3(v.y) << 1) | LeftShift3(v.x);
}

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

static std::span<TriangleVPack> MakeTrianglePacksFromTriangleV(
    const std::span<TriangleV> &triangles, GResource &resource) {
  constexpr std::size_t trianglev_pack_size = TriangleVPack::size;
  const std::size_t num_triangle_pack = triangles.size() / trianglev_pack_size;
  std::span<TriangleVPack> result     = std::span{
      resource.alloc<TriangleVPack[]>(num_triangle_pack), num_triangle_pack};

  for (std::size_t i_triangle_pack = 0, i_triangle_v = 0;
       i_triangle_pack < num_triangle_pack;
       ++i_triangle_pack, i_triangle_v += trianglev_pack_size) {
    result[i_triangle_pack] = TriangleVPack{triangles.subspan(
        i_triangle_v,
        std::min(trianglev_pack_size, triangles.size() - i_triangle_v))};
  }

  return result;
}

static void MakeTrianglePacksFromBVHImpl(
    std::span<Vec3ui> &triangles, const int depth,
    const int num_triangles_per_pack, const TriangleMesh &mesh,
    GResource &resource, std::pmr::vector<TriangleVPack> &OutTriangleVPacks) {
  if (triangles.empty()) return;
  if (triangles.size() <= num_triangles_per_pack) {
    TriangleMesh sub_mesh = mesh;
    sub_mesh.num_indices  = triangles.size() * 3;
    sub_mesh.indices = std::span{reinterpret_cast<uint32_t *>(triangles.data()),
                                 triangles.size() * 3};
    auto triangle_v  = CastMeshToTriangleV(sub_mesh, resource);
    auto triangle_packs = MakeTrianglePacksFromTriangleV(triangle_v, resource);
    OutTriangleVPacks.insert(OutTriangleVPacks.end(), triangle_packs.begin(),
                             triangle_packs.end());
  } else {
    const int dim          = depth % 3;
    auto      mid_triangle = triangles.begin() + triangles.size() / 2;
    auto      get_center   = [&mesh](const Vec3ui &a) {
      return (mesh.vertex_positions[a.x] + mesh.vertex_positions[a.y] +
              mesh.vertex_positions[a.z]) /
             3;
    };
    std::nth_element(triangles.begin(), mid_triangle, triangles.end(),
                     [&](const Vec3ui &a, const Vec3ui &b) -> bool {
                       return get_center(a)[dim] < get_center(b)[dim];
                     });
    float mid            = get_center(*mid_triangle)[dim];
    mid_triangle         = std::partition(triangles.begin(), triangles.end(),
                                          [&](const Vec3ui &a) -> bool {
                                    return get_center(a)[dim] < mid;
                                  });  // std::partition
    auto triangles_left  = std::span{triangles.begin(), mid_triangle};
    auto triangles_right = std::span{mid_triangle, triangles.end()};
    MakeTrianglePacksFromBVHImpl(triangles_left, depth + 1,
                                 num_triangles_per_pack, mesh, resource,
                                 OutTriangleVPacks);
    MakeTrianglePacksFromBVHImpl(triangles_right, depth + 1,
                                 num_triangles_per_pack, mesh, resource,
                                 OutTriangleVPacks);
  }
}

std::span<TriangleVPack> MakeTrianglePacksFromZOrderCurve(TriangleMesh &mesh,
                                                          GResource &resource) {
  const std::size_t trianglev_pack_size = TriangleVPack::size;
  const std::size_t num_triangles       = mesh.num_indices / 3;
  const std::size_t num_triangle_pack =
      std::ceil(static_cast<double>(num_triangles) / trianglev_pack_size /
                TriangleVPack::triangle_type::size);

  std::span<Vec3ui> indices_triangle_view{
      reinterpret_cast<Vec3ui *>(mesh.indices.data()), mesh.num_indices / 3};

  tbb::parallel_sort(indices_triangle_view.begin(), indices_triangle_view.end(),
                     [&mesh](const Vec3ui &a, const Vec3ui &b) -> bool {
                       const auto center_x = mesh.vertex_positions[a.x] +
                                             mesh.vertex_positions[a.y] +
                                             mesh.vertex_positions[a.z];
                       const auto center_y = mesh.vertex_positions[b.x] +
                                             mesh.vertex_positions[b.y] +
                                             mesh.vertex_positions[b.z];
                       return EncodeMorton3(center_x / 3) <
                              EncodeMorton3(center_y / 3);
                     });

  const auto triangle_v = CastMeshToTriangleV(mesh, resource);

  // TODO
  // resource.dealloc(triangle_v.data());
  return MakeTrianglePacksFromTriangleV(triangle_v, resource);
}

std::span<TriangleVPack> MakeTrianglePacksFromBVH(TriangleMesh &mesh,
                                                  GResource    &resource) {
  const std::size_t trianglev_pack_size = TriangleVPack::size;
  const std::size_t num_triangles       = mesh.num_indices / 3;
  const std::size_t num_triangle_pack =
      std::ceil(static_cast<double>(num_triangles) / trianglev_pack_size /
                TriangleVPack::triangle_type::size);
  const std::size_t num_triangles_per_pack =
      trianglev_pack_size * TriangleV::size;
  Info("num triangles per pack: {}", num_triangles_per_pack);

  std::span<Vec3ui> indices_triangle_view{
      reinterpret_cast<Vec3ui *>(mesh.indices.data()), mesh.num_indices / 3};

  std::pmr::vector<TriangleVPack> trianglev_packs;
  MakeTrianglePacksFromBVHImpl(indices_triangle_view, 0, num_triangles_per_pack,
                               mesh, resource, trianglev_packs);
  auto result =
      std::span{resource.alloc<TriangleVPack[]>(trianglev_packs.size()),
                trianglev_packs.size()};
  std::copy(trianglev_packs.begin(), trianglev_packs.end(), result.begin());
  return result;
}

EVY_NAMESPACE_END
