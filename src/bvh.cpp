#include "bvh.h"

#include <oneapi/tbb/parallel_sort.h>

#include <iostream>
#include <span>

#include "embree3/rtcore_common.h"
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
      std::ceil(static_cast<double>(num_triangles) / trianglev_pack_size /
                TriangleVPack::triangle_type::size);

  // Alloc the result
  std::span<TriangleVPack> result = std::span{
      resource.alloc<TriangleVPack[]>(num_triangle_pack), num_triangle_pack};
  std::span<Vec3ui> indices_triangle_view{
      reinterpret_cast<Vec3ui *>(mesh.indices.data()), mesh.num_indices / 3};

  // TODO: optimize this sort
  // Sort the triangles by their morton code
  tbb::parallel_sort(
      indices_triangle_view.begin(), indices_triangle_view.end(),
      [&mesh](const Vec3ui &x, const Vec3ui &y) -> bool {
        return detail_::EncodeMorton3(mesh.vertex_positions[x.x]) <
               detail_::EncodeMorton3(mesh.vertex_positions[y.x]);
      });

  auto triangle_v = detail_::CastMeshToTriangleV(mesh, resource);
  for (std::size_t i_triangle_pack = 0, i_triangle_v = 0;
       i_triangle_pack < num_triangle_pack;
       ++i_triangle_pack, i_triangle_v += trianglev_pack_size)
    result[i_triangle_pack] = TriangleVPack{triangle_v.subspan(
        i_triangle_v,
        std::min(trianglev_pack_size, triangle_v.size() - i_triangle_v))};
  return result;
}

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
  rtcSetGeometryBuildQuality(m_geom, RTC_BUILD_QUALITY_HIGH);
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