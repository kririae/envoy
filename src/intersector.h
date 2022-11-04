#ifndef __INTERSECTOR_H__
#define __INTERSECTOR_H__

#include "envoy.h"
#include "envoy_common.h"
#include "geometry.h"
#include "math_aliases.h"
#include "stats.h"
#include "xsimd/types/xsimd_sse2_register.hpp"

EVY_NAMESPACE_BEGIN

namespace detail_ {
template <typename T>
EVY_FORCEINLINE Vec3<T> StableTriangleNormal(const Vec3<T> &a, const Vec3<T> &b,
                                             const Vec3<T> &c) {
  const T       ab_x = a.z * b.y, ab_y = a.x * b.z, ab_z = a.y * b.x;
  const T       bc_x = b.z * c.y, bc_y = b.x * c.z, bc_z = b.y * c.x;
  const Vec3<T> cross_ab(Msub(a.y, b.z, ab_x), Msub(a.z, b.x, ab_y),
                         Msub(a.x, b.y, ab_z));
  const Vec3<T> cross_bc(Msub(b.y, c.z, bc_x), Msub(b.z, c.x, bc_y),
                         Msub(b.x, c.y, bc_z));
  const auto    sx = Abs(ab_x) < Abs(bc_x);
  const auto    sy = Abs(ab_y) < Abs(bc_y);
  const auto    sz = Abs(ab_z) < Abs(bc_z);
  return {Select(sx, cross_ab.x, cross_bc.x),
          Select(sy, cross_ab.y, cross_bc.y),
          Select(sz, cross_ab.z, cross_bc.z)};
}

EVY_FORCEINLINE xs::batch<float, xs::sse2> CastVec3ToFloatBatch(
    const Vec3f &a) {
  alignas(16) std::array<float, 4> aligned_a{a.x, a.y, a.z, 1.0};
  return xs::batch<float, xs::sse2>::load_aligned(aligned_a.data());
}
}  // namespace detail_

/**
 * Intersect AABB with a ray
 */
EVY_FORCEINLINE bool BoundIntersect1(const Vec3f &lower, const Vec3f &upper,
                                     const Vec3f &ray_o, const Vec3f &ray_d,
                                     float &tnear, float &tfar) {
  float       t0 = 0, t1 = std::numeric_limits<float>::max();
  const Vec3f inv_ray_d  = Vec3f{1.0} / ray_d;
  const Vec3f t_near_vec = (lower - ray_o) * inv_ray_d;
  const Vec3f t_far_vec  = (upper - ray_o) * inv_ray_d;

  const float t_near0  = t_near_vec[0];
  const float t_near1  = t_near_vec[1];
  const float t_near2  = t_near_vec[2];
  const float t_far0   = t_far_vec[0];
  const float t_far1   = t_far_vec[1];
  const float t_far2   = t_far_vec[2];
  const float t_near_0 = std::min(t_near0, t_far0);
  const float t_near_1 = std::min(t_near1, t_far1);
  const float t_near_2 = std::min(t_near2, t_far2);
  const float t_far_0  = std::max(t_near0, t_far0);
  const float t_far_1  = std::max(t_near1, t_far1);
  const float t_far_2  = std::max(t_near2, t_far2);
  t0                   = std::max({t0, t_near_0, t_near_1, t_near_2});
  t1                   = std::min({t1, t_far_0, t_far_1, t_far_2});

  tnear = t0, tfar = t1;
  return t0 < t1;
}

EVY_FORCEINLINE bool BoundIntersect1(const BBox3f &bound, const Vec3f &ray_o,
                                     const Vec3f &ray_d, float &tnear,
                                     float &tfar) {
  return BoundIntersect1(bound.lower, bound.upper, ray_o, ray_d, tnear, tfar);
}

/**
 * @brief Intersect TriangleV with a single ray
 * @see See Intel Embree's implementation
 */
EVY_FORCEINLINE vmask PlueckerTriangleIntersect1(
    const vmask &valid_, const Vec3vf &v0, const Vec3vf &v1, const Vec3vf &v2,
    const Vec3f &ray_o, const Vec3f &ray_d, vfloat &thit, Vec3vf &ng) {
  stats::IncTriangleIntersectionTimes(vec_type<vfloat>::size);

  vmask        valid = valid_;
  const vfloat ulp   = std::numeric_limits<float>::epsilon();
  const vfloat zero{0.0};
  const Vec3vf o{ray_o};  // broadcast
  const Vec3vf d{ray_d};  // broadcast
  const Vec3vf _v0 = v0 - o;
  const Vec3vf _v1 = v1 - o;
  const Vec3vf _v2 = v2 - o;
  const Vec3vf e0  = _v2 - _v0;
  const Vec3vf e1  = _v0 - _v1;
  const Vec3vf e2  = _v1 - _v2;

  const vfloat u   = Dot(Cross(e0, v2 + v0), d);
  const vfloat v   = Dot(Cross(e1, v0 + v1), d);
  const vfloat w   = Dot(Cross(e2, v1 + v2), d);
  const vfloat uvw = u + v + w;
  const vfloat eps = ulp * Abs(uvw);

  valid = valid & (Min(u, v, w) >= -eps | Max(u, v, w) <= eps);

  // All triangles cannot intersect
  if (None(valid)) return false;

  // Calculate geometry normal (not yet normalized)
  const Vec3vf Ng  = Normalize(detail_::StableTriangleNormal(e0, e1, e2));
  const vfloat den = Twice(Dot(Ng, d));

  // Depth test
  const vfloat T = Twice(Dot(v0, Ng));
  const vfloat t = T * Rcp(den);

  valid = valid & (zero <= t);
  if (None(valid)) return false;
  valid = valid & (zero != den);
  if (None(valid)) return false;

  // Out
  ng   = Ng;
  thit = t;

  return valid;
}

/**
 * @brief Triangle Intersect Wrapper
 * @see other PlueckerTriangleIntersect1
 */
EVY_FORCEINLINE vmask PlueckerTriangleIntersect1(const TriangleV &vtriangle,
                                                 const Vec3f     &ray_o,
                                                 const Vec3f     &ray_d,
                                                 vfloat &thit, Vec3vf &ng) {
  return PlueckerTriangleIntersect1(vtriangle.valid(), vtriangle.v0(),
                                    vtriangle.v1(), vtriangle.v2(), ray_o,
                                    ray_d, thit, ng);
}

/**
 * @brief PlueckerTriangleIntersect1() accepts a vectorized triangle and a
 * series of related parameters, then output the final intersection result
 * reduced to float and Vec3f into `thit` and `ng`
 * @return bool If there's really an intersection
 */
EVY_FORCEINLINE bool PlueckerTriangleIntersect1(const TriangleV &vtriangle,
                                                const Vec3f     &ray_o,
                                                const Vec3f &ray_d, float &thit,
                                                Vec3f &ng) {
  vfloat      vthit;
  Vec3vf      vng;
  const vmask vresult = PlueckerTriangleIntersect1(
      vtriangle.valid(), vtriangle.v0(), vtriangle.v1(), vtriangle.v2(), ray_o,
      ray_d, vthit, vng);
  if (!Any(vresult)) return false;

  const vfloat valid_thits = Select(vresult, vthit, vfloat{FLOAT_INF});
  thit                     = ReduceMin(valid_thits); /* out */

  // TODO: optimize the filter
  ng.x = Filter(vresult, vng.x);  // yet reduce
  ng.y = Filter(vresult, vng.y);
  ng.z = Filter(vresult, vng.z);
  return true;
}

EVY_NAMESPACE_END

#endif
