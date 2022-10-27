#ifndef __INTERSECTOR_H__
#define __INTERSECTOR_H__

#include "envoy_common.h"
#include "math_aliases.h"

EVY_NAMESPACE_BEGIN

/**
 * @brief Intersect TriangleV with a single ray
 */
EVY_FORCEINLINE vfloat::batch_bool_type PlueckerTriangleIntersect1(
    const Vec3vf &v0, const Vec3vf &v1, const Vec3vf &v2, const Vec3f &ray_o,
    const Vec3f &ray_d, vfloat &thit, Vec3vf &ng) {
  const vfloat ulp = std::numeric_limits<float>::epsilon();
  const Vec3vf o{ray_o};  // broadcast
  const Vec3vf d{ray_d};  // broadcast
  const Vec3vf _v0 = v0 - o;
  const Vec3vf _v1 = v1 - o;
  const Vec3vf _v2 = v2 - o;
  const Vec3vf e0  = _v2 - _v0;
  const Vec3vf e1  = _v0 - _v1;
  const Vec3vf e2  = _v1 - _v2;

  const vfloat U   = Dot(Cross(e0, v2 + v0), d);
  const vfloat V   = Dot(Cross(e1, v0 + v1), d);
  const vfloat W   = Dot(Cross(e2, v1 + v2), d);
  const vfloat UVW = U + V + W;
  const vfloat eps = ulp * Abs(UVW);

  return false;
}

EVY_NAMESPACE_END

#endif
