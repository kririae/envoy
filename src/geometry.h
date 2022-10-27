#ifndef __GEOMETRY_H__
#define __GEOMETRY_H__

#include "common.h"
#include "math_aliases.h"

EVY_NAMESPACE_BEGIN

class BBoxV {
public:

private:
  // lower: <[BBox_0.lower.x, ], [BBox_0.lower.y, ], [BBox_0.lower.z]>
  // upper: <[BBox_0.upper.x, ], [BBox_0.upper.y, ], [BBox_0.upper.z]>
  Vec3vf lower, upper;
};
/**
 * @brief Vectorized Triangles
 */
class TriangleV {
public:
  using vfloat_type                 = typename Vec3vf::value_type;
  static constexpr std::size_t size = vfloat_type::size;

  EVY_FORCEINLINE TriangleV() = default;
  EVY_FORCEINLINE TriangleV(const Vec3vf &in_v0, const Vec3vf &in_v1,
                            const Vec3vf &in_v2)
      : v0(in_v0), v1(in_v1), v2(in_v2) {}

private:
  Vec3vf v0, v1, v2;
};

EVY_NAMESPACE_END

#endif
