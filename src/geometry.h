#ifndef __GEOMETRY_H__
#define __GEOMETRY_H__

#include <cstddef>

#include "envoy.h"
#include "envoy_common.h"
#include "math_aliases.h"
#include "zpp_bits.h"

EVY_NAMESPACE_BEGIN

/**
 * General Bounding Box
 */
template <detail_::LinalgVector T>
class alignas(32) BBox {
public:
  EVY_FORCEINLINE BBox() = default;
  EVY_FORCEINLINE BBox(const T &v) : lower(v), upper(v) {}
  EVY_FORCEINLINE BBox(const T &lower, const T &upper)
      : lower(lower), upper(upper) {}
  template <typename T_>
  EVY_FORCEINLINE BBox(const BBox<T_> &other)
      : lower(other.lower), upper(other.upper) {}

  EVY_FORCEINLINE BBox &operator=(const BBox &other) {
    lower = other.lower;
    upper = other.upper;
    return *this;
  }

  EVY_FORCEINLINE bool operator==(const BBox &other) const {
    return lower == other.lower && upper == other.lower;
  }

  EVY_FORCEINLINE bool empty() const {
    for (int i = 0; i < vec_type<T>::size; ++i)
      if (lower[i] > upper[i]) return true;
    return false;
  }

  EVY_FORCEINLINE static BBox merge(const BBox &a, const BBox &b) {
    return BBox(Min(a.lower, b.lower), Max(a.upper, b.upper));
  }

  T lower{FLOAT_INF}, upper{FLOAT_MINUS_INF};
};

using BBox3f = BBox<Vec3f>;

// TODO: Vectorized Bounding Box
class BBoxV {
public:

private:
  // lower: <[BBox_0.lower.x, ], [BBox_0.lower.y, ], [BBox_0.lower.z]>
  // upper: <[BBox_0.upper.x, ], [BBox_0.upper.y, ], [BBox_0.upper.z]>
  Vec3vf lower, upper;
  vmask  m_valid;
};

/**
 * @brief Vectorized Triangles
 */
class alignas(32) TriangleV {
public:
  friend zpp::bits::access;
  constexpr static auto serialize(auto &archive, auto &self) {
    return archive(std::span{self}.as_bytes());
  }

  using vfloat_type                 = typename vec_type<Vec3vf>::value_type;
  static constexpr std::size_t size = vfloat_type::size;

  EVY_FORCEINLINE TriangleV() = default;
  EVY_FORCEINLINE TriangleV(const vmask &valid, const Vec3vf &in_v0,
                            const Vec3vf &in_v1, const Vec3vf &in_v2)
      : m_v0(in_v0), m_v1(in_v1), m_v2(in_v2), m_valid(valid) {}

  EVY_FORCEINLINE Vec3vf v0() const { return m_v0; }
  EVY_FORCEINLINE Vec3vf v1() const { return m_v1; }
  EVY_FORCEINLINE Vec3vf v2() const { return m_v2; }
  EVY_FORCEINLINE vmask  valid() const { return m_valid; }
  EVY_FORCEINLINE BBox3f all_bound() const {
    const BBox3f bv0 = vertex_bound(m_v0);
    const BBox3f bv1 = vertex_bound(m_v1);
    const BBox3f bv2 = vertex_bound(m_v2);
    return BBox3f::merge(bv0, BBox3f::merge(bv1, bv2));
  }

private:
  EVY_FORCEINLINE BBox3f vertex_bound(const Vec3vf &v) const {
    // take v0 as an example
    const vmask mask = valid();
    Vec3f       lower, upper;
    lower.x = ReduceMin(Select(mask, v.x, vfloat{FLOAT_INF}));
    lower.y = ReduceMin(Select(mask, v.y, vfloat{FLOAT_INF}));
    lower.z = ReduceMin(Select(mask, v.z, vfloat{FLOAT_INF}));
    upper.x = ReduceMax(Select(mask, v.x, vfloat{FLOAT_MINUS_INF}));
    upper.y = ReduceMax(Select(mask, v.y, vfloat{FLOAT_MINUS_INF}));
    upper.z = ReduceMax(Select(mask, v.z, vfloat{FLOAT_MINUS_INF}));
    return {lower, upper};
  }

  Vec3vf m_v0, m_v1, m_v2;
  vmask  m_valid;
};

EVY_NAMESPACE_END

#endif
