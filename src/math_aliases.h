#ifndef __MATH_ALIASES_H__
#define __MATH_ALIASES_H__

#include <linalg.h>

#include <concepts>
#include <glm/glm.hpp>
#include <type_traits>

#include "envoy.h"
#include "xsimd/xsimd.hpp"

namespace xs = xsimd;

EVY_NAMESPACE_BEGIN

namespace detail_ {
// clang-format off
template <typename T>
concept GlmVector = requires(T t) {
	{ glm::vec{t} } -> std::same_as<T>;
};

template<typename T>
concept LinalgVector = requires(T t) {
	{ linalg::vec{t} } -> std::same_as<T>;
};

template<typename T>
concept XsimdVector = requires(T t) {
	{ xs::batch{t} } -> std::same_as<T>;
};
// clang-format on
}  // namespace detail_

template <typename T>
struct vec_type;

template <typename T, int M>
struct vec_type<linalg::vec<T, M>> {
  using value_type                  = T;
  static constexpr std::size_t size = M;
};

/**
 * Aliases for linalg and xsimd
 */
using Vec2f = linalg::vec<float, 2>;
using Vec3f = linalg::vec<float, 3>;
using Vec4f = linalg::vec<float, 4>;

using Vec2i = linalg::vec<int, 2>;
using Vec3i = linalg::vec<int, 3>;
using Vec4i = linalg::vec<int, 4>;

using Mat4f = linalg::mat<float, 4, 4>;

using vfloat  = xs::batch<float>;
using vint    = xs::batch<int>;
using vdouble = xs::batch<double>;
using vbool   = xs::batch<bool>;

using Vec2vf = linalg::vec<vfloat, 2>;
using Vec3vf = linalg::vec<vfloat, 3>;
using Vec4vf = linalg::vec<vfloat, 4>;

using Vec2vi = linalg::vec<vint, 2>;
using Vec3vi = linalg::vec<vint, 3>;
using Vec4vi = linalg::vec<vint, 4>;

/// a subset of glm's implementation, accepting only GlmVector
/// the following should support SoA vectors
template <detail_::GlmVector T>
EVY_FORCEINLINE decltype(auto) Length(const T &x) {
  return glm::length(x);
}

template <detail_::GlmVector T>
EVY_FORCEINLINE decltype(auto) Distance(const T &x, const T &y) {
  return glm::distance(x, y);
}

template <detail_::GlmVector T>
EVY_FORCEINLINE decltype(auto) Dot(const T &x, const T &y) {
  return glm::dot(x, y);
}

template <detail_::GlmVector T>
EVY_FORCEINLINE decltype(auto) Cross(const T &x, const T &y) {
  return glm::cross(x, y);
}

template <detail_::GlmVector T>
EVY_FORCEINLINE decltype(auto) Normalize(const T &x) {
  return linalg::normalize(x);
}

template <detail_::LinalgVector T>
EVY_FORCEINLINE decltype(auto) Length(const T &x) {
  return linalg::length(x);
}

template <detail_::LinalgVector T>
EVY_FORCEINLINE decltype(auto) Distance(const T &x, const T &y) {
  return linalg::distance(x, y);
}

template <detail_::LinalgVector T>
EVY_FORCEINLINE decltype(auto) Dot(const T &x, const T &y) {
  return linalg::dot(x, y);
}

template <detail_::LinalgVector T>
EVY_FORCEINLINE decltype(auto) Cross(const T &x, const T &y) {
  return linalg::cross(x, y);
}

template <detail_::LinalgVector T>
EVY_FORCEINLINE decltype(auto) Normalize(const T &x) {
  return linalg::normalize(x);
}

/// a wrapper for XSIMD's operators presented in
/// https://xsimd.readthedocs.io/en/latest/api/reducer_index.html
template <detail_::XsimdVector T>
EVY_FORCEINLINE decltype(auto) ReduceAdd(const T &x) {
  return xs::reduce_add(x);
}

template <detail_::XsimdVector T>
EVY_FORCEINLINE decltype(auto) ReduceMax(const T &x) {
  return xs::reduce_max(x);
}

template <detail_::XsimdVector T>
EVY_FORCEINLINE decltype(auto) ReduceMin(const T &x) {
  return xs::reduce_min(x);
}

template <detail_::XsimdVector T>
EVY_FORCEINLINE decltype(auto) Abs(const T &x) {
  return xs::abs(x);
}

template <detail_::XsimdVector T>
EVY_FORCEINLINE decltype(auto) Min(const T &x, const T &y) {
  if constexpr (std::is_same_v<typename T::value_type, float>) {
    return xs::fmin(x, y);
  } else {
    return xs::min(x, y);
  }
}

template <detail_::XsimdVector T>
EVY_FORCEINLINE decltype(auto) Max(const T &x, const T &y) {
  if constexpr (std::is_same_v<typename T::value_type, float>) {
    return xs::fmax(x, y);
  } else {
    return xs::max(x, y);
  }
}

EVY_NAMESPACE_END

#endif
