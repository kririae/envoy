#ifndef __MATH_ALIASES_H__
#define __MATH_ALIASES_H__

#include <concepts>
#include <glm/detail/qualifier.hpp>
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
// clang-format on
}  // namespace detail_

/**
 * Aliases for glm and xsimd
 */
using Vec2f = glm::vec2;
using Vec3f = glm::vec3;
using Vec4f = glm::vec4;

using Vec2i = glm::ivec2;
using Vec3i = glm::ivec3;
using Vec4i = glm::ivec4;

using Mat4f = glm::mat4;

using vfloat  = xsimd::batch<float>;
using vdouble = xsimd::batch<double>;
using vbool   = xsimd::batch<bool>;
using vint    = xsimd::batch<int>;

using Vec2vf = glm::vec<2, vfloat>;
using Vec3vf = glm::vec<3, vfloat>;
using Vec4vf = glm::vec<4, vfloat>;

using Vec2vi = glm::vec<2, vint>;
using Vec3vi = glm::vec<3, vint>;
using Vec4vi = glm::vec<4, vint>;

/// a subset of glm's implementation, accepting only GlmVector

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
  return glm::normalize(x);
}

EVY_NAMESPACE_END

#endif
