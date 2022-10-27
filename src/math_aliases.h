#ifndef __MATH_ALIASES_H__
#define __MATH_ALIASES_H__

#include <concepts>
#include <glm/glm.hpp>
#include <type_traits>

#include "envoy.h"

EVY_NAMESPACE_BEGIN

namespace detail_ {
// clang-format off
template <typename T>
concept GlmVector = requires(T t) {
  { glm::vec{t} } -> std::same_as<T>;
};
// clang-format on
}  // namespace detail_

using Vector2f = glm::vec2;
using Vector3f = glm::vec3;
using Vector4f = glm::vec4;

using Vector2i = glm::ivec2;
using Vector3i = glm::ivec3;
using Vector4i = glm::ivec4;

using Matrix4f = glm::mat4;

template <detail_::GlmVector T>
EVY_FORCEINLINE auto Dot(const T &x, const T &y) -> T {
  return glm::dot(x, y);
}

EVY_NAMESPACE_END

#endif
