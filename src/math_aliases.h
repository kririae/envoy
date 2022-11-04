#ifndef __MATH_ALIASES_H__
#define __MATH_ALIASES_H__

#include <linalg.h>

#include <concepts>
#include <glm/glm.hpp>
#include <limits>
#include <type_traits>
#include <xsimd/types/xsimd_sse2_register.hpp>

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

template<typename T>
concept XsimdBatchBool = requires(T t) {
  { xs::batch_bool{t} } -> std::same_as<T>;
};

template<typename T>
concept Bool = XsimdBatchBool<T> || std::is_same_v<T, bool>;

template<typename T>
concept Vector = XsimdVector<T> || LinalgVector<T>;
// clang-format on
}  // namespace detail_

template <typename T>
struct vec_type;

template <typename T, int M>
struct vec_type<linalg::vec<T, M>> {
  using value_type                  = T;
  static constexpr std::size_t size = M;
};

template <typename T, typename A>
struct vec_type<xs::batch<T, A>> {
  using value_type                  = T;
  static constexpr std::size_t size = xs::batch<T, A>::size;
};

template <typename T>
struct batch_type;

template <typename T, typename A>
struct batch_type<xs::batch<T, A>> {
  using arch_type  = A;
  using value_type = T;

  static constexpr std::size_t size = xs::batch<T, A>::size;
};

/// a wrapper for XSIMD's operators presented in
/// https://xsimd.readthedocs.io/en/latest/api/reducer_index.html
/// to make the code not that intrusive, we'll not specify the width as an
/// template
using default_arch = xsimd::avx;

template <typename T>
using vtype = xs::batch<T>;  // vectorized type

using vfloat  = xs::batch<float, default_arch>;
using vint    = xs::batch<int, default_arch>;
using vdouble = xs::batch<double, default_arch>;
using vbool   = xs::batch<bool, default_arch>;
using vmask   = vfloat::batch_bool_type;

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

// Non-linear
template <detail_::XsimdVector T>
EVY_FORCEINLINE decltype(auto) Abs(const T &x) {
  return xs::abs(x);
}

// Non-linear
template <detail_::XsimdVector T>
EVY_FORCEINLINE decltype(auto) Sqrt(const T &x) {
  if constexpr (std::is_same_v<typename batch_type<T>::arch_type,
                               xs::generic>) {
    return 0;
  } else {
    return xs::sqrt(x);
  }
}

template <detail_::XsimdVector T>
EVY_FORCEINLINE decltype(auto) Rcp(const T &x) {
  return 1 / x;
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
EVY_FORCEINLINE decltype(auto) Min(const T &x, const T &y, const T &z) {
  if constexpr (std::is_same_v<typename T::value_type, float>) {
    return xs::fmin(xs::fmin(x, y), z);
  } else {
    return xs::min(xs::min(x, y), z);
  }
}

template <detail_::XsimdVector T>
EVY_FORCEINLINE decltype(auto) Max(const T &x, const T &y, const T &z) {
  if constexpr (std::is_same_v<typename T::value_type, float>) {
    return xs::fmax(xs::fmax(x, y), z);
  } else {
    return xs::max(xs::max(x, y), z);
  }
}

template <detail_::XsimdVector TVec, detail_::Bool TBool>
EVY_FORCEINLINE decltype(auto) Select(const TBool &cond, const TVec &a,
                                      const TVec &b) {
  return xs::select(cond, a, b);
}

/**
 * @brief Filter a[i] with (valid[i]==1)
 *
 * @tparam T
 * @param valid
 * @param a
 * @return EVY_FORCEINLINE
 */
template <detail_::XsimdVector T>
EVY_FORCEINLINE auto Filter(const vmask &valid, const T &a) {
  // TODO: For now, a naive implementation
  using value_type          = typename vec_type<T>::value_type;
  static constexpr int size = vec_type<T>::size;

  T b{1};                      // broadcast
  b = Select(valid, b, T{0});  // extract the mask
  value_type mask[size], arr[size];
  b.store_unaligned(mask);
  a.store_unaligned(arr);
  for (int i = 0; i < size; ++i)
    if (mask[i] == 1) return arr[i];
  return value_type{};
}

// All is true
template <detail_::XsimdBatchBool T>
EVY_FORCEINLINE decltype(auto) All(const T &x) {
  return xs::all(x);
}

// Any is true
template <detail_::XsimdBatchBool T>
EVY_FORCEINLINE decltype(auto) Any(const T &x) {
  return xs::any(x);
}

// None is true
template <detail_::XsimdBatchBool T>
EVY_FORCEINLINE decltype(auto) None(const T &x) {
  return xs::none(x);
}

// @note from
// https://github.com/facebookincubator/velox/blob/main/velox/common/base/SimdUtil-inl.h
template <typename T, typename A>
xsimd::batch_bool<T, A> LeadingMask(int n, const A &) {
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

/// General Operations
template <typename T>
EVY_FORCEINLINE decltype(auto) Msub(const T &a, const T &b, const T &c) {
  return a * b - c;
}

template <typename T>
EVY_FORCEINLINE decltype(auto) Twice(const T &a) {
  return a + a;
}

// Non-linear
template <typename T>
EVY_FORCEINLINE decltype(auto) Abs(const T &a) {
  return abs(a);
}

// Non-linear
template <typename T>
EVY_FORCEINLINE decltype(auto) Sqrt(const T &a) {
  return sqrt(a);
}

/**
 * Aliases for linalg and xsimd
 */
template <typename T>
using Vec3 = linalg::vec<T, 3>;

using Vec2f = linalg::vec<float, 2>;
using Vec3f = linalg::vec<float, 3>;
using Vec4f = linalg::vec<float, 4>;

using Vec2i = linalg::vec<int, 2>;
using Vec3i = linalg::vec<int, 3>;
using Vec4i = linalg::vec<int, 4>;

using Vec2ui = linalg::vec<uint32_t, 2>;
using Vec3ui = linalg::vec<uint32_t, 3>;
using Vec4ui = linalg::vec<uint32_t, 4>;

using Mat4f = linalg::mat<float, 4, 4>;

using Vec2vf = linalg::vec<vfloat, 2>;
using Vec3vf = linalg::vec<vfloat, 3>;
using Vec4vf = linalg::vec<vfloat, 4>;

using Vec2vi = linalg::vec<vint, 2>;
using Vec3vi = linalg::vec<vint, 3>;
using Vec4vi = linalg::vec<vint, 4>;

using namespace linalg::ostream_overloads;

constexpr float FLOAT_INF       = std::numeric_limits<float>::infinity();
constexpr float FLOAT_MINUS_INF = -std::numeric_limits<float>::infinity();

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
  return glm::normalize(x);
}

template <detail_::LinalgVector T>
EVY_FORCEINLINE decltype(auto) Length(const T &x) {
  // TODO: general interpretation
  constexpr int vec_size = vec_type<T>::size;
  if constexpr (vec_size == 3) {
    return Sqrt(x.x * x.x + x.y * x.y + x.z * x.z);
  } else {
    auto result = x.x * x.x;
    for (int i = 1; i < vec_size; ++i) result = result + x[i] * x[i];
    return result;
  }
}

template <detail_::LinalgVector T>
EVY_FORCEINLINE decltype(auto) Distance(const T &x, const T &y) {
  return Length(x - y);
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
  return x / T{Length(x)}; /* LinalgVector cannot implicitly broadcast */
}

template <detail_::LinalgVector T>
EVY_FORCEINLINE decltype(auto) Max(const T &x, const T &y) {
  return linalg::max(x, y);
}

template <detail_::LinalgVector T>
EVY_FORCEINLINE decltype(auto) Min(const T &x, const T &y) {
  return linalg::min(x, y);
}

template <typename T>
EVY_FORCEINLINE decltype(auto) GetGB(const T &x) {
  return static_cast<double>(x) / 1024 / 1024 / 1024;
}

template <typename T>
EVY_FORCEINLINE decltype(auto) GetMB(const T &x) {
  return static_cast<double>(x) / 1024 / 1024;
}

template <typename T>
EVY_FORCEINLINE decltype(auto) GetKB(const T &x) {
  return static_cast<double>(x) / 1024;
}

EVY_NAMESPACE_END

#endif
