#include <gtest/gtest.h>

#include <iostream>

#include "math_aliases.h"

using namespace envoy;

constexpr int size = vec_type<Vec3vf>::value_type::size;

TEST(math_aliases, linalg_and_xsimd) {
  constexpr float eps = 1e-4;

  Vec3vf vec;
  // just like vec{1, 1, 1}, vec{2, 2, 2} ...
  vec.x         = xs::batch<float, xs::avx2>{1, 2, 3, 4, 5, 6, 7, 8};
  vec.y         = xs::batch<float, xs::avx2>{1, 2, 3, 4, 5, 6, 7, 8};
  vec.z         = xs::batch<float, xs::avx2>{1, 2, 3, 4, 5, 6, 7, 8};
  auto  vec_new = Normalize(vec);
  float x[size], y[size], z[size];
  vec_new.x.store_unaligned(x);
  vec_new.y.store_unaligned(y);
  vec_new.z.store_unaligned(z);

  for (int i = 0; i < size; ++i) {
    EXPECT_NEAR(x[i], 1 / sqrt(3), eps);
    EXPECT_NEAR(y[i], 1 / sqrt(3), eps);
    EXPECT_NEAR(z[i], 1 / sqrt(3), eps);
  }

  Vec3f fvec{1, 2, 3};
}