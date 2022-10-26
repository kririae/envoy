#include <fmt/format.h>
#include "gtest/gtest.h"
#include "lru.h"

using namespace envoy;

TEST(LRU, all) {
  LRU<int, float> cache(2);
  cache.insert(1, 1.0);
  cache.insert(2, 2.0);

  EXPECT_TRUE(!cache.empty());
  EXPECT_EQ(cache.capacity(), 2);
  EXPECT_EQ(cache.size(), 2);
  EXPECT_EQ(cache.get(1), 1.0);
  EXPECT_EQ(cache.get(2), 2.0);
  EXPECT_EQ(cache.get(3), std::nullopt);

  cache.insert(3, 3.0);
  EXPECT_EQ(cache.get(1), std::nullopt);
  EXPECT_EQ(cache.get(2), 2.0);
  EXPECT_EQ(cache.get(3), 3.0);
}
