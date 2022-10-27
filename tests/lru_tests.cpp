#include <gtest/gtest.h>

#include "lru.h"

using namespace envoy;

static int &get_counter_instance() {
  static int counter = 0;
  return counter;
}

struct Destruct {
  Destruct() = default;
  ~Destruct() { ++get_counter_instance(); }
};

TEST(LRU, item_destruct) {
  LRU<int, Destruct> cache(2);
  Destruct           d1, d2, d3, d4;
  cache.insert(1, d1);
  cache.insert(2, d2);
  cache.insert(3, d3);
  // one cache item is evicted, so it is destructed
  EXPECT_EQ(get_counter_instance(), 1);

  cache.insert(4, d3);
  EXPECT_EQ(get_counter_instance(), 2);
}

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
