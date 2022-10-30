#include <gtest/gtest.h>
#include <unistd.h>

#include <iostream>

#include "geometry.h"
#include "pages.h"

using namespace envoy;

TEST(pages, size) {
  TrianglePage page;
  EXPECT_EQ(sizeof(page), getpagesize());
}

TEST(pages, serialize) {
  TrianglePage page(1), page_empty;
  auto [data, in, out] = zpp::bits::data_in_out();
  EXPECT_EQ(&page, page.addr);
  EXPECT_EQ(page.rm_page_id, 1);
  EXPECT_EQ((reinterpret_cast<uint64_t>(page.addr) % getpagesize()), 0);
  out(page).or_throw();

  in(page_empty).or_throw();
  EXPECT_EQ(page_empty.rm_page_id, 1);
  EXPECT_NE(page.addr, page_empty.addr);
  EXPECT_EQ((reinterpret_cast<uint64_t>(page_empty.addr) % getpagesize()), 0);
}
