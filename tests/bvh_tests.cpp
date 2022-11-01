#include <gtest/gtest.h>

#include "bvh.h"
#include "envoy_common.h"
#include "geometry.h"
#include "mesh.h"
#include "resource_manager.h"
#include "rng.h"
#include "stats.h"

using namespace envoy;

TEST(bvh, geometry) {
  Random    rng;
  GResource resource;
  auto      mesh =
      MakeTriangleMesh(__get_asset_path("bun_zipper_res4.ply"), resource);

  auto bvh     = SerialBvh(mesh, resource);
  auto bvh_ref = EmbreeBvh(mesh, resource);
  bvh.build();
  bvh_ref.build();

  for (int i = 0; i < 10000; ++i) {
    BvhRayHit rayhit1, rayhit2;
    rayhit1.ray_o = Vec3f{0.0};
    rayhit1.ray_d = Normalize(Vec3f{rng.get1D(), rng.get1D(), rng.get1D()});
    rayhit2.ray_o = rayhit1.ray_o;
    rayhit2.ray_d = rayhit1.ray_d;

    bool _1 = bvh.intersect(rayhit1);
    bool _2 = bvh.intersect(rayhit2);
    EXPECT_EQ(_1, _2);
    EXPECT_NEAR(rayhit1.tfar, rayhit2.tfar, 1e-5);
    EXPECT_NEAR((rayhit1.hit_ng.x), (rayhit2.hit_ng.x), 1e-3);
  }

  stats::Report();
}
