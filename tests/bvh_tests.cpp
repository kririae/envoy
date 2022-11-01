#include <gtest/gtest.h>

#include "bvh.h"
#include "envoy_common.h"
#include "geometry.h"
#include "mesh.h"
#include "resource_manager.h"
#include "stats.h"

using namespace envoy;

TEST(bvh, geometry) {
  GResource resource;
  auto      mesh = MakeTriangleMesh(__get_asset_path("sphere.ply"), resource);
  auto      triangles = detail_::CastMeshToTriangleV(mesh, resource);

  auto bvh = SerialBvh(triangles, resource);
  bvh.build();
  BvhRayHit rayhit;
  rayhit.ray_o = Vec3f{0.0};
  rayhit.ray_d = Vec3f{1.0, 0.0, 0.0};

  bool _ = bvh.intersect(rayhit);
  EXPECT_TRUE(_);
  EXPECT_NEAR(rayhit.tfar, 0.1, 1e-5);
  EXPECT_NEAR((rayhit.hit_ng.x), 1.0, 1e-3);

  stats::Report();
}
