#include <bits/chrono.h>

#include <chrono>

#include "bvh.h"
#include "envoy_common.h"
#include "geometry.h"
#include "intersector.h"
#include "lru.h"
#include "mesh.h"
#include "resource_manager.h"
#include "rng.h"
#include "stats.h"

using namespace envoy;

int main() {
  Random    rng;
  GResource resource;
  auto      mesh = MakeTriangleMesh(__get_asset_path("sphere.ply"), resource);

  auto bvh     = BaseBvh(mesh, resource);
  auto bvh_ref = EmbreeBvh(mesh, resource);

  bvh.build();
  bvh_ref.build();

  auto start = std::chrono::high_resolution_clock::now();

  for (int i = 0; i < 500000; ++i) {
    BvhRayHit rayhit1, rayhit2;
    rayhit1.ray_o = Vec3f{0.0};
    rayhit1.ray_d = Normalize(Vec3f{rng.get1D(), rng.get1D(), rng.get1D()});
    rayhit2.ray_o = rayhit1.ray_o;
    rayhit2.ray_d = rayhit1.ray_d;

    bool _1 = bvh.intersect(rayhit1);
    // bool _2 = bvh_ref.intersect(rayhit2);
  }

  auto end = std::chrono::high_resolution_clock::now();
  Info("test finished in {}",
       std::chrono::duration_cast<std::chrono::milliseconds>(end - start));

  stats::Report();
  return 0;
}
