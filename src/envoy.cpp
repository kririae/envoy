#include "envoy.h"

#include "lru.h"
#include "mesh.h"
#include "resource_manager.h"

using namespace envoy;

int main() {
  GResource resource;
  Info("resource_manager is created successfully");
  auto _ = MakeTriangleMesh(__get_asset_path("bun_zipper_res4.ply"), resource);
  return 0;
}
