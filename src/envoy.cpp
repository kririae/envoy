#include <args.hxx>

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

int main(int argc, char *argv[]) {
  // command line variables parsing
  args::ArgumentParser parser(
      "This is the performance test interface for envoy", "krr.");
  args::HelpFlag help(parser, "help", "Display help menu", {'h', "help"});
  args::ValueFlag<std::string> asset_name_arg(
      parser, "string", "Specify the asset name to load", {'a', "asset-name"});
  args::ValueFlag<int> num_iter_arg(
      parser, "integer", "Specify the numer of iterations", {'n', "num-iter"});
  try {
    parser.ParseCLI(argc, argv);
  } catch (args::Help) {
    std::cout << parser;
    return 0;
  } catch (args::ParseError e) {
    std::cerr << e.what() << std::endl;
    std::cerr << parser;
    return 1;
  } catch (args::ValidationError e) {
    std::cerr << e.what() << std::endl;
    std::cerr << parser;
    return 1;
  }

  std::string asset_name = "dambreak0.ply";
  int         num_iter   = 500000;
  if (asset_name_arg) asset_name = args::get(asset_name_arg);
  if (num_iter_arg) num_iter = args::get(num_iter_arg);

  Random    rng;
  GResource resource;
  auto      mesh = MakeTriangleMesh(__get_asset_path(asset_name), resource);

  auto bvh     = RadixBvh(mesh, resource);
  auto bvh_ref = EmbreeBvh(mesh, resource);

  bvh.build_info();
  bvh_ref.build_info();

  auto start = std::chrono::high_resolution_clock::now();

  for (int i = 0; i < num_iter; ++i) {
    BvhRayHit rayhit1, rayhit2;
    rayhit1.ray_o = Vec3f{0.0};
    rayhit1.ray_d = Normalize(Vec3f{rng.get1D(), rng.get1D(), rng.get1D()});
    rayhit2.ray_o = rayhit1.ray_o;
    rayhit2.ray_d = rayhit1.ray_d;

    bool _1 = bvh.intersect(rayhit1);
    // bool _2 = bvh_ref.intersect(rayhit2);
    // if (_1 != _2) Info("Failed: {}", i);
  }

  auto end = std::chrono::high_resolution_clock::now();
  Info("test finished in {}",
       std::chrono::duration_cast<std::chrono::milliseconds>(end - start));

  stats::Report();
  return 0;
}
