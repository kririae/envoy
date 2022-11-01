#include "stats.h"

EVY_NAMESPACE_BEGIN
namespace stats {

void IncTriangleIntersectionTimes(std::size_t n) {
  auto &accumulator =
      get_accumulator_instance<uint64_t, EnvoyTriangleIntersectTimes,
                               EnvoyTriangleIntersectTimes::features>();
  accumulator(n);
}

uint64_t GetTriangleIntersectionTimes() {
  auto &accumulator =
      get_accumulator_instance<uint64_t, EnvoyTriangleIntersectTimes,
                               EnvoyTriangleIntersectTimes::features>();
  return boost::accumulators::sum(accumulator);
}

void RecResourceAllocateInfo(std::size_t n) {
  auto &accumulator =
      get_accumulator_instance<uint64_t, EnvoyResourceAllocateInfo,
                               EnvoyResourceAllocateInfo::features>();
  accumulator(n);
}

decltype(auto) GetResourceAllocateInfo() {
  auto &accumulator =
      get_accumulator_instance<uint64_t, EnvoyResourceAllocateInfo,
                               EnvoyResourceAllocateInfo::features>();
  return std::make_tuple(boost::accumulators::sum(accumulator),
                         boost::accumulators::mean(accumulator),
                         boost::accumulators::max(accumulator));
}

void Report() {
  Info("SIMD width:         {}", vec_type<vfloat>::size);
  Info("Intersection times: {}", GetTriangleIntersectionTimes());
  Info("Resource Allocate Info:");
  auto resource_allocate_info = GetResourceAllocateInfo();
  Info("  bytes_sum:  {:<.4f} MB", GetMB(get<0>(resource_allocate_info)));
  Info("  bytes_mean: {:<.4f} KB", GetKB(get<1>(resource_allocate_info)));
  Info("  bytes_max:  {:<.4f} MB", GetMB(get<2>(resource_allocate_info)));
}

}  // namespace stats
EVY_NAMESPACE_END