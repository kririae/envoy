#ifndef __STATISTICS_H__
#define __STATISTICS_H__

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/framework/accumulator_set.hpp>
#include <boost/accumulators/framework/features.hpp>
#include <boost/accumulators/statistics.hpp>

#include "envoy_common.h"

EVY_NAMESPACE_BEGIN
namespace stats {

template <typename T, typename S, typename... Ts>
inline auto &get_accumulator_instance() {
  static boost::accumulators::accumulator_set<T, Ts...> accumulator;
  return accumulator;
}

class EnvoyTriangleIntersectTimes {
public:
  using features = boost::accumulators::features<boost::accumulators::tag::sum>;
};

EVY_FORCEINLINE void IncTriangleIntersectionTimes(std::size_t n = 1) {
  auto &accumulator =
      get_accumulator_instance<uint64_t, EnvoyTriangleIntersectTimes,
                               EnvoyTriangleIntersectTimes::features>();
  accumulator(n);
}

EVY_FORCEINLINE uint64_t GetTriangleIntersectionTimes() {
  auto &accumulator =
      get_accumulator_instance<uint64_t, EnvoyTriangleIntersectTimes,
                               EnvoyTriangleIntersectTimes::features>();
  return boost::accumulators::sum(accumulator);
}

inline void Report() {
  Info("SIMD width {}", vec_type<vfloat>::size);
  Info("Intersection times {}", GetTriangleIntersectionTimes());
}

}  // namespace stats
EVY_NAMESPACE_END

#endif
