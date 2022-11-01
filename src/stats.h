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
static auto &get_accumulator_instance() {
  static boost::accumulators::accumulator_set<T, Ts...> accumulator;
  return accumulator;
}

// intersector.h
class EnvoyTriangleIntersectTimes {
public:
  using features = boost::accumulators::features<boost::accumulators::tag::sum>;
};

void     IncTriangleIntersectionTimes(std::size_t n = 1);
uint64_t GetTriangleIntersectionTimes();

// resource_manager.h
class EnvoyResourceAllocateInfo {
public:
  using features = boost::accumulators::features<boost::accumulators::tag::max,
                                                 boost::accumulators::tag::mean,
                                                 boost::accumulators::tag::sum>;
};

void           RecResourceAllocateInfo(std::size_t n);
decltype(auto) GetResourceAllocateInfo();

void Report();

}  // namespace stats
EVY_NAMESPACE_END

#endif
