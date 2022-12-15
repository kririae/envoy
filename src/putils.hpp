#ifndef __PUTILS_HPP__
#define __PUTILS_HPP__

#include <oneapi/tbb/parallel_for.h>
#include <oneapi/tbb/parallel_reduce.h>
#include <oneapi/tbb/parallel_sort.h>

#include "envoy.h"

EVY_NAMESPACE_BEGIN

EVY_FORCEINLINE void ParallelForLinear(
    std::size_t begin, std::size_t end,
    std::function<void(std::size_t index)> func) {
  if constexpr (true) {
    tbb::parallel_for(tbb::blocked_range<std::size_t>(begin, end),
                      [=](const tbb::blocked_range<std::size_t> &r) -> void {
                        for (std::size_t i = r.begin(); i != r.end(); ++i)
                          func(i);
                      });
  } else {
#pragma omp parallel for
    for (std::size_t i = begin; i != end; ++i) func(i);
  }
}

EVY_NAMESPACE_END

#endif
