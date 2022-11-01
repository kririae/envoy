#ifndef __RNG_H__
#define __RNG_H__

#include <random>

#include "envoy_common.h"

EVY_NAMESPACE_BEGIN

class Random {
public:
  Random(uint32_t seed = 0) : m_seed(seed), m_generator(m_seed) {}
  Vec2f get2D() { return {get1D(), get1D()}; }
  float get1D() {
    static std::uniform_real_distribution<float> distribution(0, 1);
    return distribution(m_generator);
  }

private:
  uint32_t     m_seed;
  std::mt19937 m_generator;
};

EVY_NAMESPACE_END

#endif
