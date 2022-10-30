#include "bvh.h"

#include <iostream>

#include "envoy_common.h"

EVY_NAMESPACE_BEGIN

void SerialBvh::build() {
  m_bound = triangles[0].all_bound();
  for (auto &triangle : triangles) {
    m_bound = BBox3f::merge(m_bound, triangle.all_bound());
  }
}

bool SerialBvh::intersect(BvhRayHit &rayhit) {
  bool  hit  = false;
  float tfar = rayhit.tfar;
  for (const auto &triangle : triangles) {
    float thit;
    Vec3f ng;
    bool  inter = PlueckerTriangleIntersect1(triangle, rayhit.ray_o,
                                             rayhit.ray_d, thit, ng);
    if (!inter) continue;
    if (thit > tfar) continue;
    hit           = true;
    tfar          = thit;
    rayhit.hit    = hit;
    rayhit.tfar   = tfar;
    rayhit.hit_ng = rayhit.hit_ns = ng;
  }

  return hit;
}

EVY_NAMESPACE_END