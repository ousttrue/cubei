#pragma once
#include "q3Vec3.h"

struct q3RaycastData {
  q3Vec3 start; // Beginning point of the ray
  q3Vec3 dir;   // Direction of the ray (normalized)
  float t;      // Time specifying ray endpoint

  float toi;     // Solved time of impact
  q3Vec3 normal; // Surface normal at impact

  void Set(const q3Vec3 &startPoint, const q3Vec3 &direction,
           float endPointTime) {
    start = startPoint;
    dir = direction;
    t = endPointTime;
  }

  // Uses toi, start and dir to compute the point at toi. Should
  // only be called after a raycast has been conducted with a
  // return value of true.
  q3Vec3 GetImpactPoint() const { return q3Vec3(start + dir * toi); }
};
