#pragma once
#include "q3Vec3.h"

struct q3HalfSpace {
  q3HalfSpace();
  q3HalfSpace(const q3Vec3 &normal, float distance);

  void Set(const q3Vec3 &a, const q3Vec3 &b, const q3Vec3 &c);
  void Set(const q3Vec3 &n, const q3Vec3 &p);
  const q3Vec3 Origin() const;
  float Distance(const q3Vec3 &p) const;
  const q3Vec3 Projected(const q3Vec3 &p) const;

  q3Vec3 normal;
  float distance;
};
