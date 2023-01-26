#pragma once
#include "../scene/q3Body.h"
#include "q3ContactState.h"
#include <span>

struct q3ContactConstraintState {
  q3ContactState contacts[8];
  int contactCount;
  q3Vec3 tangentVectors[2]; // Tangent vectors
  q3Vec3 normal;            // From A to B
  q3BodyState A;
  q3BodyState B;
  float restitution;
  float friction;

  std::span<q3ContactState> span() { return std::span(contacts, contactCount); }
};
