#pragma once
#include "../math/q3Vec3.h"

struct q3ContactState {
  q3Vec3 ra;               // Vector from C.O.M to contact position
  q3Vec3 rb;               // Vector from C.O.M to contact position
  float penetration;       // Depth of penetration from collision
  float normalImpulse;     // Accumulated normal impulse
  float tangentImpulse[2]; // Accumulated friction impulse
  float bias;              // Restitution + baumgarte
  float normalMass;        // Normal constraint mass
  float tangentMass[2];    // Tangent constraint mass
};
