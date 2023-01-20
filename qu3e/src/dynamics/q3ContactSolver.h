//--------------------------------------------------------------------------------------------------
/**
@file	q3ContactSolver.h

@author	Randy Gaul
@date	10/10/2014

        Copyright (c) 2014 Randy Gaul http://www.randygaul.net

        This software is provided 'as-is', without any express or implied
        warranty. In no event will the authors be held liable for any damages
        arising from the use of this software.

        Permission is granted to anyone to use this software for any purpose,
        including commercial applications, and to alter it and redistribute it
        freely, subject to the following restrictions:
          1. The origin of this software must not be misrepresented; you must
not claim that you wrote the original software. If you use this software in a
product, an acknowledgment in the product documentation would be appreciated but
is not required.
          2. Altered source versions must be plainly marked as such, and must
not be misrepresented as being the original software.
          3. This notice may not be removed or altered from any source
distribution.
*/
//--------------------------------------------------------------------------------------------------

#ifndef Q3CONTACTSOLVER_H
#define Q3CONTACTSOLVER_H

#include "../math/q3Math.h"
#include "../scene/q3Body.h"
#include <span>

//--------------------------------------------------------------------------------------------------
// q3ContactSolver
//--------------------------------------------------------------------------------------------------
class q3Island;
struct q3VelocityState;

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

struct q3ContactConstraintState {
  q3ContactState contacts[8];
  int contactCount;
  q3Vec3 tangentVectors[2]; // Tangent vectors
  q3Vec3 normal;            // From A to B
  q3BodyState A;
  q3BodyState B;
  // q3Vec3 centerA;
  // q3Vec3 centerB;
  // q3Mat3 iA;
  // q3Mat3 iB;
  // float mA;
  // float mB;
  // int indexA;
  // int indexB;
  float restitution;
  float friction;

  std::span<q3ContactState> span() {
    return std::span(contacts, contactCount);
  }
};

struct q3ContactSolver {
  void Initialize(q3Island *island);
  void ShutDown(void);

  void PreSolve(float dt);
  void Solve(void);

  q3Island *m_island;
  q3ContactConstraintState *m_contacts;
  int m_contactCount;
  q3VelocityState *m_velocities;

  bool m_enableFriction;
};

#endif // Q3CONTACTSOLVER_H
