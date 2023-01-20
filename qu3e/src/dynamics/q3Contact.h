//--------------------------------------------------------------------------------------------------
/**
@file	q3Contact.h

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

#ifndef Q3CONTACT_H
#define Q3CONTACT_H

#include "../math/q3Math.h"
#include "../scene/q3Box.h"
#include "q3ContactEdge.h"

//--------------------------------------------------------------------------------------------------
// q3Contact
//--------------------------------------------------------------------------------------------------
class q3Body;
struct q3Box;
struct q3ContactConstraint;

// Restitution mixing. The idea is to use the maximum bounciness, so bouncy
// objects will never not bounce during collisions.
inline float q3MixRestitution(const q3Box *A, const q3Box *B) {
  return q3Max(A->restitution, B->restitution);
}

// Friction mixing. The idea is to allow a very low friction value to
// drive down the mixing result. Example: anything slides on ice.
inline float q3MixFriction(const q3Box *A, const q3Box *B) {
  return std::sqrt(A->friction * B->friction);
}

// in stands for "incoming"
// out stands for "outgoing"
// I stands for "incident"
// R stands for "reference"
// See D. Gregorius GDC 2015 on creating contacts for more details
// Each feature pair is used to cache solutions from one physics tick to
// another. This is called warmstarting, and lets boxes stack and stay stable.
// Feature pairs identify points of contact over multiple physics ticks. Each
// feature pair is the junction of an incoming feature and an outgoing feature,
// usually a result of clipping routines. The exact info stored in the feature
// pair can be arbitrary as long as the result is a unique ID for a given
// intersecting configuration.
union q3FeaturePair {
  struct {
    uint8_t inR;
    uint8_t outR;
    uint8_t inI;
    uint8_t outI;
  };

  int key;
};

struct q3Contact {
  q3Vec3 position;         // World coordinate of contact
  float penetration;       // Depth of penetration from collision
  float normalImpulse;     // Accumulated normal impulse
  float tangentImpulse[2]; // Accumulated friction impulse
  float bias;              // Restitution + baumgarte
  float normalMass;        // Normal constraint mass
  float tangentMass[2];    // Tangent constraint mass
  q3FeaturePair fp;        // Features on A and B for this contact
  uint8_t warmStarted;     // Used for debug rendering
};

struct q3Manifold {
  void SetPair(q3Box *a, q3Box *b);

  q3Box *A;
  q3Box *B;

  q3Vec3 normal;            // From A to B
  q3Vec3 tangentVectors[2]; // Tangent vectors
  q3Contact contacts[8];
  int contactCount;

  q3Manifold *next;
  q3Manifold *prev;

  bool sensor;
};

enum class q3ContactConstraintFlags {
  eNone = 0,
  eColliding = 0x00000001,    // Set when contact collides during a step
  eWasColliding = 0x00000002, // Set when two objects stop colliding
  eIsland = 0x00000004,       // For internal marking during island forming
};

struct q3ContactConstraint {
  void SolveCollision(void);

  q3Box *A;
  q3Box *B;
  q3Body *bodyA;
  q3Body *bodyB;
  q3ContactEdge edgeA;
  q3ContactEdge edgeB;

  float friction;
  float restitution;

  q3Manifold manifold;

  q3ContactConstraintFlags m_flags = {};

  bool HasFlag(q3ContactConstraintFlags flag) const {
    return ((int)m_flags & (int)flag) != 0;
  }
  void AddFlag(q3ContactConstraintFlags flag) {
    m_flags = (q3ContactConstraintFlags)((int)m_flags | (int)flag);
  }
  void RemoveFlag(q3ContactConstraintFlags flag) {
    m_flags = (q3ContactConstraintFlags)((int)m_flags & ~(int)flag);
  }
};

void q3BoxtoBox(q3Manifold *m, q3Box *a, q3Box *b);

#endif // Q3CONTACT_H
