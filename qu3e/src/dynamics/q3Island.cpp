//--------------------------------------------------------------------------------------------------
/**
@file	q3Island.cpp

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

#include "q3Island.h"
#include "../scene/q3Box.h"
#include "../scene/q3Scene.h"
#include "q3BroadPhase.h"
#include "q3Contact.h"
#include "q3ContactConstraint.h"
#include "q3ContactManager.h"
#include "q3ContactSolver.h"

#include <Remotery.h>

q3Island::q3Island(q3Body *seed, class q3ContactManager *contactManager) {

  // Mark seed as apart of island
  seed->AddFlag(q3BodyFlags::eIsland);

  std::vector<class q3Body *> m_stack;
  m_stack.push_back(seed);

  // Perform DFS on constraint graph
  while (!m_stack.empty()) {
    // Decrement stack to implement iterative backtracking
    q3Body *body = m_stack.back();
    m_stack.pop_back();
    m_bodies.push_back(body);

    // Awaken all bodies connected to the island
    body->SetToAwake();

    // Do not search across static bodies to keep island
    // formations as small as possible, however the static
    // body itself should be apart of the island in order
    // to properly represent a full contact
    if (body->HasFlag(q3BodyFlags::eStatic))
      continue;

    // Search all contacts connected to this body
    for (q3ContactEdge *edge = contactManager->ContactEdge(body); edge;
         edge = edge->next) {
      q3ContactConstraint *contact = edge->constraint;

      // Skip contacts that have been added to an island already
      if (contact->HasFlag(q3ContactConstraintFlags::eIsland))
        continue;

      // Can safely skip this contact if it didn't actually collide with
      // anything
      if (!contact->HasFlag(q3ContactConstraintFlags::eColliding))
        continue;

      // Skip sensors
      if (contact->A->Sensor() || contact->B->Sensor())
        continue;

      // Mark island flag and add to island
      contact->AddFlag(q3ContactConstraintFlags::eIsland);
      m_constraints.push_back({contact, {}});

      // Attempt to add the other body in the contact to the island
      // to simulate contact awakening propogation
      q3Body *other = edge->other;
      if (other->HasFlag(q3BodyFlags::eIsland))
        continue;

      m_stack.push_back(other);
      other->AddFlag(q3BodyFlags::eIsland);
    }
  }

  assert(m_bodies.size() != 0);

  for (auto &[cc, c] : m_constraints) {
    c.A = cc->bodyA;
    c.stateA = cc->bodyA->State();
    c.B = cc->bodyB;
    c.stateB = cc->bodyB->State();
    c.restitution = cc->restitution;
    c.friction = cc->friction;
    c.normal = cc->manifold.normal;
    c.tangentVectors[0] = cc->manifold.tangentVectors[0];
    c.tangentVectors[1] = cc->manifold.tangentVectors[1];
    c.contactCount = cc->manifold.contactCount;

    int j = 0;
    for (auto &s : c.span()) {
      auto cp = &cc->manifold.contacts[j++];
      s.ra = cp->position - c.stateA.m_worldCenter;
      s.rb = cp->position - c.stateB.m_worldCenter;
      s.penetration = cp->penetration;
      s.normalImpulse = cp->normalImpulse;
      s.tangentImpulse[0] = cp->tangentImpulse[0];
      s.tangentImpulse[1] = cp->tangentImpulse[1];
    }
  }
}

q3Island::~q3Island() {
  // Reset all static island flags
  // This allows static bodies to participate in other island formations
  for (auto body : m_bodies) {
    if (body->HasFlag(q3BodyFlags::eStatic))
      body->RemoveFlag(q3BodyFlags::eIsland);
  }
}
