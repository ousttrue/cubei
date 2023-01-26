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

//--------------------------------------------------------------------------------------------------
// Internal Implementation Constants (do not change unless you know what you're
// doing)
//--------------------------------------------------------------------------------------------------
#define Q3_SLEEP_TIME float(0.5)

//--------------------------------------------------------------------------------------------------
// q3Island
//--------------------------------------------------------------------------------------------------
void q3Island::Step(const q3Env &env, q3Scene *scene,
                    class q3BroadPhase *broadphase,
                    q3ContactManager *contactManager) {
  rmt_ScopedCPUSample(qSceneStep, 0);

  if (scene->NewBox()) {
    broadphase->UpdatePairs(std::bind(
        &q3ContactManager::AddContact, contactManager, std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
  }

  // Remove contacts without broadphase overlap
  {
    rmt_ScopedCPUSample(qTestCollisions, 0);
    auto testOverlap = [broadphase](q3Box *a, q3Box *b) {
      return broadphase->TestOverlap(a->BroadPhaseIndex(),
                                     b->BroadPhaseIndex());
    };
    auto it = contactManager->begin();
    for (; it != contactManager->end();) {
      if ((*it)->Test(testOverlap)) {
        ++it;
      } else {
        it = contactManager->RemoveContact(*it);
      }
    }
  }

  for (auto body : *scene) {
    body->RemoveFlag(q3BodyFlags::eIsland);
  }

  m_bodies.reserve(scene->BodyCount());
  m_velocities.reserve(scene->BodyCount());
  m_stack.resize(scene->BodyCount());
  m_constraints.reserve(contactManager->ContactCount());
  m_constraintStates.reserve(contactManager->ContactCount());

  // Build each active island and then solve each built island
  for (auto seed : *scene) {
    Process(env, seed, contactManager);
  }

  // Update the broadphase AABBs
  scene->UpdateTransforms();

  // Look for new contacts
  // ContactManager for each pair found
  // Has broadphase find all contacts and call AddContact on the
  broadphase->UpdatePairs(
      [contactManager](q3Body *bodyA, q3Box *A, q3Body *bodyB, q3Box *B) {
        contactManager->AddContact(bodyA, A, bodyB, B);
      });

  // Clear all forces
  for (auto body : *scene) {
    body->ClearForce();
  }
}

void q3Island::Process(const q3Env &env, q3Body *seed,
                       class q3ContactManager *contactManager) {
  // Seed cannot be apart of an island already
  if (seed->HasFlag(q3BodyFlags::eIsland))
    return;

  // Seed must be awake
  if (!seed->HasFlag(q3BodyFlags::eAwake))
    return;

  // Seed cannot be a static body in order to keep islands
  // as small as possible
  if (seed->HasFlag(q3BodyFlags::eStatic))
    return;

  // Mark seed as apart of island
  seed->AddFlag(q3BodyFlags::eIsland);

  int stackCount = 0;
  m_stack[stackCount++] = seed;
  m_bodies.clear();
  m_constraints.clear();

  // Perform DFS on constraint graph
  while (stackCount > 0) {
    // Decrement stack to implement iterative backtracking
    q3Body *body = m_stack[--stackCount];
    AddBody(body);

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
      AddConstraint(contact);

      // Attempt to add the other body in the contact to the island
      // to simulate contact awakening propogation
      q3Body *other = edge->other;
      if (other->HasFlag(q3BodyFlags::eIsland))
        continue;

      assert(stackCount < m_stack.size());

      m_stack[stackCount++] = other;
      other->AddFlag(q3BodyFlags::eIsland);
    }
  }

  assert(m_bodies.size() != 0);

  Initialize();
  Solve(env);

  // Reset all static island flags
  // This allows static bodies to participate in other island formations
  for (auto body : m_bodies) {
    if (body->HasFlag(q3BodyFlags::eStatic))
      body->RemoveFlag(q3BodyFlags::eIsland);
  }
}

void q3Island::Solve(const q3Env &env) {
  rmt_ScopedCPUSample(q3IslandSolve, 0);

  // Apply gravity
  // Integrate velocities and create state buffers, calculate world inertia
  for (int i = 0; i < m_bodies.size(); ++i) {
    q3Body *body = m_bodies[i];
    body->Solve(env);
    m_velocities[i] = body->VelocityState();
  }

  {
    // Create contact solver, pass in state buffers, create buffers for contacts
    // Initialize velocity constraint for normal + friction and warm start
    q3ContactSolver contactSolver(env, this);
    for (int i = 0; i < env.m_iterations; ++i)
      contactSolver.Solve();
  }

  // Copy back state buffers
  // Integrate positions
  for (int i = 0; i < m_bodies.size(); ++i) {
    q3Body *body = m_bodies[i];
    body->SetVelocityState(env, m_velocities[i]);
  }

  if (env.m_allowSleep) {
    // Find minimum sleep time of the entire island
    float minSleepTime = std::numeric_limits<float>::max();
    for (auto body : m_bodies) {
      body->Sleep(env, &minSleepTime);
    }

    // Put entire island to sleep so long as the minimum found sleep time
    // is below the threshold. If the minimum sleep time reaches below the
    // sleeping threshold, the entire island will be reformed next step
    // and sleep test will be tried again.
    if (minSleepTime > Q3_SLEEP_TIME) {
      for (auto body : m_bodies)
        body->SetToSleep();
    }
  }
}

void q3Island::Initialize() {
  rmt_ScopedCPUSample(q3IslandInitialize, 0);

  for (int i = 0; i < m_constraints.size(); ++i) {
    q3ContactConstraint *cc = m_constraints[i];
    q3ContactConstraintState *c = &m_constraintStates[i];

    c->A = cc->bodyA->State();
    c->B = cc->bodyB->State();
    c->restitution = cc->restitution;
    c->friction = cc->friction;
    c->normal = cc->manifold.normal;
    c->tangentVectors[0] = cc->manifold.tangentVectors[0];
    c->tangentVectors[1] = cc->manifold.tangentVectors[1];
    c->contactCount = cc->manifold.contactCount;

    int j = 0;
    for (auto &s : c->span()) {
      auto cp = &cc->manifold.contacts[j++];
      s.ra = cp->position - c->A.m_worldCenter;
      s.rb = cp->position - c->B.m_worldCenter;
      s.penetration = cp->penetration;
      s.normalImpulse = cp->normalImpulse;
      s.tangentImpulse[0] = cp->tangentImpulse[0];
      s.tangentImpulse[1] = cp->tangentImpulse[1];
    }
  }
}
