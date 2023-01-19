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
#include "../broadphase/q3BroadPhase.h"
#include "../common/q3Memory.h"
#include "../common/q3Settings.h"
#include "q3Body.h"
#include "q3Contact.h"
#include "q3ContactSolver.h"

#include <Remotery.h>

//--------------------------------------------------------------------------------------------------
// q3Island
//--------------------------------------------------------------------------------------------------
void q3Island::Process(const std::list<q3Body *> &bodyList, size_t contactCount,
                       const q3Env &env) {
  for (auto body : bodyList) {
    body->m_flags &= ~q3Body::eIsland;
  }

  m_bodies.reserve(bodyList.size());
  m_velocities.reserve(bodyList.size());
  m_contacts.reserve(contactCount);
  m_contactStates.reserve(contactCount);
  m_env = env;

  // Build each active island and then solve each built island
  int stackSize = bodyList.size();
  m_stack.resize(stackSize);
  for (auto seed : bodyList) {
    // Seed cannot be apart of an island already
    if (seed->m_flags & q3Body::eIsland)
      continue;

    // Seed must be awake
    if (!(seed->m_flags & q3Body::eAwake))
      continue;

    // Seed cannot be a static body in order to keep islands
    // as small as possible
    if (seed->m_flags & q3Body::eStatic)
      continue;

    int stackCount = 0;
    m_stack[stackCount++] = seed;
    m_bodies.clear();
    m_contacts.clear();

    // Mark seed as apart of island
    seed->m_flags |= q3Body::eIsland;

    // Perform DFS on constraint graph
    while (stackCount > 0) {
      // Decrement stack to implement iterative backtracking
      q3Body *body = m_stack[--stackCount];
      Add(body);

      // Awaken all bodies connected to the island
      body->SetToAwake();

      // Do not search across static bodies to keep island
      // formations as small as possible, however the static
      // body itself should be apart of the island in order
      // to properly represent a full contact
      if (body->m_flags & q3Body::eStatic)
        continue;

      // Search all contacts connected to this body
      q3ContactEdge *contacts = body->m_contactList;
      for (q3ContactEdge *edge = contacts; edge; edge = edge->next) {
        q3ContactConstraint *contact = edge->constraint;

        // Skip contacts that have been added to an island already
        if (contact->m_flags & q3ContactConstraint::eIsland)
          continue;

        // Can safely skip this contact if it didn't actually collide with
        // anything
        if (!(contact->m_flags & q3ContactConstraint::eColliding))
          continue;

        // Skip sensors
        if (contact->A->sensor || contact->B->sensor)
          continue;

        // Mark island flag and add to island
        contact->m_flags |= q3ContactConstraint::eIsland;
        Add(contact);

        // Attempt to add the other body in the contact to the island
        // to simulate contact awakening propogation
        q3Body *other = edge->other;
        if (other->m_flags & q3Body::eIsland)
          continue;

        assert(stackCount < stackSize);

        m_stack[stackCount++] = other;
        other->m_flags |= q3Body::eIsland;
      }
    }

    assert(m_bodies.size() != 0);

    Initialize();
    Solve(m_env);

    // Reset all static island flags
    // This allows static bodies to participate in other island formations
    for (auto body : m_bodies) {
      if (body->m_flags & q3Body::eStatic)
        body->m_flags &= ~q3Body::eIsland;
    }
  }
}

void q3Island::Solve(const q3Env &env) {
  rmt_ScopedCPUSample(q3IslandSolve, 0);

  // Apply gravity
  // Integrate velocities and create state buffers, calculate world inertia
  for (int i = 0; i < m_bodies.size(); ++i) {
    q3Body *body = m_bodies[i];
    q3VelocityState *v = &m_velocities[i];

    if (body->m_flags & q3Body::eDynamic) {
      body->ApplyLinearForce(env.m_gravity * body->m_gravityScale);

      // Calculate world space intertia tensor
      q3Mat3 r = body->m_tx.rotation;
      body->m_invInertiaWorld = r * body->m_invInertiaModel * q3Transpose(r);

      // Integrate velocity
      body->m_linearVelocity += (body->m_force * body->m_invMass) * env.m_dt;
      body->m_angularVelocity +=
          (body->m_invInertiaWorld * body->m_torque) * env.m_dt;

      // From Box2D!
      // Apply damping.
      // ODE: dv/dt + c * v = 0
      // Solution: v(t) = v0 * exp(-c * t)
      // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) *
      // exp(-c * dt) = v * exp(-c * dt) v2 = exp(-c * dt) * v1 Pade
      // approximation: v2 = v1 * 1 / (1 + c * dt)
      body->m_linearVelocity *=
          float(1.0) / (float(1.0) + env.m_dt * body->m_linearDamping);
      body->m_angularVelocity *=
          float(1.0) / (float(1.0) + env.m_dt * body->m_angularDamping);
    }

    v->v = body->m_linearVelocity;
    v->w = body->m_angularVelocity;
  }

  // Create contact solver, pass in state buffers, create buffers for contacts
  // Initialize velocity constraint for normal + friction and warm start
  q3ContactSolver contactSolver;
  contactSolver.Initialize(this);
  contactSolver.PreSolve(env.m_dt);

  // Solve contacts
  for (int i = 0; i < env.m_iterations; ++i)
    contactSolver.Solve();

  contactSolver.ShutDown();

  // Copy back state buffers
  // Integrate positions
  for (int i = 0; i < m_bodies.size(); ++i) {
    q3Body *body = m_bodies[i];
    q3VelocityState *v = &m_velocities[i];

    if (body->m_flags & q3Body::eStatic)
      continue;

    body->m_linearVelocity = v->v;
    body->m_angularVelocity = v->w;

    // Integrate position
    body->m_worldCenter += body->m_linearVelocity * env.m_dt;
    body->m_q.Integrate(body->m_angularVelocity, env.m_dt);
    body->m_q = q3Normalize(body->m_q);
    body->m_tx.rotation = body->m_q.ToMat3();
  }

  if (env.m_allowSleep) {
    // Find minimum sleep time of the entire island
    float minSleepTime = Q3_R32_MAX;
    for (auto body : m_bodies) {
      if (body->m_flags & q3Body::eStatic)
        continue;

      const float sqrLinVel =
          q3Dot(body->m_linearVelocity, body->m_linearVelocity);
      const float cbAngVel =
          q3Dot(body->m_angularVelocity, body->m_angularVelocity);
      const float linTol = Q3_SLEEP_LINEAR;
      const float angTol = Q3_SLEEP_ANGULAR;

      if (sqrLinVel > linTol || cbAngVel > angTol) {
        minSleepTime = float(0.0);
        body->m_sleepTime = float(0.0);
      }

      else {
        body->m_sleepTime += env.m_dt;
        minSleepTime = q3Min(minSleepTime, body->m_sleepTime);
      }
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

//--------------------------------------------------------------------------------------------------
void q3Island::Add(q3Body *body) {
  body->m_islandIndex = m_bodies.size();
  m_bodies.push_back(body);
  m_velocities.push_back({});
}

//--------------------------------------------------------------------------------------------------
void q3Island::Add(q3ContactConstraint *contact) {
  m_contacts.push_back(contact);
  m_contactStates.push_back({});
}

//--------------------------------------------------------------------------------------------------
void q3Island::Initialize() {
  rmt_ScopedCPUSample(q3IslandInitialize, 0);

  for (int i = 0; i < m_contacts.size(); ++i) {
    q3ContactConstraint *cc = m_contacts[i];
    q3ContactConstraintState *c = &m_contactStates[i];

    c->centerA = cc->bodyA->m_worldCenter;
    c->centerB = cc->bodyB->m_worldCenter;
    c->iA = cc->bodyA->m_invInertiaWorld;
    c->iB = cc->bodyB->m_invInertiaWorld;
    c->mA = cc->bodyA->m_invMass;
    c->mB = cc->bodyB->m_invMass;
    c->restitution = cc->restitution;
    c->friction = cc->friction;
    c->indexA = cc->bodyA->m_islandIndex;
    c->indexB = cc->bodyB->m_islandIndex;
    c->normal = cc->manifold.normal;
    c->tangentVectors[0] = cc->manifold.tangentVectors[0];
    c->tangentVectors[1] = cc->manifold.tangentVectors[1];
    c->contactCount = cc->manifold.contactCount;

    int j = 0;
    for (auto &s : c->span()) {
      auto cp = &cc->manifold.contacts[j++];
      s.ra = cp->position - c->centerA;
      s.rb = cp->position - c->centerB;
      s.penetration = cp->penetration;
      s.normalImpulse = cp->normalImpulse;
      s.tangentImpulse[0] = cp->tangentImpulse[0];
      s.tangentImpulse[1] = cp->tangentImpulse[1];
    }
  }
}
