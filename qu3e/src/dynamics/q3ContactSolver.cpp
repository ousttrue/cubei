//--------------------------------------------------------------------------------------------------
/**
@file	q3ContactSolver.cpp

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

#include "q3ContactSolver.h"
#include "../math/q3Math.h"
#include "../scene/q3Env.h"
#include "q3Contact.h"
#include "q3ContactConstraint.h"

#include <Remotery.h>

#define Q3_BAUMGARTE float(0.2)
#define Q3_PENETRATION_SLOP float(0.05)
#define Q3_SLEEP_TIME float(0.5)

struct q3ContactSolver {
  bool m_enableFriction;
  std::span<std::tuple<struct q3ContactConstraint *, q3ContactConstraintState>>
      m_constraints;

public:
  q3ContactSolver(float dt, bool enableFriction,
                  std::span<std::tuple<struct q3ContactConstraint *,
                                       q3ContactConstraintState>>
                      constraints);
  ~q3ContactSolver();
  void Solve(void);
};

q3ContactSolver::q3ContactSolver(
    float dt, bool enableFriction,
    std::span<
        std::tuple<struct q3ContactConstraint *, q3ContactConstraintState>>
        constraints)
    : m_enableFriction(enableFriction), m_constraints(constraints) {

  for (auto &[cc, cs] : m_constraints) {
    q3Vec3 vA = cs.A->VelocityState().linearVelocity;
    q3Vec3 wA = cs.A->VelocityState().angularVelocity;
    q3Vec3 vB = cs.B->VelocityState().linearVelocity;
    q3Vec3 wB = cs.B->VelocityState().angularVelocity;

    for (int j = 0; j < cs.contactCount; ++j) {
      q3ContactState *c = cs.contacts + j;

      // Precalculate JM^-1JT for contact and friction constraints
      q3Vec3 raCn = q3Cross(c->ra, cs.normal);
      q3Vec3 rbCn = q3Cross(c->rb, cs.normal);
      float nm = cs.stateA.m_invMass + cs.stateB.m_invMass;
      float tm[2];
      tm[0] = nm;
      tm[1] = nm;

      nm += q3Dot(raCn, cs.stateA.m_invInertiaWorld * raCn) +
            q3Dot(rbCn, cs.stateB.m_invInertiaWorld * rbCn);
      c->normalMass = q3Invert(nm);

      for (int i = 0; i < 2; ++i) {
        q3Vec3 raCt = q3Cross(cs.tangentVectors[i], c->ra);
        q3Vec3 rbCt = q3Cross(cs.tangentVectors[i], c->rb);
        tm[i] += q3Dot(raCt, cs.stateA.m_invInertiaWorld * raCt) +
                 q3Dot(rbCt, cs.stateB.m_invInertiaWorld * rbCt);
        c->tangentMass[i] = q3Invert(tm[i]);
      }

      // Precalculate bias factor
      c->bias = -Q3_BAUMGARTE * (float(1.0) / dt) *
                std::min(float(0.0), c->penetration + Q3_PENETRATION_SLOP);

      // Warm start contact
      q3Vec3 P = cs.normal * c->normalImpulse;

      if (m_enableFriction) {
        P += cs.tangentVectors[0] * c->tangentImpulse[0];
        P += cs.tangentVectors[1] * c->tangentImpulse[1];
      }

      vA -= P * cs.stateA.m_invMass;
      wA -= cs.stateA.m_invInertiaWorld * q3Cross(c->ra, P);

      vB += P * cs.stateB.m_invMass;
      wB += cs.stateB.m_invInertiaWorld * q3Cross(c->rb, P);

      // Add in restitution bias
      float dv =
          q3Dot(vB + q3Cross(wB, c->rb) - vA - q3Cross(wA, c->ra), cs.normal);

      if (dv < -float(1.0))
        c->bias += -(cs.restitution) * dv;
    }

    cs.A->VelocityState().linearVelocity = vA;
    cs.A->VelocityState().angularVelocity = wA;
    cs.B->VelocityState().linearVelocity = vB;
    cs.B->VelocityState().angularVelocity = wB;
  }
}

q3ContactSolver::~q3ContactSolver(void) {
  for (auto &[cc, c] : m_constraints) {
    for (int j = 0; j < c.contactCount; ++j) {
      q3Contact *oc = cc->manifold.contacts + j;
      q3ContactState *cs = c.contacts + j;
      oc->normalImpulse = cs->normalImpulse;
      oc->tangentImpulse[0] = cs->tangentImpulse[0];
      oc->tangentImpulse[1] = cs->tangentImpulse[1];
    }
  }
}

void q3ContactSolver::Solve() {
  for (auto &[cc, cs] : m_constraints) {

    q3Vec3 vA = cs.A->VelocityState().linearVelocity;
    q3Vec3 wA = cs.A->VelocityState().angularVelocity;
    q3Vec3 vB = cs.B->VelocityState().linearVelocity;
    q3Vec3 wB = cs.B->VelocityState().angularVelocity;

    for (int j = 0; j < cs.contactCount; ++j) {
      q3ContactState *c = cs.contacts + j;

      // relative velocity at contact
      q3Vec3 dv = vB + q3Cross(wB, c->rb) - vA - q3Cross(wA, c->ra);

      // Friction
      if (m_enableFriction) {
        for (int i = 0; i < 2; ++i) {
          float lambda = -q3Dot(dv, cs.tangentVectors[i]) * c->tangentMass[i];

          // Calculate frictional impulse
          float maxLambda = cs.friction * c->normalImpulse;

          // Clamp frictional impulse
          float oldPT = c->tangentImpulse[i];
          c->tangentImpulse[i] = q3Clamp(-maxLambda, maxLambda, oldPT + lambda);
          lambda = c->tangentImpulse[i] - oldPT;

          // Apply friction impulse
          q3Vec3 impulse = cs.tangentVectors[i] * lambda;
          vA -= impulse * cs.stateA.m_invMass;
          wA -= cs.stateA.m_invInertiaWorld * q3Cross(c->ra, impulse);

          vB += impulse * cs.stateB.m_invMass;
          wB += cs.stateB.m_invInertiaWorld * q3Cross(c->rb, impulse);
        }
      }

      // Normal
      {
        dv = vB + q3Cross(wB, c->rb) - vA - q3Cross(wA, c->ra);

        // Normal impulse
        float vn = q3Dot(dv, cs.normal);

        // Factor in positional bias to calculate impulse scalar j
        float lambda = c->normalMass * (-vn + c->bias);

        // Clamp impulse
        float tempPN = c->normalImpulse;
        c->normalImpulse = std::max(tempPN + lambda, float(0.0));
        lambda = c->normalImpulse - tempPN;

        // Apply impulse
        q3Vec3 impulse = cs.normal * lambda;
        vA -= impulse * cs.stateA.m_invMass;
        wA -= cs.stateA.m_invInertiaWorld * q3Cross(c->ra, impulse);

        vB += impulse * cs.stateB.m_invMass;
        wB += cs.stateB.m_invInertiaWorld * q3Cross(c->rb, impulse);
      }
    }

    cs.A->VelocityState().linearVelocity = vA;
    cs.A->VelocityState().angularVelocity = wA;
    cs.B->VelocityState().linearVelocity = vB;
    cs.B->VelocityState().angularVelocity = wB;
  }
}

void q3ContactSolve(const q3Env &env,
                    std::span<std::tuple<struct q3ContactConstraint *,
                                         q3ContactConstraintState>>
                        constraints) {
  // Create contact solver, pass in state buffers, create buffers for contacts
  // Initialize velocity constraint for normal + friction and warm start
  q3ContactSolver contactSolver(env.m_dt, env.m_enableFriction, constraints);
  for (int i = 0; i < env.m_iterations; ++i) {
    contactSolver.Solve();
  }
}

void q3ContactsSolve(const q3Env &env, std::span<q3Body *> bodies,
                     std::span<q3ContactConstraint *> constraints) {
  rmt_ScopedCPUSample(q3ContactsSolve, 0);

  // Apply gravity
  // Integrate velocities and create state buffers, calculate world inertia
  for (auto body : bodies) {
    body->ApplyForce(env);
  }

  std::vector<std::tuple<struct q3ContactConstraint *, q3ContactConstraintState>>
      constraintWithStates;
  for (auto cc : constraints) {
    constraintWithStates.push_back({cc, {}});
    auto &[_, c] = constraintWithStates.back();

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

  // Solve contacts. Modify velocity of bodies
  q3ContactSolve(env, constraintWithStates);

  // Copy back state buffers
  // Integrate positions
  for (auto body : bodies) {
    body->ApplyVelocityState(env);
  }

  if (env.m_allowSleep) {
    // Find minimum sleep time of the entire island
    float minSleepTime = std::numeric_limits<float>::max();
    for (auto body : bodies) {
      body->Sleep(env, &minSleepTime);
    }

    // Put entire island to sleep so long as the minimum found sleep time
    // is below the threshold. If the minimum sleep time reaches below the
    // sleeping threshold, the entire island will be reformed next step
    // and sleep test will be tried again.
    if (minSleepTime > Q3_SLEEP_TIME) {
      for (auto body : bodies) {
        body->SetToSleep();
      }
    }
  }
}
