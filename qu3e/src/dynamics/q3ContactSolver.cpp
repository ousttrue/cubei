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
#include "../common/q3Geometry.h"
#include "../common/q3Memory.h"
#include "../common/q3Settings.h"
#include "q3Body.h"
#include "q3Contact.h"
#include "q3Island.h"

//--------------------------------------------------------------------------------------------------
// q3ContactSolver
//--------------------------------------------------------------------------------------------------
void q3ContactSolver::Initialize(q3Island *island) {
  m_island = island;
  m_contactCount = island->m_contacts.size();
  m_contacts = island->m_contactStates.data();
  m_velocities = m_island->m_velocities.data();
  m_enableFriction = island->m_env.m_enableFriction;
}

//--------------------------------------------------------------------------------------------------
void q3ContactSolver::ShutDown(void) {
  for (int i = 0; i < m_contactCount; ++i) {
    q3ContactConstraintState *c = m_contacts + i;
    q3ContactConstraint *cc = m_island->m_contacts[i];

    for (int j = 0; j < c->contactCount; ++j) {
      q3Contact *oc = cc->manifold.contacts + j;
      q3ContactState *cs = c->contacts + j;
      oc->normalImpulse = cs->normalImpulse;
      oc->tangentImpulse[0] = cs->tangentImpulse[0];
      oc->tangentImpulse[1] = cs->tangentImpulse[1];
    }
  }
}

//--------------------------------------------------------------------------------------------------
void q3ContactSolver::PreSolve(float dt) {
  for (int i = 0; i < m_contactCount; ++i) {
    q3ContactConstraintState *cs = m_contacts + i;

    q3Vec3 vA = m_velocities[cs->A.m_islandIndex].v;
    q3Vec3 wA = m_velocities[cs->A.m_islandIndex].w;
    q3Vec3 vB = m_velocities[cs->B.m_islandIndex].v;
    q3Vec3 wB = m_velocities[cs->B.m_islandIndex].w;

    for (int j = 0; j < cs->contactCount; ++j) {
      q3ContactState *c = cs->contacts + j;

      // Precalculate JM^-1JT for contact and friction constraints
      q3Vec3 raCn = q3Cross(c->ra, cs->normal);
      q3Vec3 rbCn = q3Cross(c->rb, cs->normal);
      float nm = cs->A.m_invMass + cs->B.m_invMass;
      float tm[2];
      tm[0] = nm;
      tm[1] = nm;

      nm += q3Dot(raCn, cs->A.m_invInertiaWorld * raCn) +
            q3Dot(rbCn, cs->B.m_invInertiaWorld * rbCn);
      c->normalMass = q3Invert(nm);

      for (int i = 0; i < 2; ++i) {
        q3Vec3 raCt = q3Cross(cs->tangentVectors[i], c->ra);
        q3Vec3 rbCt = q3Cross(cs->tangentVectors[i], c->rb);
        tm[i] += q3Dot(raCt, cs->A.m_invInertiaWorld * raCt) +
                 q3Dot(rbCt, cs->B.m_invInertiaWorld * rbCt);
        c->tangentMass[i] = q3Invert(tm[i]);
      }

      // Precalculate bias factor
      c->bias = -Q3_BAUMGARTE * (float(1.0) / dt) *
                q3Min(float(0.0), c->penetration + Q3_PENETRATION_SLOP);

      // Warm start contact
      q3Vec3 P = cs->normal * c->normalImpulse;

      if (m_enableFriction) {
        P += cs->tangentVectors[0] * c->tangentImpulse[0];
        P += cs->tangentVectors[1] * c->tangentImpulse[1];
      }

      vA -= P * cs->A.m_invMass;
      wA -= cs->A.m_invInertiaWorld * q3Cross(c->ra, P);

      vB += P * cs->B.m_invMass;
      wB += cs->B.m_invInertiaWorld * q3Cross(c->rb, P);

      // Add in restitution bias
      float dv =
          q3Dot(vB + q3Cross(wB, c->rb) - vA - q3Cross(wA, c->ra), cs->normal);

      if (dv < -float(1.0))
        c->bias += -(cs->restitution) * dv;
    }

    m_velocities[cs->A.m_islandIndex].v = vA;
    m_velocities[cs->A.m_islandIndex].w = wA;
    m_velocities[cs->B.m_islandIndex].v = vB;
    m_velocities[cs->B.m_islandIndex].w = wB;
  }
}

//--------------------------------------------------------------------------------------------------
void q3ContactSolver::Solve() {
  for (int i = 0; i < m_contactCount; ++i) {
    q3ContactConstraintState *cs = m_contacts + i;

    q3Vec3 vA = m_velocities[cs->A.m_islandIndex].v;
    q3Vec3 wA = m_velocities[cs->A.m_islandIndex].w;
    q3Vec3 vB = m_velocities[cs->B.m_islandIndex].v;
    q3Vec3 wB = m_velocities[cs->B.m_islandIndex].w;

    for (int j = 0; j < cs->contactCount; ++j) {
      q3ContactState *c = cs->contacts + j;

      // relative velocity at contact
      q3Vec3 dv = vB + q3Cross(wB, c->rb) - vA - q3Cross(wA, c->ra);

      // Friction
      if (m_enableFriction) {
        for (int i = 0; i < 2; ++i) {
          float lambda = -q3Dot(dv, cs->tangentVectors[i]) * c->tangentMass[i];

          // Calculate frictional impulse
          float maxLambda = cs->friction * c->normalImpulse;

          // Clamp frictional impulse
          float oldPT = c->tangentImpulse[i];
          c->tangentImpulse[i] = q3Clamp(-maxLambda, maxLambda, oldPT + lambda);
          lambda = c->tangentImpulse[i] - oldPT;

          // Apply friction impulse
          q3Vec3 impulse = cs->tangentVectors[i] * lambda;
          vA -= impulse * cs->A.m_invMass;
          wA -= cs->A.m_invInertiaWorld * q3Cross(c->ra, impulse);

          vB += impulse * cs->B.m_invMass;
          wB += cs->B.m_invInertiaWorld * q3Cross(c->rb, impulse);
        }
      }

      // Normal
      {
        dv = vB + q3Cross(wB, c->rb) - vA - q3Cross(wA, c->ra);

        // Normal impulse
        float vn = q3Dot(dv, cs->normal);

        // Factor in positional bias to calculate impulse scalar j
        float lambda = c->normalMass * (-vn + c->bias);

        // Clamp impulse
        float tempPN = c->normalImpulse;
        c->normalImpulse = q3Max(tempPN + lambda, float(0.0));
        lambda = c->normalImpulse - tempPN;

        // Apply impulse
        q3Vec3 impulse = cs->normal * lambda;
        vA -= impulse * cs->A.m_invMass;
        wA -= cs->A.m_invInertiaWorld * q3Cross(c->ra, impulse);

        vB += impulse * cs->B.m_invMass;
        wB += cs->B.m_invInertiaWorld * q3Cross(c->rb, impulse);
      }
    }

    m_velocities[cs->A.m_islandIndex].v = vA;
    m_velocities[cs->A.m_islandIndex].w = wA;
    m_velocities[cs->B.m_islandIndex].v = vB;
    m_velocities[cs->B.m_islandIndex].w = wB;
  }
}
