/*
        Copyright (c) 2012 Hiroshi Matsuike

        This software is provided 'as-is', without any express or implied
        warranty. In no event will the authors be held liable for any damages
        arising from the use of this software.

        Permission is granted to anyone to use this software for any purpose,
        including commercial applications, and to alter it and redistribute it
        freely, subject to the following restrictions:

        1. The origin of this software must not be misrepresented; you must not
        claim that you wrote the original software. If you use this software
        in a product, an acknowledgment in the product documentation would be
        appreciated but is not required.

        2. Altered source versions must be plainly marked as such, and must not
   be misrepresented as being the original software.

        3. This notice may not be removed or altered from any source
   distribution.
*/

#include "EpxConstraintSolver.h"
#include "../collision/EpxVectorFunction.h"
#include "../elements/EpxSolverBody.h"

namespace EasyPhysics {

void epxSolveConstraints(EpxState *states, const EpxRigidBody *bodies,
                         EpxUInt32 numRigidBodies,
                         std::span<const EpxPair> pairs, EpxBallJoint *joints,
                         EpxUInt32 numJoints, EpxUInt32 iteration,
                         EpxFloat bias, EpxFloat slop, EpxFloat timeStep,
                         EpxAllocator *allocator) {
  assert(states);
  assert(bodies);

  // ソルバー用プロキシを作成
  EpxSolverBody *solverBodies = (EpxSolverBody *)allocator->allocate(
      sizeof(EpxSolverBody) * numRigidBodies);
  assert(solverBodies);

  for (EpxUInt32 i = 0; i < numRigidBodies; i++) {
    EpxState &state = states[i];
    const EpxRigidBody &body = bodies[i];
    EpxSolverBody &solverBody = solverBodies[i];

    solverBody.orientation = state.m_orientation;
    solverBody.deltaLinearVelocity = EpxVector3(0.0f);
    solverBody.deltaAngularVelocity = EpxVector3(0.0f);

    if (state.m_motionType == EpxMotionTypeStatic) {
      solverBody.massInv = 0.0f;
      solverBody.inertiaInv = EpxMatrix3(0.0f);
    } else {
      solverBody.massInv = 1.0f / body.m_mass;
      EpxMatrix3 m(solverBody.orientation);
      solverBody.inertiaInv = m * inverse(body.m_inertia) * transpose(m);
    }
  }

  // 拘束のセットアップ
  for (EpxUInt32 i = 0; i < numJoints; i++) {
    EpxBallJoint &joint = joints[i];

    EpxState &stateA = states[joint.rigidBodyA];
    const EpxRigidBody &bodyA = bodies[joint.rigidBodyA];
    EpxSolverBody &solverBodyA = solverBodies[joint.rigidBodyA];

    EpxState &stateB = states[joint.rigidBodyB];
    const EpxRigidBody &bodyB = bodies[joint.rigidBodyB];
    EpxSolverBody &solverBodyB = solverBodies[joint.rigidBodyB];

    EpxVector3 rA = rotate(solverBodyA.orientation, joint.anchorA);
    EpxVector3 rB = rotate(solverBodyB.orientation, joint.anchorB);

    EpxVector3 positionA = stateA.m_position + rA;
    EpxVector3 positionB = stateB.m_position + rB;
    EpxVector3 direction = positionA - positionB;
    EpxFloat distanceSqr = lengthSqr(direction);

    if (distanceSqr < EPX_EPSILON * EPX_EPSILON) {
      joint.constraint.jacDiagInv = 0.0f;
      joint.constraint.rhs = 0.0f;
      joint.constraint.lowerLimit = -EPX_FLT_MAX;
      joint.constraint.upperLimit = EPX_FLT_MAX;
      joint.constraint.axis = EpxVector3(1.0f, 0.0f, 0.0f);
      continue;
    }

    EpxFloat distance = sqrtf(distanceSqr);
    direction /= distance;

    EpxVector3 velocityA =
        stateA.m_linearVelocity + cross(stateA.m_angularVelocity, rA);
    EpxVector3 velocityB =
        stateB.m_linearVelocity + cross(stateB.m_angularVelocity, rB);
    EpxVector3 relativeVelocity = velocityA - velocityB;

    EpxMatrix3 K = EpxMatrix3::scale(
                       EpxVector3(solverBodyA.massInv + solverBodyB.massInv)) -
                   crossMatrix(rA) * solverBodyA.inertiaInv * crossMatrix(rA) -
                   crossMatrix(rB) * solverBodyB.inertiaInv * crossMatrix(rB);

    EpxFloat denom = dot(K * direction, direction);
    joint.constraint.jacDiagInv = 1.0f / denom;
    joint.constraint.rhs = -dot(relativeVelocity, direction); // velocity error
    joint.constraint.rhs -= joint.bias * distance / timeStep; // position error
    joint.constraint.rhs *= joint.constraint.jacDiagInv;
    joint.constraint.lowerLimit = -EPX_FLT_MAX;
    joint.constraint.upperLimit = EPX_FLT_MAX;
    joint.constraint.axis = direction;

    joint.constraint.accumImpulse = 0.0f;
  }

  for (auto &pair : pairs) {
    EpxState &stateA = states[pair.rigidBodyA];
    const EpxRigidBody &bodyA = bodies[pair.rigidBodyA];
    EpxSolverBody &solverBodyA = solverBodies[pair.rigidBodyA];

    EpxState &stateB = states[pair.rigidBodyB];
    const EpxRigidBody &bodyB = bodies[pair.rigidBodyB];
    EpxSolverBody &solverBodyB = solverBodies[pair.rigidBodyB];

    assert(pair.contact);

    pair.contact->m_friction = sqrtf(bodyA.m_friction * bodyB.m_friction);

    for (EpxUInt32 j = 0; j < pair.contact->m_numContacts; j++) {
      EpxContactPoint &cp = pair.contact->m_contactPoints[j];

      EpxVector3 rA = rotate(solverBodyA.orientation, cp.pointA);
      EpxVector3 rB = rotate(solverBodyB.orientation, cp.pointB);

      EpxMatrix3 K =
          EpxMatrix3::scale(
              EpxVector3(solverBodyA.massInv + solverBodyB.massInv)) -
          crossMatrix(rA) * solverBodyA.inertiaInv * crossMatrix(rA) -
          crossMatrix(rB) * solverBodyB.inertiaInv * crossMatrix(rB);

      EpxVector3 velocityA =
          stateA.m_linearVelocity + cross(stateA.m_angularVelocity, rA);
      EpxVector3 velocityB =
          stateB.m_linearVelocity + cross(stateB.m_angularVelocity, rB);
      EpxVector3 relativeVelocity = velocityA - velocityB;

      EpxVector3 tangent1, tangent2;

      epxCalcTangentVector(cp.normal, tangent1, tangent2);

      EpxFloat restitution =
          pair.type == EpxPairTypeNew
              ? 0.5f * (bodyA.m_restitution + bodyB.m_restitution)
              : 0.0f;

      // Normal
      {
        EpxVector3 axis = cp.normal;
        EpxFloat denom = dot(K * axis, axis);
        cp.constraints[0].jacDiagInv = 1.0f / denom;
        cp.constraints[0].rhs = -(1.0f + restitution) *
                                dot(relativeVelocity, axis); // velocity error
        cp.constraints[0].rhs -= (bias * EPX_MIN(0.0f, cp.distance + slop)) /
                                 timeStep; // position error
        cp.constraints[0].rhs *= cp.constraints[0].jacDiagInv;
        cp.constraints[0].lowerLimit = 0.0f;
        cp.constraints[0].upperLimit = EPX_FLT_MAX;
        cp.constraints[0].axis = axis;
      }

      // Tangent1
      {
        EpxVector3 axis = tangent1;
        EpxFloat denom = dot(K * axis, axis);
        cp.constraints[1].jacDiagInv = 1.0f / denom;
        cp.constraints[1].rhs = -dot(relativeVelocity, axis);
        cp.constraints[1].rhs *= cp.constraints[1].jacDiagInv;
        cp.constraints[1].lowerLimit = 0.0f;
        cp.constraints[1].upperLimit = 0.0f;
        cp.constraints[1].axis = axis;
      }

      // Tangent2
      {
        EpxVector3 axis = tangent2;
        EpxFloat denom = dot(K * axis, axis);
        cp.constraints[2].jacDiagInv = 1.0f / denom;
        cp.constraints[2].rhs = -dot(relativeVelocity, axis);
        cp.constraints[2].rhs *= cp.constraints[2].jacDiagInv;
        cp.constraints[2].lowerLimit = 0.0f;
        cp.constraints[2].upperLimit = 0.0f;
        cp.constraints[2].axis = axis;
      }
    }
  }

  // Warm starting
  for (auto &pair : pairs) {
    EpxSolverBody &solverBodyA = solverBodies[pair.rigidBodyA];
    EpxSolverBody &solverBodyB = solverBodies[pair.rigidBodyB];

    for (EpxUInt32 j = 0; j < pair.contact->m_numContacts; j++) {
      EpxContactPoint &cp = pair.contact->m_contactPoints[j];
      EpxVector3 rA = rotate(solverBodyA.orientation, cp.pointA);
      EpxVector3 rB = rotate(solverBodyB.orientation, cp.pointB);

      for (EpxUInt32 k = 0; k < 3; k++) {
        EpxFloat deltaImpulse = cp.constraints[k].accumImpulse;
        solverBodyA.deltaLinearVelocity +=
            deltaImpulse * solverBodyA.massInv * cp.constraints[k].axis;
        solverBodyA.deltaAngularVelocity += deltaImpulse *
                                            solverBodyA.inertiaInv *
                                            cross(rA, cp.constraints[k].axis);
        solverBodyB.deltaLinearVelocity -=
            deltaImpulse * solverBodyB.massInv * cp.constraints[k].axis;
        solverBodyB.deltaAngularVelocity -= deltaImpulse *
                                            solverBodyB.inertiaInv *
                                            cross(rB, cp.constraints[k].axis);
      }
    }
  }

  // 拘束の演算
  for (EpxUInt32 itr = 0; itr < iteration; itr++) {
    for (EpxUInt32 i = 0; i < numJoints; i++) {
      EpxBallJoint &joint = joints[i];

      EpxSolverBody &solverBodyA = solverBodies[joint.rigidBodyA];
      EpxSolverBody &solverBodyB = solverBodies[joint.rigidBodyB];

      EpxVector3 rA = rotate(solverBodyA.orientation, joint.anchorA);
      EpxVector3 rB = rotate(solverBodyB.orientation, joint.anchorB);

      EpxConstraint &constraint = joint.constraint;
      EpxFloat deltaImpulse = constraint.rhs;
      EpxVector3 deltaVelocityA = solverBodyA.deltaLinearVelocity +
                                  cross(solverBodyA.deltaAngularVelocity, rA);
      EpxVector3 deltaVelocityB = solverBodyB.deltaLinearVelocity +
                                  cross(solverBodyB.deltaAngularVelocity, rB);
      deltaImpulse -= constraint.jacDiagInv *
                      dot(constraint.axis, deltaVelocityA - deltaVelocityB);
      EpxFloat oldImpulse = constraint.accumImpulse;
      constraint.accumImpulse =
          EPX_CLAMP(oldImpulse + deltaImpulse, constraint.lowerLimit,
                    constraint.upperLimit);
      deltaImpulse = constraint.accumImpulse - oldImpulse;
      solverBodyA.deltaLinearVelocity +=
          deltaImpulse * solverBodyA.massInv * constraint.axis;
      solverBodyA.deltaAngularVelocity +=
          deltaImpulse * solverBodyA.inertiaInv * cross(rA, constraint.axis);
      solverBodyB.deltaLinearVelocity -=
          deltaImpulse * solverBodyB.massInv * constraint.axis;
      solverBodyB.deltaAngularVelocity -=
          deltaImpulse * solverBodyB.inertiaInv * cross(rB, constraint.axis);
    }

    for (auto &pair : pairs) {
      EpxSolverBody &solverBodyA = solverBodies[pair.rigidBodyA];
      EpxSolverBody &solverBodyB = solverBodies[pair.rigidBodyB];

      for (EpxUInt32 j = 0; j < pair.contact->m_numContacts; j++) {
        EpxContactPoint &cp = pair.contact->m_contactPoints[j];
        EpxVector3 rA = rotate(solverBodyA.orientation, cp.pointA);
        EpxVector3 rB = rotate(solverBodyB.orientation, cp.pointB);

        {
          EpxConstraint &constraint = cp.constraints[0];
          EpxFloat deltaImpulse = constraint.rhs;
          EpxVector3 deltaVelocityA =
              solverBodyA.deltaLinearVelocity +
              cross(solverBodyA.deltaAngularVelocity, rA);
          EpxVector3 deltaVelocityB =
              solverBodyB.deltaLinearVelocity +
              cross(solverBodyB.deltaAngularVelocity, rB);
          deltaImpulse -= constraint.jacDiagInv *
                          dot(constraint.axis, deltaVelocityA - deltaVelocityB);
          EpxFloat oldImpulse = constraint.accumImpulse;
          constraint.accumImpulse =
              EPX_CLAMP(oldImpulse + deltaImpulse, constraint.lowerLimit,
                        constraint.upperLimit);
          deltaImpulse = constraint.accumImpulse - oldImpulse;
          solverBodyA.deltaLinearVelocity +=
              deltaImpulse * solverBodyA.massInv * constraint.axis;
          solverBodyA.deltaAngularVelocity += deltaImpulse *
                                              solverBodyA.inertiaInv *
                                              cross(rA, constraint.axis);
          solverBodyB.deltaLinearVelocity -=
              deltaImpulse * solverBodyB.massInv * constraint.axis;
          solverBodyB.deltaAngularVelocity -= deltaImpulse *
                                              solverBodyB.inertiaInv *
                                              cross(rB, constraint.axis);
        }

        EpxFloat maxFriction =
            pair.contact->m_friction * fabs(cp.constraints[0].accumImpulse);
        cp.constraints[1].lowerLimit = -maxFriction;
        cp.constraints[1].upperLimit = maxFriction;
        cp.constraints[2].lowerLimit = -maxFriction;
        cp.constraints[2].upperLimit = maxFriction;

        {
          EpxConstraint &constraint = cp.constraints[1];
          EpxFloat deltaImpulse = constraint.rhs;
          EpxVector3 deltaVelocityA =
              solverBodyA.deltaLinearVelocity +
              cross(solverBodyA.deltaAngularVelocity, rA);
          EpxVector3 deltaVelocityB =
              solverBodyB.deltaLinearVelocity +
              cross(solverBodyB.deltaAngularVelocity, rB);
          deltaImpulse -= constraint.jacDiagInv *
                          dot(constraint.axis, deltaVelocityA - deltaVelocityB);
          EpxFloat oldImpulse = constraint.accumImpulse;
          constraint.accumImpulse =
              EPX_CLAMP(oldImpulse + deltaImpulse, constraint.lowerLimit,
                        constraint.upperLimit);
          deltaImpulse = constraint.accumImpulse - oldImpulse;
          solverBodyA.deltaLinearVelocity +=
              deltaImpulse * solverBodyA.massInv * constraint.axis;
          solverBodyA.deltaAngularVelocity += deltaImpulse *
                                              solverBodyA.inertiaInv *
                                              cross(rA, constraint.axis);
          solverBodyB.deltaLinearVelocity -=
              deltaImpulse * solverBodyB.massInv * constraint.axis;
          solverBodyB.deltaAngularVelocity -= deltaImpulse *
                                              solverBodyB.inertiaInv *
                                              cross(rB, constraint.axis);
        }
        {
          EpxConstraint &constraint = cp.constraints[2];
          EpxFloat deltaImpulse = constraint.rhs;
          EpxVector3 deltaVelocityA =
              solverBodyA.deltaLinearVelocity +
              cross(solverBodyA.deltaAngularVelocity, rA);
          EpxVector3 deltaVelocityB =
              solverBodyB.deltaLinearVelocity +
              cross(solverBodyB.deltaAngularVelocity, rB);
          deltaImpulse -= constraint.jacDiagInv *
                          dot(constraint.axis, deltaVelocityA - deltaVelocityB);
          EpxFloat oldImpulse = constraint.accumImpulse;
          constraint.accumImpulse =
              EPX_CLAMP(oldImpulse + deltaImpulse, constraint.lowerLimit,
                        constraint.upperLimit);
          deltaImpulse = constraint.accumImpulse - oldImpulse;
          solverBodyA.deltaLinearVelocity +=
              deltaImpulse * solverBodyA.massInv * constraint.axis;
          solverBodyA.deltaAngularVelocity += deltaImpulse *
                                              solverBodyA.inertiaInv *
                                              cross(rA, constraint.axis);
          solverBodyB.deltaLinearVelocity -=
              deltaImpulse * solverBodyB.massInv * constraint.axis;
          solverBodyB.deltaAngularVelocity -= deltaImpulse *
                                              solverBodyB.inertiaInv *
                                              cross(rB, constraint.axis);
        }
      }
    }
  }

  // 速度を更新
  for (EpxUInt32 i = 0; i < numRigidBodies; i++) {
    states[i].m_linearVelocity += solverBodies[i].deltaLinearVelocity;
    states[i].m_angularVelocity += solverBodies[i].deltaAngularVelocity;
  }

  allocator->deallocate(solverBodies);
}

} // namespace EasyPhysics
