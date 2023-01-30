#include "q3TimeStep.h"
#include "../scene/q3Scene.h"
#include "q3BroadPhase.h"
#include "q3ContactConstraint.h"
#include "q3ContactManager.h"
#include "q3ContactSolver.h"
#include "q3Island.h"
#include <Remotery.h>

void q3TimeStep(const q3Env &env, q3Scene *scene,
                class q3BroadPhase *broadphase,
                q3ContactManager *contactManager) {
  rmt_ScopedCPUSample(q3TimeStep, 0);

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

#if 1
  // Build each active island and then solve each built island
  for (auto seed : *scene) {
    // Seed cannot be apart of an island already
    if (seed->HasFlag(q3BodyFlags::eIsland)) {
      continue;
    }

    // Seed must be awake
    if (!seed->HasFlag(q3BodyFlags::eAwake)) {
      continue;
    }

    // Seed cannot be a static body in order to keep islands
    // as small as possible
    if (seed->HasFlag(q3BodyFlags::eStatic)) {
      continue;
    }

    q3Island island(seed, contactManager);
    q3ContactsSolve(env, island.m_bodies, island.m_constraints);
  }
#else
  // not span. range
  std::vector<q3Body *> bodies;
  for(auto body: *scene)
  {
    bodies.push_back(body);
  }
  std::vector<q3ContactConstraint *> constraints;
  for(auto constraint: *contactManager)
  {
    constraints.push_back(constraint);
  }
  q3ContactsSolve(env, bodies, constraints);
#endif

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
