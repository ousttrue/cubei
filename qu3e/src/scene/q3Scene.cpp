//--------------------------------------------------------------------------------------------------
/**
@file	q3Scene.h

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

#include <stdlib.h>

#include "../collision/q3Box.h"
#include "../dynamics/q3Body.h"
#include "../dynamics/q3Contact.h"
#include "../dynamics/q3ContactSolver.h"
#include "../dynamics/q3Island.h"
#include "q3Scene.h"
#include <vector>
#include <Remotery.h>

//--------------------------------------------------------------------------------------------------
// q3Scene
//--------------------------------------------------------------------------------------------------
q3Scene::q3Scene(float dt)
    : m_boxAllocator(sizeof(q3Box), 256), m_dt(dt)
{}

//--------------------------------------------------------------------------------------------------
q3Scene::~q3Scene() { Shutdown(); }

//--------------------------------------------------------------------------------------------------
void q3Scene::Step() {
  rmt_ScopedCPUSample(qSceneStep, 0);

  if (m_newBox) {
    m_contactManager.m_broadphase.UpdatePairs();
    m_newBox = false;
  }

  m_contactManager.TestCollisions();

  for (auto body : m_bodyList) {
    body->m_flags &= ~q3Body::eIsland;
  }

  std::vector<q3Body*> bodies(m_bodyList.size());
  std::vector<q3VelocityState> velocities(m_bodyList.size());
  std::vector<q3ContactConstraint *> contacts(m_contactManager.m_contactCount);
  std::vector<q3ContactConstraintState> contactStates(m_contactManager.m_contactCount);
  // Size the stack island, pick worst case size
  q3Island island;
  island.m_bodyCapacity = m_bodyList.size();
  island.m_contactCapacity = m_contactManager.m_contactCount;
  island.m_bodies = bodies.data();
  island.m_velocities = velocities.data();
  island.m_contacts = contacts.data();
  island.m_contactStates = contactStates.data();
  island.m_allowSleep = m_allowSleep;
  island.m_enableFriction = m_enableFriction;
  island.m_bodyCount = 0;
  island.m_contactCount = 0;
  island.m_dt = m_dt;
  island.m_gravity = m_gravity;
  island.m_iterations = m_iterations;

  // Build each active island and then solve each built island
  int stackSize = m_bodyList.size();
  std::vector<q3Body *> stack(stackSize);
  for (auto seed : m_bodyList) {
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
    stack[stackCount++] = seed;
    island.m_bodyCount = 0;
    island.m_contactCount = 0;

    // Mark seed as apart of island
    seed->m_flags |= q3Body::eIsland;

    // Perform DFS on constraint graph
    while (stackCount > 0) {
      // Decrement stack to implement iterative backtracking
      q3Body *body = stack[--stackCount];
      island.Add(body);

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
        island.Add(contact);

        // Attempt to add the other body in the contact to the island
        // to simulate contact awakening propogation
        q3Body *other = edge->other;
        if (other->m_flags & q3Body::eIsland)
          continue;

        assert(stackCount < stackSize);

        stack[stackCount++] = other;
        other->m_flags |= q3Body::eIsland;
      }
    }

    assert(island.m_bodyCount != 0);

    island.Initialize();
    island.Solve();

    // Reset all static island flags
    // This allows static bodies to participate in other island formations
    for (int i = 0; i < island.m_bodyCount; i++) {
      q3Body *body = island.m_bodies[i];

      if (body->m_flags & q3Body::eStatic)
        body->m_flags &= ~q3Body::eIsland;
    }
  }

  // Update the broadphase AABBs
  for (auto body : m_bodyList) {
    if (body->m_flags & q3Body::eStatic)
      continue;

    body->SynchronizeProxies();
  }

  // Look for new contacts
  m_contactManager.FindNewContacts();

  // Clear all forces
  for (auto body : m_bodyList) {
    body->m_force = {};
    body->m_torque = {};
  }
}

//--------------------------------------------------------------------------------------------------
q3Body *q3Scene::CreateBody(const q3BodyDef &def) {
  auto body = new q3Body(def, this);
  // Add body to scene bodyList
  m_bodyList.push_back(body);
  return body;
}

//--------------------------------------------------------------------------------------------------
void q3Scene::RemoveBody(q3Body *body) {
  m_contactManager.RemoveContactsFromBody(body);

  body->RemoveAllBoxes();

  // Remove body from scene bodyList
  m_bodyList.remove(body);
  delete body;
}

//--------------------------------------------------------------------------------------------------
void q3Scene::RemoveAllBodies() {
  for (auto body : m_bodyList) {
    body->RemoveAllBoxes();
    delete body;
  }
  m_bodyList.clear();
}

//--------------------------------------------------------------------------------------------------
void q3Scene::SetAllowSleep(bool allowSleep) {
  m_allowSleep = allowSleep;

  if (!allowSleep) {
    for (auto body : m_bodyList)
      body->SetToAwake();
  }
}

//--------------------------------------------------------------------------------------------------
void q3Scene::SetIterations(int iterations) {
  m_iterations = q3Max(1, iterations);
}

//--------------------------------------------------------------------------------------------------
void q3Scene::SetEnableFriction(bool enabled) { m_enableFriction = enabled; }

//--------------------------------------------------------------------------------------------------
void q3Scene::Render(q3Render *render) const {
  for (auto body : m_bodyList) {
    body->Render(render);
  }
  m_contactManager.RenderContacts(render);
  m_contactManager.m_broadphase.m_tree.Render(render);
}

//--------------------------------------------------------------------------------------------------
const q3Vec3 q3Scene::GetGravity() const { return m_gravity; }

//--------------------------------------------------------------------------------------------------
void q3Scene::SetGravity(const q3Vec3 &gravity) { m_gravity = gravity; }

//--------------------------------------------------------------------------------------------------
void q3Scene::Shutdown() {
  RemoveAllBodies();

  m_boxAllocator.Clear();
}

//--------------------------------------------------------------------------------------------------
void q3Scene::SetContactListener(q3ContactListener *listener) {
  m_contactManager.m_contactListener = listener;
}

//--------------------------------------------------------------------------------------------------
void q3Scene::QueryAABB(q3QueryCallback *cb, const q3AABB &aabb) const {
  struct SceneQueryWrapper {
    bool TreeCallBack(int id) {
      q3AABB aabb;
      q3Box *box = (q3Box *)broadPhase->m_tree.GetUserData(id);

      box->ComputeAABB(box->body->GetTransform(), &aabb);

      if (q3AABBtoAABB(m_aabb, aabb)) {
        return cb->ReportShape(box);
      }

      return true;
    }

    q3QueryCallback *cb;
    const q3BroadPhase *broadPhase;
    q3AABB m_aabb;
  };

  SceneQueryWrapper wrapper;
  wrapper.m_aabb = aabb;
  wrapper.broadPhase = &m_contactManager.m_broadphase;
  wrapper.cb = cb;
  m_contactManager.m_broadphase.m_tree.Query(&wrapper, aabb);
}

//--------------------------------------------------------------------------------------------------
void q3Scene::QueryPoint(q3QueryCallback *cb, const q3Vec3 &point) const {
  struct SceneQueryWrapper {
    bool TreeCallBack(int id) {
      q3Box *box = (q3Box *)broadPhase->m_tree.GetUserData(id);

      if (box->TestPoint(box->body->GetTransform(), m_point)) {
        cb->ReportShape(box);
      }

      return true;
    }

    q3QueryCallback *cb;
    const q3BroadPhase *broadPhase;
    q3Vec3 m_point;
  };

  SceneQueryWrapper wrapper;
  wrapper.m_point = point;
  wrapper.broadPhase = &m_contactManager.m_broadphase;
  wrapper.cb = cb;
  const float k_fattener = float(0.5);
  q3Vec3 v{k_fattener, k_fattener, k_fattener};
  q3AABB aabb;
  aabb.min = point - v;
  aabb.max = point + v;
  m_contactManager.m_broadphase.m_tree.Query(&wrapper, aabb);
}

//--------------------------------------------------------------------------------------------------
void q3Scene::RayCast(q3QueryCallback *cb, q3RaycastData &rayCast) const {
  struct SceneQueryWrapper {
    bool TreeCallBack(int id) {
      q3Box *box = (q3Box *)broadPhase->m_tree.GetUserData(id);

      if (box->Raycast(box->body->GetTransform(), m_rayCast)) {
        return cb->ReportShape(box);
      }

      return true;
    }

    q3QueryCallback *cb;
    const q3BroadPhase *broadPhase;
    q3RaycastData *m_rayCast;
  };

  SceneQueryWrapper wrapper;
  wrapper.m_rayCast = &rayCast;
  wrapper.broadPhase = &m_contactManager.m_broadphase;
  wrapper.cb = cb;
  m_contactManager.m_broadphase.m_tree.Query(&wrapper, rayCast);
}

//--------------------------------------------------------------------------------------------------
void q3Scene::Dump(FILE *file) const {
  fprintf(file,
          "// Ensure 64/32-bit memory compatability with the dump contents\n");
  fprintf(file, "assert( sizeof( int* ) == %zu );\n", sizeof(int *));
  fprintf(file, "scene.SetGravity( q3Vec3( %.15lf, %.15lf, %.15lf ) );\n",
          m_gravity.x, m_gravity.y, m_gravity.z);
  fprintf(file, "scene.SetAllowSleep( %s );\n",
          m_allowSleep ? "true" : "false");
  fprintf(file, "scene.SetEnableFriction( %s );\n",
          m_enableFriction ? "true" : "false");

  fprintf(file,
          "q3Body** bodies = (q3Body**)q3Alloc( sizeof( q3Body* ) * %d );\n",
          m_bodyList.size());

  int i = 0;
  for (auto body : m_bodyList) {
    body->Dump(file, i++);
  }

  fprintf(file, "q3Free( bodies );\n");
}
