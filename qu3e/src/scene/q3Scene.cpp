
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

#include "q3Scene.h"
#include "../collision/q3Box.h"
#include "../dynamics/q3Body.h"
#include "../dynamics/q3Contact.h"
#include "../dynamics/q3ContactSolver.h"
#include "../dynamics/q3Island.h"
#include <Remotery.h>
#include <stdlib.h>
#include <vector>

q3Scene::q3Scene(float dt) : m_boxAllocator(sizeof(q3Box), 256) {
  m_env.m_dt = dt;
}

q3Scene::~q3Scene() { Shutdown(); }

void q3Scene::Step() {
  rmt_ScopedCPUSample(qSceneStep, 0);

  if (m_newBox) {
    m_contactManager.m_broadphase.UpdatePairs();
    m_newBox = false;
  }

  m_contactManager.TestCollisions();

  m_island.Process(m_bodyList, m_contactManager.m_contactCount, m_env);

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

q3Body *q3Scene::CreateBody(const q3BodyDef &def) {
  auto body = new q3Body(def, this);
  // Add body to scene bodyList
  m_bodyList.push_back(body);
  return body;
}

void q3Scene::RemoveBody(q3Body *body) {
  m_contactManager.RemoveContactsFromBody(body);
  body->RemoveAllBoxes();
  // Remove body from scene bodyList
  m_bodyList.remove(body);
  delete body;
}

void q3Scene::RemoveAllBodies() {
  for (auto body : m_bodyList) {
    body->RemoveAllBoxes();
    delete body;
  }
  m_bodyList.clear();
}

void q3Scene::SetAllowSleep(bool allowSleep) {
  m_env.m_allowSleep = allowSleep;
  if (!allowSleep) {
    for (auto body : m_bodyList)
      body->SetToAwake();
  }
}

void q3Scene::Render(q3Render *render) const {
  for (auto body : m_bodyList) {
    body->Render(render);
  }
  m_contactManager.RenderContacts(render);
  m_contactManager.m_broadphase.m_tree.Render(render);
}

void q3Scene::Shutdown() {
  RemoveAllBodies();

  m_boxAllocator.Clear();
}

void q3Scene::SetContactListener(q3ContactListener *listener) {
  m_contactManager.m_contactListener = listener;
}

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

void q3Scene::Dump(FILE *file) const {
  fprintf(file,
          "// Ensure 64/32-bit memory compatability with the dump contents\n");
  fprintf(file, "assert( sizeof( int* ) == %zu );\n", sizeof(int *));
  fprintf(file, "scene.SetGravity( q3Vec3( %.15lf, %.15lf, %.15lf ) );\n",
          m_env.m_gravity.x, m_env.m_gravity.y, m_env.m_gravity.z);
  fprintf(file, "scene.SetAllowSleep( %s );\n",
          m_env.m_allowSleep ? "true" : "false");
  fprintf(file, "scene.SetEnableFriction( %s );\n",
          m_env.m_enableFriction ? "true" : "false");

  fprintf(file,
          "q3Body** bodies = (q3Body**)q3Alloc( sizeof( q3Body* ) * %zu );\n",
          m_bodyList.size());

  int i = 0;
  for (auto body : m_bodyList) {
    body->Dump(file, i++);
  }

  fprintf(file, "q3Free( bodies );\n");
}
