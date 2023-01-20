
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
#include "../dynamics/q3Contact.h"
#include "../dynamics/q3ContactSolver.h"
#include "../dynamics/q3Island.h"
#include "q3Body.h"
#include "q3Box.h"
#include <Remotery.h>
#include <stdlib.h>
#include <vector>

q3Scene::~q3Scene() { RemoveAllBodies(); }

void q3Scene::Step(const q3Env &env) {
  rmt_ScopedCPUSample(qSceneStep, 0);

  m_contactManager.TestCollisions(m_newBox);
  m_newBox = false;

  m_island.Process(m_bodyList, m_contactManager.ContactCount(), env);

  // Update the broadphase AABBs
  for (auto body : m_bodyList) {
    if (body->HasFlag(q3BodyFlags::eStatic))
      continue;
    body->m_transformUpdated();
  }

  // Look for new contacts
  m_contactManager.FindNewContacts();

  // Clear all forces
  for (auto body : m_bodyList) {
    body->ClearForce();
  }
}

q3Body *q3Scene::CreateBody(const q3BodyDef &def) {
  auto body = new q3Body(def);
  body->m_transformUpdated = [scene = this, self = body]() {
    scene->m_contactManager.m_broadphase.SynchronizeProxies(self);
  };
  body->m_onRemoveBox = [scene = this](const q3Box *box) {
    scene->m_contactManager.m_broadphase.RemoveBox(box);
  };
  body->m_onRemoveConstraint = [scene = this](q3ContactConstraint *constraint) {
    scene->m_contactManager.RemoveContact(constraint);
  };

  // Add body to scene bodyList
  m_bodyList.push_back(body);
  return body;
}

//--------------------------------------------------------------------------------------------------
const q3Box *q3Scene::AddBox(q3Body *body, const q3BoxDef &def) {
  auto box = new q3Box;
  box->local = def.m_tx;
  box->e = def.m_e;

  q3AABB aabb;
  box->ComputeAABB(body->Transform(), &aabb);

  box->body = body;
  box->friction = def.m_friction;
  box->restitution = def.m_restitution;
  box->density = def.m_density;
  box->sensor = def.m_sensor;

  body->AddBox(box);
  body->CalculateMassData();

  m_contactManager.m_broadphase.InsertBox(box, aabb);
  m_newBox = true;

  return box;
}

void q3Scene::RemoveBody(q3Body *body) {
  m_contactManager.RemoveContactsFromBody(body);
  body->RemoveAllBoxes();
  m_contactManager.RemoveContactsFromBody(body);

  // Remove body from scene bodyList
  m_bodyList.remove(body);
  delete body;
}

void q3Scene::RemoveAllBodies() {
  for (auto body : m_bodyList) {
    body->RemoveAllBoxes();
    m_contactManager.RemoveContactsFromBody(body);
    delete body;
  }
  m_bodyList.clear();
}

void q3Scene::Dump(FILE *file, const q3Env &m_env) const {
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
