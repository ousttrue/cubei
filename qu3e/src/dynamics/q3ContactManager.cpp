//--------------------------------------------------------------------------------------------------
/**
@file	q3ContactManager.cpp

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

#include "q3ContactManager.h"
#include "../debug/q3Render.h"
#include "../scene/q3Body.h"
#include "../scene/q3Box.h"
#include "../scene/q3Scene.h"
#include "q3Contact.h"

#include <Remotery.h>

//--------------------------------------------------------------------------------------------------
// q3ContactManager
//--------------------------------------------------------------------------------------------------
q3ContactManager::q3ContactManager() : m_broadphase(this) {}

//--------------------------------------------------------------------------------------------------
void q3ContactManager::AddContact(q3Box *A, q3Box *B) {
  q3Body *bodyA = A->body;
  q3Body *bodyB = B->body;
  if (!bodyA->CanCollide(bodyB))
    return;

  // Search for existing matching contact
  // Return if found duplicate to avoid duplicate constraints
  // Mark pre-existing duplicates as active
  q3ContactEdge *edge = A->body->m_contactList;
  while (edge) {
    if (edge->other == bodyB) {
      q3Box *shapeA = edge->constraint->A;
      q3Box *shapeB = edge->constraint->B;

      // @TODO: Verify this against Box2D; not sure if this is all we need here
      if ((A == shapeA) && (B == shapeB))
        return;
    }

    edge = edge->next;
  }

  // Create new contact
  auto contact = new q3ContactConstraint;
  contact->A = A;
  contact->B = B;
  contact->bodyA = A->body;
  contact->bodyB = B->body;
  contact->manifold.SetPair(A, B);
  contact->m_flags = {};
  contact->friction = q3MixFriction(A, B);
  contact->restitution = q3MixRestitution(A, B);
  contact->manifold.contactCount = 0;

  for (int i = 0; i < 8; ++i)
    contact->manifold.contacts[i].warmStarted = 0;

  m_contactList.push_back(contact);

  // Connect A
  contact->edgeA.constraint = contact;
  contact->edgeA.other = bodyB;

  contact->edgeA.prev = NULL;
  contact->edgeA.next = bodyA->m_contactList;
  if (bodyA->m_contactList)
    bodyA->m_contactList->prev = &contact->edgeA;
  bodyA->m_contactList = &contact->edgeA;

  // Connect B
  contact->edgeB.constraint = contact;
  contact->edgeB.other = bodyA;

  contact->edgeB.prev = NULL;
  contact->edgeB.next = bodyB->m_contactList;
  if (bodyB->m_contactList)
    bodyB->m_contactList->prev = &contact->edgeB;
  bodyB->m_contactList = &contact->edgeB;

  bodyA->SetToAwake();
  bodyB->SetToAwake();
}

//--------------------------------------------------------------------------------------------------
void q3ContactManager::FindNewContacts() { m_broadphase.UpdatePairs(); }

//--------------------------------------------------------------------------------------------------
std::list<q3ContactConstraint *>::iterator
q3ContactManager::RemoveContact(q3ContactConstraint *contact) {
  q3Body *A = contact->bodyA;
  q3Body *B = contact->bodyB;

  // Remove from A
  if (contact->edgeA.prev)
    contact->edgeA.prev->next = contact->edgeA.next;

  if (contact->edgeA.next)
    contact->edgeA.next->prev = contact->edgeA.prev;

  if (&contact->edgeA == A->m_contactList)
    A->m_contactList = contact->edgeA.next;

  // Remove from B
  if (contact->edgeB.prev)
    contact->edgeB.prev->next = contact->edgeB.next;

  if (contact->edgeB.next)
    contact->edgeB.next->prev = contact->edgeB.prev;

  if (&contact->edgeB == B->m_contactList)
    B->m_contactList = contact->edgeB.next;

  A->SetToAwake();
  B->SetToAwake();

  // Remove contact from the manager
  auto it = std::find(m_contactList.begin(), m_contactList.end(), contact);
  assert(it != m_contactList.end());
  it = m_contactList.erase(it);
  delete contact;
  return it;
}

//--------------------------------------------------------------------------------------------------
void q3ContactManager::RemoveContactsFromBody(q3Body *body) {
  q3ContactEdge *edge = body->m_contactList;

  while (edge) {
    q3ContactEdge *next = edge->next;
    RemoveContact(edge->constraint);
    edge = next;
  }
}

//--------------------------------------------------------------------------------------------------
void q3ContactManager::RemoveFromBroadphase(q3Body *body) {
  for (auto box : *body) {
    m_broadphase.RemoveBox(box);
  }
}

//--------------------------------------------------------------------------------------------------
void q3ContactManager::TestCollisions(bool newBox) {
  rmt_ScopedCPUSample(qTestCollisions, 0);
  if (newBox) {
    m_broadphase.UpdatePairs();
  }

  auto it = m_contactList.begin();
  for (; it != m_contactList.end();) {
    auto constraint = *it;
    q3Box *A = constraint->A;
    q3Box *B = constraint->B;
    q3Body *bodyA = A->body;
    q3Body *bodyB = B->body;

    constraint->RemoveFlag(q3ContactConstraintFlags::eIsland);

    if (!bodyA->IsAwake() && !bodyB->IsAwake()) {
      ++it;
      continue;
    }

    if (!bodyA->CanCollide(bodyB)) {
      it = RemoveContact(constraint);
      continue;
    }

    // Check if contact should persist
    if (!m_broadphase.TestOverlap(A->broadPhaseIndex, B->broadPhaseIndex)) {
      it = RemoveContact(constraint);
      continue;
    }
    q3Manifold *manifold = &constraint->manifold;
    q3Manifold oldManifold = constraint->manifold;
    q3Vec3 ot0 = oldManifold.tangentVectors[0];
    q3Vec3 ot1 = oldManifold.tangentVectors[1];
    constraint->SolveCollision();
    q3ComputeBasis(manifold->normal, manifold->tangentVectors,
                   manifold->tangentVectors + 1);

    for (int i = 0; i < manifold->contactCount; ++i) {
      q3Contact *c = manifold->contacts + i;
      c->tangentImpulse[0] = c->tangentImpulse[1] = c->normalImpulse =
          float(0.0);
      uint8_t oldWarmStart = c->warmStarted;
      c->warmStarted = uint8_t(0);

      for (int j = 0; j < oldManifold.contactCount; ++j) {
        q3Contact *oc = oldManifold.contacts + j;
        if (c->fp.key == oc->fp.key) {
          c->normalImpulse = oc->normalImpulse;

          // Attempt to re-project old friction solutions
          q3Vec3 friction =
              ot0 * oc->tangentImpulse[0] + ot1 * oc->tangentImpulse[1];
          c->tangentImpulse[0] = q3Dot(friction, manifold->tangentVectors[0]);
          c->tangentImpulse[1] = q3Dot(friction, manifold->tangentVectors[1]);
          c->warmStarted = q3Max(oldWarmStart, uint8_t(oldWarmStart + 1));
          break;
        }
      }
    }

    if (m_contactListener) {
      auto now_colliding =
          (int)constraint->m_flags & (int)q3ContactConstraintFlags::eColliding;
      auto was_colliding = (int)constraint->m_flags &
                           (int)q3ContactConstraintFlags::eWasColliding;

      if (now_colliding && !was_colliding) {
        m_contactListener->BeginContact(constraint);
      } else if (!now_colliding && was_colliding) {
        m_contactListener->EndContact(constraint);
      }
    }
    ++it;
  }
}

//--------------------------------------------------------------------------------------------------
void q3ContactManager::RenderContacts(q3Render *render) const {
  for (auto contact : m_contactList) {
    const q3Manifold *m = &contact->manifold;

    if (!contact->HasFlag(q3ContactConstraintFlags::eColliding)) {
      continue;
    }

    for (int j = 0; j < m->contactCount; ++j) {
      const q3Contact *c = m->contacts + j;
      float blue = (float)(255 - c->warmStarted) / 255.0f;
      float red = 1.0f - blue;
      render->SetScale(10.0f, 10.0f, 10.0f);
      render->SetPenColor(red, blue, blue);
      render->SetPenPosition(c->position.x, c->position.y, c->position.z);
      render->Point();

      if (m->A->body->IsAwake())
        render->SetPenColor(1.0f, 1.0f, 1.0f);
      else
        render->SetPenColor(0.2f, 0.2f, 0.2f);

      render->SetPenPosition(c->position.x, c->position.y, c->position.z);
      render->Line(c->position.x + m->normal.x * 0.5f,
                   c->position.y + m->normal.y * 0.5f,
                   c->position.z + m->normal.z * 0.5f);
    }
  }

  render->SetScale(1.0f, 1.0f, 1.0f);
}

void q3ContactManager::QueryAABB(q3QueryCallback *cb,
                                 const q3AABB &aabb) const {
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
  wrapper.broadPhase = &m_broadphase;
  wrapper.cb = cb;
  m_broadphase.m_tree.Query(&wrapper, aabb);
}

void q3ContactManager::QueryPoint(q3QueryCallback *cb,
                                  const q3Vec3 &point) const {
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
  wrapper.broadPhase = &m_broadphase;
  wrapper.cb = cb;
  const float k_fattener = float(0.5);
  q3Vec3 v{k_fattener, k_fattener, k_fattener};
  q3AABB aabb;
  aabb.min = point - v;
  aabb.max = point + v;
  m_broadphase.m_tree.Query(&wrapper, aabb);
}

void q3ContactManager::RayCast(q3QueryCallback *cb,
                               q3RaycastData &rayCast) const {
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
  wrapper.broadPhase = &m_broadphase;
  wrapper.cb = cb;
  m_broadphase.m_tree.Query(&wrapper, rayCast);
}
