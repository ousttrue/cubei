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
#include "q3Contact.h"
#include "q3ContactConstraint.h"
#include "q3Magnifold.h"
#include <q3Render.h>

#include <Remotery.h>

// Restitution mixing. The idea is to use the maximum bounciness, so bouncy
// objects will never not bounce during collisions.
static float q3MixRestitution(const q3Box *A, const q3Box *B) {
  return std::max(A->Restitution(), B->Restitution());
}

// Friction mixing. The idea is to allow a very low friction value to
// drive down the mixing result. Example: anything slides on ice.
static float q3MixFriction(const q3Box *A, const q3Box *B) {
  return std::sqrt(A->Friction() * B->Friction());
}

q3ContactManager::q3ContactManager() {}

void q3ContactManager::AddContact(q3Body *bodyA, q3Box *A, q3Body *bodyB,
                                  q3Box *B) {
  if (!bodyA->CanCollide(bodyB))
    return;

  // Search for existing matching contact
  // Return if found duplicate to avoid duplicate constraints
  // Mark pre-existing duplicates as active
  for (q3ContactEdge *edge = ContactEdge(bodyA); edge; edge = edge->next) {
    if (edge->other == bodyB) {
      // @TODO: Verify this against Box2D; not sure if this is all we need here
      if ((A == edge->constraint->A) && (B == edge->constraint->B))
        return;
    }
  }

  // Create new contact
  auto contact = new q3ContactConstraint;
  contact->A = A;
  contact->B = B;
  contact->bodyA = bodyA;
  contact->bodyB = bodyB;
  contact->m_flags = {};
  contact->friction = q3MixFriction(A, B);
  contact->restitution = q3MixRestitution(A, B);

  contact->manifold.SetPair(bodyA, A, bodyB, B);
  contact->manifold.contactCount = 0;
  for (int i = 0; i < 8; ++i)
    contact->manifold.contacts[i].warmStarted = 0;

  m_contactList.push_back(contact);

  // Connect A
  contact->edgeA.constraint = contact;
  contact->edgeA.other = bodyB;

  contact->edgeA.prev = NULL;
  auto edgeA = ContactEdge(bodyA);
  contact->edgeA.next = edgeA;
  if (edgeA)
    edgeA->prev = &contact->edgeA;
  m_edgeMap[bodyA] = &contact->edgeA;

  // Connect B
  contact->edgeB.constraint = contact;
  contact->edgeB.other = bodyA;

  contact->edgeB.prev = NULL;
  auto edgeB = ContactEdge(bodyB);
  contact->edgeB.next = edgeB;
  if (edgeB)
    edgeB->prev = &contact->edgeB;
  m_edgeMap[bodyB] = &contact->edgeB;

  bodyA->SetToAwake();
  bodyB->SetToAwake();
}

std::list<q3ContactConstraint *>::iterator
q3ContactManager::RemoveContact(q3ContactConstraint *contact) {
  q3Body *A = contact->bodyA;
  q3Body *B = contact->bodyB;

  // Remove from A
  if (contact->edgeA.prev)
    contact->edgeA.prev->next = contact->edgeA.next;

  if (contact->edgeA.next)
    contact->edgeA.next->prev = contact->edgeA.prev;

  if (&contact->edgeA == ContactEdge(A)) {
    m_edgeMap[A] = contact->edgeA.next;
  }

  // Remove from B
  if (contact->edgeB.prev)
    contact->edgeB.prev->next = contact->edgeB.next;

  if (contact->edgeB.next)
    contact->edgeB.next->prev = contact->edgeB.prev;

  if (&contact->edgeB == ContactEdge(B))
    m_edgeMap[B] = contact->edgeB.next;

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
  q3ContactEdge *edge = ContactEdge(body);

  while (edge) {
    q3ContactEdge *next = edge->next;
    RemoveContact(edge->constraint);
    edge = next;
  }
}

//--------------------------------------------------------------------------------------------------
void q3ContactManager::TestCollisions(
    const std::function<bool(int a, int b)> &testOverlap) {
  rmt_ScopedCPUSample(qTestCollisions, 0);
  auto it = m_contactList.begin();
  for (; it != m_contactList.end();) {
    auto constraint = *it;
    q3Box *A = constraint->A;
    q3Box *B = constraint->B;
    q3Body *bodyA = constraint->bodyA;
    q3Body *bodyB = constraint->bodyB;
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
    if (!testOverlap(A->BroadPhaseIndex(), B->BroadPhaseIndex())) {
      it = RemoveContact(constraint);
      continue;
    }
    q3Manifold *manifold = &constraint->manifold;
    q3Manifold oldManifold = constraint->manifold;
    q3Vec3 ot0 = oldManifold.tangentVectors[0];
    q3Vec3 ot1 = oldManifold.tangentVectors[1];
    // constraint->SolveCollision();
    // void q3ContactConstraint::SolveCollision(void) {
    manifold->contactCount = 0;

    q3BoxtoBox(manifold, bodyA, A, bodyB, B);

    if (manifold->contactCount > 0) {
      if (constraint->HasFlag(q3ContactConstraintFlags::eColliding)) {
        constraint->AddFlag(q3ContactConstraintFlags::eWasColliding);
      } else {
        constraint->AddFlag(q3ContactConstraintFlags::eColliding);
      }
    } else {
      if (constraint->HasFlag(q3ContactConstraintFlags::eColliding)) {
        constraint->RemoveFlag(q3ContactConstraintFlags::eColliding);
        constraint->AddFlag(q3ContactConstraintFlags::eWasColliding);
      } else {
        constraint->RemoveFlag(q3ContactConstraintFlags::eWasColliding);
      }
    }
    // }

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
          c->warmStarted = std::max(oldWarmStart, uint8_t(oldWarmStart + 1));
          break;
        }
      }
    }
    ++it;
  }
}

//--------------------------------------------------------------------------------------------------
void q3ContactManager::Render(q3Render *render) const {
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

      if (std::get<0>(m->A)->IsAwake())
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
