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
  auto contact = std::make_shared<q3ContactConstraint>(A, bodyA, B, bodyB);
  m_contactList.push_back(contact);

  // Connect A
  auto edgeA = ContactEdge(bodyA);
  contact->edgeA = {
      .other = bodyB,
      .constraint = contact,
      .next = edgeA,
      .prev = NULL,
  };
  if (edgeA) {
    edgeA->prev = &contact->edgeA;
  }
  m_edgeMap[bodyA] = &contact->edgeA;

  // Connect B
  auto edgeB = ContactEdge(bodyB);
  contact->edgeB = {
      .other = bodyA,
      .constraint = contact,
      .next = edgeB,
      .prev = NULL,
  };
  if (edgeB) {
    edgeB->prev = &contact->edgeB;
  }
  m_edgeMap[bodyB] = &contact->edgeB;

  bodyA->SetToAwake();
  bodyB->SetToAwake();
}

std::list<q3ContactConstraintPtr>::iterator
q3ContactManager::RemoveContact(q3ContactConstraintPtr contact) {
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
