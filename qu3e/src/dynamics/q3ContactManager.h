//--------------------------------------------------------------------------------------------------
/**
@file	q3ContactManager.h

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

#pragma once
#include "q3ContactConstraint.h"
#include <functional>
#include <list>
#include <memory>
#include <unordered_map>
class q3Box;
class q3Body;
class q3Render;
class q3Stack;

class q3ContactManager {
  std::list<q3ContactConstraintPtr> m_contactList;

  std::unordered_map<class q3Body *, struct q3ContactEdge *> m_edgeMap;

public:
  q3ContactManager();
  std::list<q3ContactConstraintPtr>::iterator begin() {
    return m_contactList.begin();
  }
  std::list<q3ContactConstraintPtr>::iterator end() {
    return m_contactList.end();
  }
  size_t ContactCount() const { return m_contactList.size(); }

  struct q3ContactEdge *ContactEdge(q3Body *body) {
    auto found = m_edgeMap.find(body);
    if (found == m_edgeMap.end()) {
      return nullptr;
    }
    return found->second;
  }

  // Add a new contact constraint for a pair of objects
  // unless the contact constraint already exists
  void AddContact(q3Body *bodyA, q3Box *A, q3Body *bodyB, q3Box *B);

  // Remove a specific contact
  std::list<q3ContactConstraintPtr>::iterator
  RemoveContact(q3ContactConstraintPtr contact);

  // Remove all contacts from a body
  void RemoveContactsFromBody(q3Body *body);

  void Render(q3Render *debugDrawer) const;
};
