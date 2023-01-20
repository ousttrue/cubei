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

#ifndef Q3CONTACTMANAGER_H
#define Q3CONTACTMANAGER_H

#include "../broadphase/q3BroadPhase.h"

#include <list>
//--------------------------------------------------------------------------------------------------
// q3ContactManager
//--------------------------------------------------------------------------------------------------
struct q3ContactConstraint;

// This listener is used to gather information about two shapes colliding. This
// can be used for game logic and sounds. Physics objects created in these
// callbacks will not be reported until the following frame. These callbacks
// can be called frequently, so make them efficient.
class q3ContactListener {
public:
  virtual ~q3ContactListener() {}

  virtual void BeginContact(const q3ContactConstraint *contact) = 0;
  virtual void EndContact(const q3ContactConstraint *contact) = 0;
};

struct q3Box;
class q3Body;
class q3Render;
class q3Stack;

// This class represents general queries for points, AABBs and Raycasting.
// ReportShape is called the moment a valid shape is found. The return
// value of ReportShape controls whether to continue or stop the query.
// By returning only true, all shapes that fulfill the query will be re-
// ported.
class q3QueryCallback {
public:
  virtual ~q3QueryCallback() {}

  virtual bool ReportShape(q3Box *box) = 0;
};

class q3ContactManager {
  std::list<q3ContactConstraint *> m_contactList;
  q3ContactListener *m_contactListener = nullptr;

public:
  q3BroadPhase m_broadphase;
  q3ContactManager();

  size_t ContactCount() const { return m_contactList.size(); }

  // Add a new contact constraint for a pair of objects
  // unless the contact constraint already exists
  void AddContact(q3Box *A, q3Box *B);

  // Has broadphase find all contacts and call AddContact on the
  // ContactManager for each pair found
  void FindNewContacts(void);

  // Remove a specific contact
  std::list<q3ContactConstraint *>::iterator
  RemoveContact(q3ContactConstraint *contact);

  // Remove all contacts from a body
  void RemoveContactsFromBody(q3Body *body);
  void RemoveFromBroadphase(q3Body *body);

  // Remove contacts without broadphase overlap
  // Solves contact manifolds
  void TestCollisions(bool newBox);
  static void SolveCollision(void *param);

  void RenderContacts(q3Render *debugDrawer) const;

  // Sets the listener to report collision start/end. Provides the user
  // with a pointer to an q3ContactConstraint. The q3ContactConstraint
  // holds pointers to the two shapes involved in a collision, and the
  // two bodies connected to each shape. The q3ContactListener will be
  // called very often, so it is recommended for the funciton to be very
  // efficient. Provide a NULL pointer to remove the previously set
  // listener.
  void SetContactListener(q3ContactListener *listener) {
    m_contactListener = listener;
  }

  // Query the world to find any shapes that can potentially intersect
  // the provided AABB. This works by querying the broadphase with an
  // AAABB -- only *potential* intersections are reported. Perhaps the
  // user might use lmDistance as fine-grained collision detection.
  void QueryAABB(q3QueryCallback *cb, const q3AABB &aabb) const;

  // Query the world to find any shapes intersecting a world space point.
  void QueryPoint(q3QueryCallback *cb, const q3Vec3 &point) const;

  // Query the world to find any shapes intersecting a ray.
  void RayCast(q3QueryCallback *cb, q3RaycastData &rayCast) const;
};

#endif // Q3CONTACTMANAGER_H
