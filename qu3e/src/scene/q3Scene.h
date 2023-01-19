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

#pragma once
#include "../common/q3Memory.h"
#include "../common/q3Settings.h"
#include "../dynamics/q3ContactManager.h"
#include "../dynamics/q3Island.h"
#include <list>
#include <stdio.h>

//--------------------------------------------------------------------------------------------------
// q3Scene
//--------------------------------------------------------------------------------------------------
class q3Body;
struct q3BodyDef;
struct q3ContactConstraint;
class q3Render;

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

class q3Scene {
  q3ContactManager m_contactManager;
  q3PagedAllocator m_boxAllocator;
  std::list<q3Body *> m_bodyList;
  bool m_newBox = false;
  q3Island m_island;
  friend class q3Body;

public:
  q3Scene();
  ~q3Scene();

  // Run the simulation forward in time by dt (fixed timestep). Variable
  // timestep is not supported.
  void Step(const q3Env &env);

  // Construct a new rigid body. The BodyDef can be reused at the user's
  // discretion, as no reference to the BodyDef is kept.
  q3Body *CreateBody(const q3BodyDef &def);

  // Frees a body, removes all shapes associated with the body and frees
  // all shapes and contacts associated and attached to this body.
  void RemoveBody(q3Body *body);
  void RemoveAllBodies();

  // Render the scene with an interpolated time between the last frame and
  // the current simulation step.
  void Render(q3Render *render) const;

  // Removes all bodies from the scene.
  void Shutdown();

  // Sets the listener to report collision start/end. Provides the user
  // with a pointer to an q3ContactConstraint. The q3ContactConstraint
  // holds pointers to the two shapes involved in a collision, and the
  // two bodies connected to each shape. The q3ContactListener will be
  // called very often, so it is recommended for the funciton to be very
  // efficient. Provide a NULL pointer to remove the previously set
  // listener.
  void SetContactListener(q3ContactListener *listener);

  // Query the world to find any shapes that can potentially intersect
  // the provided AABB. This works by querying the broadphase with an
  // AAABB -- only *potential* intersections are reported. Perhaps the
  // user might use lmDistance as fine-grained collision detection.
  void QueryAABB(q3QueryCallback *cb, const q3AABB &aabb) const;

  // Query the world to find any shapes intersecting a world space point.
  void QueryPoint(q3QueryCallback *cb, const q3Vec3 &point) const;

  // Query the world to find any shapes intersecting a ray.
  void RayCast(q3QueryCallback *cb, q3RaycastData &rayCast) const;

  // Dump all rigid bodies and shapes into a log file. The log can be
  // used as C++ code to re-create an initial scene setup. Contacts
  // are *not* logged, meaning any cached resolution solutions will
  // not be saved to the log file. This means the log file will be most
  // accurate when dumped upon scene initialization, instead of mid-
  // simulation.
  void Dump(FILE *file, const q3Env &env) const;
};
