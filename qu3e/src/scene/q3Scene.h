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
#include "../dynamics/q3ContactManager.h"
#include "../dynamics/q3Island.h"
#include <functional>
#include <list>
#include <stdio.h>

class q3Body;
struct q3BodyDef;
class q3Scene {
  bool m_newBox = false;
  q3Island m_island;
  std::list<q3Body *> m_bodyList;

public:
  q3ContactManager m_contactManager;

  std::function<void(q3Body *)> OnBodyAdd;
  std::function<void(q3Body *)> OnBodyRemove;

  ~q3Scene();
  std::list<q3Body *>::const_iterator begin() const {
    return m_bodyList.begin();
  }
  std::list<q3Body *>::const_iterator end() const { return m_bodyList.end(); }

  // Run the simulation forward in time by dt (fixed timestep). Variable
  // timestep is not supported.
  void Step(const q3Env &env);

private:
  // Construct a new rigid body. The BodyDef can be reused at the user's
  // discretion, as no reference to the BodyDef is kept.
  q3Body *CreateBody(const q3BodyDef &def);

public:
  q3Body *CreateBody(const q3BodyDef &def,
                     class q3ContactManager *contactManager);

  // Adds a box to this body. Boxes are all defined in local space
  // of their owning body. Boxes cannot be defined relative to one
  // another. The body will recalculate its mass values. No contacts
  // will be created until the next q3Scene::Step( ) call.
  const q3Box *AddBox(q3Body *body, const struct q3BoxDef &def);

  // Frees a body, removes all shapes associated with the body and frees
  // all shapes and contacts associated and attached to this body.
  void RemoveBody(q3Body *body);
  void RemoveAllBodies();

  // Dump all rigid bodies and shapes into a log file. The log can be
  // used as C++ code to re-create an initial scene setup. Contacts
  // are *not* logged, meaning any cached resolution solutions will
  // not be saved to the log file. This means the log file will be most
  // accurate when dumped upon scene initialization, instead of mid-
  // simulation.
  void Dump(FILE *file, const q3Env &env) const;
};
