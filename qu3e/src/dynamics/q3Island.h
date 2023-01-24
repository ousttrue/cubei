//--------------------------------------------------------------------------------------------------
/**
@file	q3Island.h

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

#ifndef Q3ISLAND_H
#define Q3ISLAND_H

#include "../math/q3Geometry.h"
#include "../math/q3Math.h"
#include "../scene/q3Env.h"
#include "q3ContactSolver.h"
#include <list>
#include <vector>
//--------------------------------------------------------------------------------------------------
// q3Island
//--------------------------------------------------------------------------------------------------
class q3Island {
  std::vector<class q3Body *> m_bodies;
  std::vector<class q3Body *> m_stack;

public:
  q3Env m_env;
  std::vector<q3VelocityState> m_velocities;
  std::vector<struct q3ContactConstraint *> m_contacts;
  std::vector<struct q3ContactConstraintState> m_contactStates;

  // Run the simulation forward in time by dt (fixed timestep). Variable
  // timestep is not supported.
  void Step(const q3Env &env, class q3Scene *scene,
            class q3BroadPhase *broadPhase,
            class q3ContactManager *contactManager);

private:
  void Solve(const q3Env &env);
  void Add(class q3Body *body);
  void Add(struct q3ContactConstraint *contact);
  void Initialize();
};

#endif // Q3ISLAND_H
