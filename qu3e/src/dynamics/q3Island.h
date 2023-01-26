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

#pragma once
#include "../math/q3Math.h"
#include "../scene/q3Env.h"
#include "q3ContactConstraintState.h"
#include "q3ContactSolver.h"
#include <list>
#include <vector>

class q3Island {
  std::vector<class q3Body *> m_stack;

public:
  std::vector<std::tuple<class q3Body *, q3VelocityState>> m_bodies;
  std::vector<
      std::tuple<struct q3ContactConstraint *, q3ContactConstraintState>>
      m_constraints;

  // Run the simulation forward in time by dt (fixed timestep). Variable
  // timestep is not supported.
  void Step(const q3Env &env, class q3Scene *scene,
            class q3BroadPhase *broadPhase,
            class q3ContactManager *contactManager);

private:
  void Process(const q3Env &env, q3Body *seed,
               class q3ContactManager *contactManager);
  void Solve(const q3Env &env);
  void AddBody(class q3Body *body) {
    body->SetIslandIndex(m_bodies.size());
    m_bodies.push_back({body, {}});
  }

  void AddConstraint(struct q3ContactConstraint *constraint) {
    m_constraints.push_back({constraint, {}});
  }

  void Initialize();
};
