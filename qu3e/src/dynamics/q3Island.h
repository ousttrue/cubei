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

#include "q3ContactSolver.h"
#include "../common/q3Geometry.h"
#include "../common/q3Settings.h"
#include "../math/q3Math.h"
#include <list>
#include <vector>
//--------------------------------------------------------------------------------------------------
// q3Island
//--------------------------------------------------------------------------------------------------
struct q3VelocityState {
  q3Vec3 w;
  q3Vec3 v;
};

struct q3Env {
  float m_dt = 1.0f / 60.0f;
  q3Vec3 m_gravity = {float(0.0), float(-9.8), float(0.0)};
  int m_iterations = 20;
  bool m_allowSleep = true;
  bool m_enableFriction = true;

  // Increasing the iteration count increases the CPU cost of simulating
  // Scene.Step(). Decreasing the iterations makes the simulation less
  // realistic (convergent). A good iteration number range is 5 to 20.
  // Only positive numbers are accepted. Non-positive and negative
  // inputs set the iteration count to 1.
  void SetIterations(int iterations) { m_iterations = q3Max(1, iterations); }

  // Enables or disables rigid body sleeping. Sleeping is an effective CPU
  // optimization where bodies are put to sleep if they don't move much.
  // Sleeping bodies sit in memory without being updated, until the are
  // touched by something that wakes them up. The default is enabled.
  void SetAllowSleep(bool allowSleep) {
    m_allowSleep = allowSleep;
    if (!allowSleep) {
      // for (auto body : m_bodyList)
      //   body->SetToAwake();
    }
  }

  // Friction occurs when two rigid bodies have shapes that slide along one
  // another. The friction force resists this sliding motion.
  void SetEnableFriction(bool enabled) { m_enableFriction = enabled; }

  // Gets and sets the global gravity vector used during integration
  const q3Vec3 GetGravity() const { return m_gravity; }
  void SetGravity(const q3Vec3 &gravity) { m_gravity = gravity; }
};

class q3Island {
  std::vector<class q3Body *> m_bodies;
  std::vector<class q3Body *> m_stack;

public:
  q3Env m_env;
  std::vector<q3VelocityState> m_velocities;
  std::vector<struct q3ContactConstraint *> m_contacts;
  std::vector<struct q3ContactConstraintState> m_contactStates;
  void Process(const std::list<class q3Body *> &bodyList, size_t contactCount,
               const q3Env &env);

private:
  void Solve(const q3Env &env);
  void Add(class q3Body *body);
  void Add(struct q3ContactConstraint *contact);
  void Initialize();
};

#endif // Q3ISLAND_H
