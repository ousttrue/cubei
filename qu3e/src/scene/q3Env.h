#pragma once
#include "../math/q3Vec3.h"

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
