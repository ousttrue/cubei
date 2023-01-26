#pragma once
#include "../scene/q3Box.h"
#include "q3ContactEdge.h"
#include "q3Magnifold.h"

enum class q3ContactConstraintFlags {
  eNone = 0,
  eColliding = 0x00000001,    // Set when contact collides during a step
  eWasColliding = 0x00000002, // Set when two objects stop colliding
  eIsland = 0x00000004,       // For internal marking during island forming
};

struct q3ContactConstraint {
  q3Box *A;
  q3Box *B;
  q3Body *bodyA;
  q3Body *bodyB;
  q3ContactEdge edgeA;
  q3ContactEdge edgeB;

  float friction;
  float restitution;

  q3Manifold manifold;

  q3ContactConstraintFlags m_flags = {};

  bool HasFlag(q3ContactConstraintFlags flag) const {
    return ((int)m_flags & (int)flag) != 0;
  }
  void AddFlag(q3ContactConstraintFlags flag) {
    m_flags = (q3ContactConstraintFlags)((int)m_flags | (int)flag);
  }
  void RemoveFlag(q3ContactConstraintFlags flag) {
    m_flags = (q3ContactConstraintFlags)((int)m_flags & ~(int)flag);
  }
  bool Test(const std::function<bool(q3Box *, q3Box *)> &testOverlap);
};
