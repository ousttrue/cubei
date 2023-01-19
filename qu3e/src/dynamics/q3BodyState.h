#pragma once
#include "../math//q3Mat3.h"

struct q3BodyState
{
  q3Vec3 m_worldCenter;
  q3Mat3 m_invInertiaWorld;
  float m_invMass;
  int m_islandIndex;
};
