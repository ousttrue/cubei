//--------------------------------------------------------------------------------------------------
/**
@file	q3Box.h

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
#include "../math/q3Mat3.h"
#include "../math/q3Raycast.h"
#include "../math/q3Transform.h"
#include "../math/q3AABB.h"
#include <optional>

struct q3MassData {
  q3Mat3 inertia;
  q3Vec3 center;
  float mass;
};

struct q3BoxDef {
  q3Transform m_tx = {};
  q3Vec3 m_e = {};
  float m_friction = 0.4f;
  float m_restitution = 0.2f;
  float m_density = 1.0f;
  bool m_sensor = false;
  void Dump(FILE *file) const;
};

class q3Box {
  q3BoxDef def_;
  int broadPhaseIndex_ = -1;

public:
  q3Box(const q3BoxDef &def);
  // def
  const q3Transform &Local() const { return def_.m_tx; }
  const q3Vec3 &Extent() const { return def_.m_e; }
  float Friction() const { return def_.m_friction; }
  float Restitution() const { return def_.m_restitution; }
  bool Sensor() const { return def_.m_sensor; }

  void SetBroadPhaseIndex(int index) { broadPhaseIndex_ = index; }
  int BroadPhaseIndex() const { return broadPhaseIndex_; }
  bool TestPoint(const q3Transform &tx, const q3Vec3 &p) const;
  bool Raycast(const q3Transform &tx, q3RaycastData *raycast) const;
  q3AABB ComputeAABB(const q3Transform &tx) const;
  std::optional<q3MassData> ComputeMass() const;
  void Render(const q3Transform &tx, bool awake, class q3Render *render) const;
  void Dump(FILE *file, int index) const;
};
