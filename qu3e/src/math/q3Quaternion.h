//--------------------------------------------------------------------------------------------------
/**
@file	q3Quaternion.h

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
#include "q3Vec3.h"

struct q3Mat3;
struct q3Quaternion {
  float x;
  float y;
  float z;
  float w;

  static q3Quaternion FromAxisAngle(const q3Vec3 &axis, float radians);
  std::tuple<q3Vec3, float> ToAxisAngle() const;
  void Integrate(const q3Vec3 &dv, float dt);
  q3Quaternion operator*(const q3Quaternion &rhs) const;
  q3Quaternion &operator*=(const q3Quaternion &rhs);
  q3Mat3 ToMat3() const;
  q3Quaternion q3Normalized() const {
    float x = this->x;
    float y = this->y;
    float z = this->z;
    float w = this->w;
    float d = w * w + x * x + y * y + z * z;
    if (d == 0) {
      w = float(1.0);
    }
    d = float(1.0) / std::sqrt(d);
    if (d > float(1.0e-8)) {
      x *= d;
      y *= d;
      z *= d;
      w *= d;
    }
    return q3Quaternion{x, y, z, w};
  }
};