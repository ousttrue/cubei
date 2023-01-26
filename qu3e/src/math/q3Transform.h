//--------------------------------------------------------------------------------------------------
/**
@file	q3Transform.h

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
#include "../math/q3Geometry.h"

struct q3Transform {
  q3Vec3 position = {};
  q3Mat3 rotation = {};

  q3Transform Inversed() const {
    auto inv = rotation.Transposed();
    return {
        inv * -position,
        inv,
    };
  }

  q3Vec3 operator*(const q3Vec3 &v) const {
    return q3Vec3(rotation * v + position);
  }

  q3Transform operator*(const q3Transform &u) const {
    q3Transform v;
    v.rotation = rotation * u.rotation;
    v.position = rotation * u.position + position;
    return v;
  }
};
