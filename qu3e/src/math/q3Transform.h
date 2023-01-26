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

#ifndef Q3TRANSFORM_H
#define Q3TRANSFORM_H

#include "../math/q3Geometry.h"

struct q3Transform {
  q3Vec3 position = {};
  q3Mat3 rotation = {};

  q3Transform Inversed() const {
    auto inv = q3Transpose(rotation);
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

//--------------------------------------------------------------------------------------------------
// q3Transform
//--------------------------------------------------------------------------------------------------
inline const q3HalfSpace q3Mul(const q3Transform &tx, const q3HalfSpace &p) {
  q3Vec3 origin = p.Origin();
  origin = tx * origin;
  q3Vec3 normal = tx.rotation * p.normal;

  return q3HalfSpace(normal, q3Dot(origin, normal));
}

//--------------------------------------------------------------------------------------------------
inline const q3Vec3 q3MulT(const q3Mat3 &r, const q3Vec3 &v) {
  return q3Transpose(r) * v;
}

//--------------------------------------------------------------------------------------------------
inline const q3Mat3 q3MulT(const q3Mat3 &r, const q3Mat3 &q) {
  return q3Transpose(r) * q;
}

//--------------------------------------------------------------------------------------------------
inline const q3Transform q3MulT(const q3Transform &t, const q3Transform &u) {
  q3Transform v;
  v.rotation = q3MulT(t.rotation, u.rotation);
  v.position = q3MulT(t.rotation, u.position - t.position);
  return v;
}

//--------------------------------------------------------------------------------------------------
inline const q3HalfSpace q3MulT(const q3Transform &tx, const q3HalfSpace &p) {
  q3Vec3 origin = p.normal * p.distance;
  origin = tx.Inversed() * origin;
  q3Vec3 n = q3MulT(tx.rotation, p.normal);
  return q3HalfSpace(n, q3Dot(origin, n));
}

//--------------------------------------------------------------------------------------------------
inline const q3Transform q3Inverse(const q3Transform &tx) {
  q3Transform inverted;
  inverted.rotation = q3Transpose(tx.rotation);
  inverted.position = inverted.rotation * -tx.position;
  return inverted;
}

#endif // Q3TRANSFORM_H
