//--------------------------------------------------------------------------------------------------
/**
@file	q3Geometry.h

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

#ifndef Q3GEOMETRY_H
#define Q3GEOMETRY_H

#include "../math/q3Math.h"

//--------------------------------------------------------------------------------------------------
// q3AABB
//--------------------------------------------------------------------------------------------------
struct q3AABB {
  q3Vec3 min;
  q3Vec3 max;

  bool Contains(const q3AABB &other) const;
  bool Contains(const q3Vec3 &point) const;
  float SurfaceArea() const;

  bool IsOverlapped(const q3AABB &b) const {
    if (max.x < b.min.x || min.x > b.max.x)
      return false;

    if (max.y < b.min.y || min.y > b.max.y)
      return false;

    if (max.z < b.min.z || min.z > b.max.z)
      return false;

    return true;
  }
};

//--------------------------------------------------------------------------------------------------
// q3HalfSpace
//--------------------------------------------------------------------------------------------------
struct q3HalfSpace {
  q3HalfSpace();
  q3HalfSpace(const q3Vec3 &normal, float distance);

  void Set(const q3Vec3 &a, const q3Vec3 &b, const q3Vec3 &c);
  void Set(const q3Vec3 &n, const q3Vec3 &p);
  const q3Vec3 Origin() const;
  float Distance(const q3Vec3 &p) const;
  const q3Vec3 Projected(const q3Vec3 &p) const;

  q3Vec3 normal;
  float distance;
};

//--------------------------------------------------------------------------------------------------
// q3RaycastData
//--------------------------------------------------------------------------------------------------
struct q3RaycastData {
  q3Vec3 start; // Beginning point of the ray
  q3Vec3 dir;   // Direction of the ray (normalized)
  float t;      // Time specifying ray endpoint

  float toi;     // Solved time of impact
  q3Vec3 normal; // Surface normal at impact

  void Set(const q3Vec3 &startPoint, const q3Vec3 &direction,
           float endPointTime);

  // Uses toi, start and dir to compute the point at toi. Should
  // only be called after a raycast has been conducted with a
  // return value of true.
  const q3Vec3 GetImpactPoint() const;
};

//--------------------------------------------------------------------------------------------------
// Common
//--------------------------------------------------------------------------------------------------
// http://box2d.org/2014/02/computing-a-basis/
inline void q3ComputeBasis(const q3Vec3 &a, q3Vec3 *__restrict b,
                           q3Vec3 *__restrict c) {
  // Suppose vector a has all equal components and is a unit vector: a = (s, s,
  // s) Then 3*s*s = 1, s = sqrt(1/3) = 0.57735027. This means that at least one
  // component of a unit vector must be greater or equal to 0.57735027. Can use
  // SIMD select operation.

  if (std::abs(a.x) >= float(0.57735027))
    b->Set(a.y, -a.x, float(0.0));
  else
    b->Set(float(0.0), a.z, -a.y);

  *b = b->Normalized();
  *c = q3Cross(a, *b);
}

//--------------------------------------------------------------------------------------------------
// q3AABB
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
inline bool q3AABB::Contains(const q3AABB &other) const {
  return min.x <= other.min.x && min.y <= other.min.y && min.z <= other.min.z &&
         max.x >= other.max.x && max.y >= other.max.y && max.z >= other.max.z;
}

//--------------------------------------------------------------------------------------------------
inline bool q3AABB::Contains(const q3Vec3 &point) const {
  return min.x <= point.x && min.y <= point.y && min.z <= point.z &&
         max.x >= point.x && max.y >= point.y && max.z >= point.z;
}

//--------------------------------------------------------------------------------------------------
inline float q3AABB::SurfaceArea() const {
  float x = max.x - min.x;
  float y = max.y - min.y;
  float z = max.z - min.z;

  return float(2.0) * (x * y + x * z + y * z);
}

//--------------------------------------------------------------------------------------------------
inline const q3AABB q3Combine(const q3AABB &a, const q3AABB &b) {
  return {
      .min =
          {
              std::min(a.min.x, b.min.x),
              std::min(a.min.y, b.min.y),
              std::min(a.min.z, b.min.z),
          },
      .max =
          {
              std::max(a.max.x, b.max.x),
              std::max(a.max.y, b.max.y),
              std::max(a.max.z, b.max.z),
          },
  };
}

//--------------------------------------------------------------------------------------------------
// q3RaycastData
//--------------------------------------------------------------------------------------------------
inline void q3RaycastData::Set(const q3Vec3 &startPoint,
                               const q3Vec3 &direction, float endPointTime) {
  start = startPoint;
  dir = direction;
  t = endPointTime;
}

//--------------------------------------------------------------------------------------------------
inline const q3Vec3 q3RaycastData::GetImpactPoint() const {
  return q3Vec3(start + dir * toi);
}

#endif // Q3GEOMETRY_H
