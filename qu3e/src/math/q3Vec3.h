//--------------------------------------------------------------------------------------------------
/**
@file	q3Vec3.h

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

#ifndef Q3VEC3_H
#define Q3VEC3_H

#include "../common/q3Types.h"
#include <cmath>
#include <stdexcept>

float q3Abs(float a);
float q3Min(float a, float b);
float q3Max(float a, float b);

//--------------------------------------------------------------------------------------------------
// q3Vec3
//--------------------------------------------------------------------------------------------------
struct q3Vec3 {
  float x = 0;
  float y = 0;
  float z = 0;
  void Set(float _x, float _y, float _z) {
    x = _x;
    y = _y;
    z = _z;
  }
  void SetAll(float a) {
    x = a;
    y = a;
    z = a;
  }
  q3Vec3 &operator+=(const q3Vec3 &rhs) {
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    return *this;
  }
  q3Vec3 &operator-=(const q3Vec3 &rhs) {
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
    return *this;
  }
  q3Vec3 &operator*=(float f) {
    x *= f;
    y *= f;
    z *= f;
    return *this;
  }
  q3Vec3 &operator/=(float f) {
    x /= f;
    y /= f;
    z /= f;

    return *this;
  }

  float &operator[](uint32_t i) {
    switch (i) {
    case 0:
      return x;
    case 1:
      return y;
    case 2:
      return z;
    default:
      throw std::runtime_error("index out of range");
    }
  }
  float operator[](uint32_t i) const {
    switch (i) {
    case 0:
      return x;
    case 1:
      return y;
    case 2:
      return z;
    default:
      throw std::runtime_error("index out of range");
    }
  }
  q3Vec3 operator-(void) const { return {-x, -y, -z}; }
  const q3Vec3 operator+(const q3Vec3 &rhs) const {
    return {x + rhs.x, y + rhs.y, z + rhs.z};
  }
  const q3Vec3 operator-(const q3Vec3 &rhs) const {
    return {x - rhs.x, y - rhs.y, z - rhs.z};
  }
  const q3Vec3 operator*(float f) const { return {x * f, y * f, z * f}; }
  const q3Vec3 operator/(float f) const { return {x / f, y / f, z / f}; }
};

//--------------------------------------------------------------------------------------------------
// q3Vec3
//--------------------------------------------------------------------------------------------------
inline void q3Identity(q3Vec3 &v) { v.Set(float(0.0), float(0.0), float(0.0)); }

//--------------------------------------------------------------------------------------------------
inline const q3Vec3 q3Mul(const q3Vec3 &a, const q3Vec3 &b) {
  return {a.x * b.x, a.y * b.y, a.z * b.z};
}

//--------------------------------------------------------------------------------------------------
inline float q3Dot(const q3Vec3 &a, const q3Vec3 &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

//--------------------------------------------------------------------------------------------------
inline const q3Vec3 q3Cross(const q3Vec3 &a, const q3Vec3 &b) {
  return {
      (a.y * b.z) - (b.y * a.z),
      (b.x * a.z) - (a.x * b.z),
      (a.x * b.y) - (b.x * a.y),
  };
}

//--------------------------------------------------------------------------------------------------
inline float q3Length(const q3Vec3 &v) {
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

//--------------------------------------------------------------------------------------------------
inline float q3LengthSq(const q3Vec3 &v) {
  return v.x * v.x + v.y * v.y + v.z * v.z;
}

//--------------------------------------------------------------------------------------------------
inline const q3Vec3 q3Normalize(const q3Vec3 &v) {
  float l = q3Length(v);

  if (l != float(0.0)) {
    float inv = float(1.0) / l;
    return v * inv;
  }

  return v;
}

//--------------------------------------------------------------------------------------------------
inline float q3Distance(const q3Vec3 &a, const q3Vec3 &b) {
  float xp = a.x - b.x;
  float yp = a.y - b.y;
  float zp = a.z - b.z;

  return std::sqrt(xp * xp + yp * yp + zp * zp);
}

//--------------------------------------------------------------------------------------------------
inline float q3DistanceSq(const q3Vec3 &a, const q3Vec3 &b) {
  float xp = a.x - b.x;
  float yp = a.y - b.y;
  float zp = a.z - b.z;

  return xp * xp + yp * yp + zp * zp;
}

//--------------------------------------------------------------------------------------------------
inline const q3Vec3 q3Abs(const q3Vec3 &v) {
  return {q3Abs(v.x), q3Abs(v.y), q3Abs(v.z)};
}

//--------------------------------------------------------------------------------------------------
inline const q3Vec3 q3Min(const q3Vec3 &a, const q3Vec3 &b) {
  return {q3Min(a.x, b.x), q3Min(a.y, b.y), q3Min(a.z, b.z)};
}

//--------------------------------------------------------------------------------------------------
inline const q3Vec3 q3Max(const q3Vec3 &a, const q3Vec3 &b) {
  return {q3Max(a.x, b.x), q3Max(a.y, b.y), q3Max(a.z, b.z)};
}

//--------------------------------------------------------------------------------------------------
inline const float q3MinPerElem(const q3Vec3 &a) {
  return q3Min(a.x, q3Min(a.y, a.z));
}

//--------------------------------------------------------------------------------------------------
inline const float q3MaxPerElem(const q3Vec3 &a) {
  return q3Max(a.x, q3Max(a.y, a.z));
}

inline const q3Vec3 operator*(float f, const q3Vec3 &rhs) {
  return {rhs.x * f, rhs.y * f, rhs.z * f};
}

#endif // Q3VEC3_H
