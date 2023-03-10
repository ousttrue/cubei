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

#pragma once
#include <cmath>
#include <stdexcept>

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

  float Length() const { return std::sqrt(x * x + y * y + z * z); }
  float LengthSq() const { return x * x + y * y + z * z; }
  q3Vec3 Normalized() const {
    float l = Length();
    if (l == 0) {
      return *this;
    }

    float inv = float(1.0) / l;
    return *this * inv;
  }
};

//--------------------------------------------------------------------------------------------------
// q3Vec3
//--------------------------------------------------------------------------------------------------
inline const q3Vec3 q3Mul(const q3Vec3 &a, const q3Vec3 &b) {
  return {a.x * b.x, a.y * b.y, a.z * b.z};
}

inline float q3Dot(const q3Vec3 &a, const q3Vec3 &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline const q3Vec3 q3Cross(const q3Vec3 &a, const q3Vec3 &b) {
  return {
      (a.y * b.z) - (b.y * a.z),
      (b.x * a.z) - (a.x * b.z),
      (a.x * b.y) - (b.x * a.y),
  };
}

inline float q3Distance(const q3Vec3 &a, const q3Vec3 &b) {
  float xp = a.x - b.x;
  float yp = a.y - b.y;
  float zp = a.z - b.z;

  return std::sqrt(xp * xp + yp * yp + zp * zp);
}

inline float q3DistanceSq(const q3Vec3 &a, const q3Vec3 &b) {
  float xp = a.x - b.x;
  float yp = a.y - b.y;
  float zp = a.z - b.z;

  return xp * xp + yp * yp + zp * zp;
}

inline const q3Vec3 operator*(float f, const q3Vec3 &rhs) {
  return {rhs.x * f, rhs.y * f, rhs.z * f};
}

inline const q3Vec3 q3Lerp(const q3Vec3 &a, const q3Vec3 &b, float t) {
  return a * (float(1.0) - t) + b * t;
}

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
