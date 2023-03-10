//--------------------------------------------------------------------------------------------------
/**
@file	q3Mat3.h

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

struct q3Mat3 {
  q3Vec3 ex = {1, 0, 0};
  q3Vec3 ey = {0, 1, 0};
  q3Vec3 ez = {0, 0, 1};

  void Set(float a, float b, float c, float d, float e, float f, float g,
           float h, float i);
  void Set(const q3Vec3 &axis, float angle);
  void SetRows(const q3Vec3 &x, const q3Vec3 &y, const q3Vec3 &z);

  q3Mat3 &operator=(const q3Mat3 &rhs);
  q3Mat3 &operator*=(const q3Mat3 &rhs);
  q3Mat3 &operator*=(float f);
  q3Mat3 &operator+=(const q3Mat3 &rhs);
  q3Mat3 &operator-=(const q3Mat3 &rhs);

  q3Vec3 &operator[](uint32_t index);
  const q3Vec3 &operator[](uint32_t index) const;
  const q3Vec3 Column0() const;
  const q3Vec3 Column1() const;
  const q3Vec3 Column2() const;

  const q3Vec3 operator*(const q3Vec3 &rhs) const;
  const q3Mat3 operator*(const q3Mat3 &rhs) const;
  const q3Mat3 operator*(float f) const;
  const q3Mat3 operator+(const q3Mat3 &rhs) const;
  const q3Mat3 operator-(const q3Mat3 &rhs) const;

  q3Mat3 Transposed() const {
    return {
        {ex.x, ey.x, ez.x},
        {ex.y, ey.y, ez.y},
        {ex.z, ey.z, ez.z},
    };
  }

  q3Mat3 Inversed() const{
    q3Vec3 tmp0, tmp1, tmp2;
    float detinv;

    tmp0 = q3Cross(ey, ez);
    tmp1 = q3Cross(ez, ex);
    tmp2 = q3Cross(ex, ey);

    detinv = float(1.0) / q3Dot(ez, tmp2);

    return {
        {tmp0.x * detinv, tmp1.x * detinv, tmp2.x * detinv},
        {tmp0.y * detinv, tmp1.y * detinv, tmp2.y * detinv},
        {tmp0.z * detinv, tmp1.z * detinv, tmp2.z * detinv},
    };
  }

  static q3Mat3 Diagonal(float a) {
    return {
        {float(a), float(0.0), float(0.0)},
        {float(0.0), float(a), float(0.0)},
        {float(0.0), float(0.0), float(a)},
    };
  }

  static q3Mat3 Diagonal(float a, float b, float c) {
    return {
        {float(a), float(0.0), float(0.0)},
        {float(0.0), float(b), float(0.0)},
        {float(0.0), float(0.0), float(c)},
    };
  }

  static q3Mat3 Zero() {
    return {
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0},
    };
  }
};

inline const q3Mat3 q3Rotate(const q3Vec3 &x, const q3Vec3 &y,
                             const q3Vec3 &z) {
  return {x, y, z};
}

inline const q3Mat3 q3OuterProduct(const q3Vec3 &u, const q3Vec3 &v) {
  q3Vec3 a = v * u.x;
  q3Vec3 b = v * u.y;
  q3Vec3 c = v * u.z;

  return {a, b, c};
}

//--------------------------------------------------------------------------------------------------
inline const q3Mat3 q3Covariance(q3Vec3 *points, uint32_t numPoints) {
  float invNumPoints = float(1.0) / float(numPoints);
  q3Vec3 c = q3Vec3{float(0.0), float(0.0), float(0.0)};

  for (uint32_t i = 0; i < numPoints; ++i)
    c += points[i];

  c /= float(numPoints);

  float m00, m11, m22, m01, m02, m12;
  m00 = m11 = m22 = m01 = m02 = m12 = float(0.0);

  for (uint32_t i = 0; i < numPoints; ++i) {
    q3Vec3 p = points[i] - c;

    m00 += p.x * p.x;
    m11 += p.y * p.y;
    m22 += p.z * p.z;
    m01 += p.x * p.y;
    m02 += p.x * p.z;
    m12 += p.y * p.z;
  }

  float m01inv = m01 * invNumPoints;
  float m02inv = m02 * invNumPoints;
  float m12inv = m12 * invNumPoints;

  return {{m00 * invNumPoints, m01inv, m02inv},
          {m01inv, m11 * invNumPoints, m12inv},
          {m02inv, m12inv, m22 * invNumPoints}};
};
