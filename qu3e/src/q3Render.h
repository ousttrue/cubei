//--------------------------------------------------------------------------------------------------
/**
@file	q3Render.h

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
#include "math/q3AABB.h"
#include "math/q3Transform.h"

class q3Render {
public:
  virtual ~q3Render() {}
  virtual void BeginFrame(int width, int height, const float *projection,
                          const float *view) {}
  virtual void EndFrame(const float *projection, const float *view) {}

  virtual void SetPenColor(float r, float g, float b, float a = 1.0f) = 0;
  virtual void SetScale(float sx, float sy, float sz) = 0;

  // Draw a point with the scale from SetScale
  virtual void Point() = 0;

  virtual void LineAABB(const q3AABB &aabb);

  virtual void SetPenPosition(float x, float y, float z) = 0;

  // Render a line from pen position to this point.
  // Sets the pen position to the new point.
  virtual void Line(float x, float y, float z) = 0;

  virtual void Cube(const q3Transform &world, const q3Vec3 extent);

protected:
  virtual void SetTriNormal(float x, float y, float z){}

  // Render a triangle with the normal set by SetTriNormal.
  virtual void Triangle(float x1, float y1, float z1, float x2, float y2,
                        float z2, float x3, float y3, float z3){}
};
