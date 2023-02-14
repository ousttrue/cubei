/*
        Copyright (c) 2012 Hiroshi Matsuike

        This software is provided 'as-is', without any express or implied
        warranty. In no event will the authors be held liable for any damages
        arising from the use of this software.

        Permission is granted to anyone to use this software for any purpose,
        including commercial applications, and to alter it and redistribute it
        freely, subject to the following restrictions:

        1. The origin of this software must not be misrepresented; you must not
        claim that you wrote the original software. If you use this software
        in a product, an acknowledgment in the product documentation would be
        appreciated but is not required.

        2. Altered source versions must be plainly marked as such, and must not
   be misrepresented as being the original software.

        3. This notice may not be removed or altered from any source
   distribution.
*/

#pragma once
#include "common.h"

class Gl1Renderer {

public:
  Gl1Renderer();
  ~Gl1Renderer();
  void Begin(int width, int height);

  int InitMesh(const float *vtx, unsigned int vtxStrideBytes, const float *nml,
               unsigned int nmlStrideBytes, const unsigned short *tri,
               unsigned int triStrideBytes, int numVtx, int numTri);

  void ReleaseMeshAll();

  void Mesh(const EasyPhysics::EpxTransform3 &transform,
            const EasyPhysics::EpxVector3 &color, int meshId);

  ///////////////////////////////////////////////////////////////////////////////
  // Debug Drawing

  void DebugBegin();
  void DebugEnd();

  void DebugPoint(const EasyPhysics::EpxVector3 &position,
                  const EasyPhysics::EpxVector3 &color);

  void DebugLine(const EasyPhysics::EpxVector3 &position1,
                 const EasyPhysics::EpxVector3 &position2,
                 const EasyPhysics::EpxVector3 &color);

  void DebugAabb(const EasyPhysics::EpxVector3 &center,
                 const EasyPhysics::EpxVector3 &extent,
                 const EasyPhysics::EpxVector3 &color);

  ///////////////////////////////////////////////////////////////////////////////
  // 2D

  void Debug2dBegin(int width, int height);
  void Debug2dEnd();

  ///////////////////////////////////////////////////////////////////////////////
  // Render Parameter

  void GetViewAngle(float &angleX, float &angleY, float &radius);
  void SetViewAngle(float angleX, float angleY, float radius);

  void GetViewTarget(EasyPhysics::EpxVector3 &targetPos);
  void SetViewTarget(const EasyPhysics::EpxVector3 &targetPos);
  void GetViewRadius(float &radius);
  void SetViewRadius(float radius);
  void LookAtTarget(const EasyPhysics::EpxVector3 &viewPos,
                    const EasyPhysics::EpxVector3 &viewTarget);

  EasyPhysics::EpxVector3
  GetWorldPosition(const EasyPhysics::EpxVector3 &screenPos, int width, int height);
  EasyPhysics::EpxVector3
  GetScreenPosition(const EasyPhysics::EpxVector3 &worldPos, int width, int height);
};

uint64_t createRenderMesh(class Gl1Renderer *renderer,
                          EasyPhysics::EpxConvexMesh *convexMesh);
