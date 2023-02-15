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
#include <vector>

struct MeshBuff {
  float *vtx;
  float *nml;
  int numVtx;
  unsigned short *idx;
  unsigned short *wireIdx;
  int numIdx;

  MeshBuff() : vtx(0), nml(0), numVtx(0), idx(0), wireIdx(0), numIdx(0) {}
};
class Gl1Renderer {

  std::vector<MeshBuff> s_meshBuff;

public:
  Gl1Renderer();
  ~Gl1Renderer();
  void Begin(int width, int height, const float projection[16],
             const float view[16]);

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
};

uint64_t createRenderMesh(class Gl1Renderer *renderer,
                          EasyPhysics::EpxConvexMesh *convexMesh);
