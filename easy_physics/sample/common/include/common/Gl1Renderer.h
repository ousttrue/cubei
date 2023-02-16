#pragma once
#include "common.h"

class Gl1Renderer {

public:
  Gl1Renderer();
  ~Gl1Renderer();
  void Begin(int width, int height, const float projection[16],
             const float view[16]);

  void Render(const struct DrawDataSpan &data);

  void RenderShape(EasyPhysics::EpxShapeType type,
                   const EasyPhysics::EpxMatrix4 &transform,
                   const EasyPhysics::EpxVector3 &color);

  void RenderMesh(const EasyPhysics::EpxMatrix4 &transform,
                  const EasyPhysics::EpxVector3 &color, const float *v,
                  const uint16_t *i, uint32_t indexCount, const uint16_t *w,
                  uint32_t wireCount);

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
