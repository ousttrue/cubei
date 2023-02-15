#pragma once
#include <common/common.h>
#include <tuple>

using namespace EasyPhysics;

class GraphicsScene {
  EpxMatrix4 s_pMat, s_vMat;
  EpxVector3 s_viewPos, s_lightPos;

  // initalize parameters
  float s_lightRadius = 40.0f;
  float s_lightRadX = -0.6f;
  float s_lightRadY = 0.6f;
  float s_viewRadius = 20.0f;
  float s_viewRadX = -0.01f;
  float s_viewRadY = 0.0f;
  float s_viewHeight = 1.0f;
  EpxVector3 s_viewTgt{0.0f, 1.0f, 0.0f};

public:
  GraphicsScene(const GraphicsScene &) = delete;
  GraphicsScene &operator=(const GraphicsScene &) = delete;
  GraphicsScene();
  ~GraphicsScene();

  std::tuple<const float *, const float *> UpdateProjectionView(int width,
                                                                int height);

  std::tuple<float, float, float> GetViewAngle() const {
    return {s_viewRadX, s_viewRadY, s_viewRadius};
  }

  void SetViewAngle(float angleX, float angleY, float radius) {
    s_viewRadX = angleX;
    s_viewRadY = angleY;
    s_viewRadius = radius;
    s_viewPos = EpxMatrix3::rotationY(s_viewRadY) *
                EpxMatrix3::rotationX(s_viewRadX) *
                EpxVector3(0, 0, s_viewRadius);
  }

public:
  EasyPhysics::EpxVector3
  GetWorldPosition(const EasyPhysics::EpxVector3 &screenPos, int width,
                   int height) const;
  EasyPhysics::EpxVector3
  GetScreenPosition(const EasyPhysics::EpxVector3 &worldPos, int width,
                    int height) const;
};
