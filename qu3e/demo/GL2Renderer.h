#pragma once
#include "GLTypes.h"
#include <q3.h>

class GL2Renderer : public q3Render {
  float x_, y_, z_;
  float sx_, sy_, sz_;
  float nx_, ny_, nz_;

  Light light_ = {};

public:
  GL2Renderer();
  ~GL2Renderer();
  void SetPenColor(float r, float g, float b, float a = 1.0f) override;
  void SetPenPosition(float x, float y, float z) override;
  void SetScale(float sx, float sy, float sz) override;
  void Line(float x, float y, float z) override;
  void Triangle(float x1, float y1, float z1, float x2, float y2, float z2,
                float x3, float y3, float z3) override;
  void SetTriNormal(float x, float y, float z) override;
  void Point() override;

  void BeginFrame(int width, int height, const float *projection, const float *view) override;
};
