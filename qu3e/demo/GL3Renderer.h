#pragma once
#include "GLTypes.h"
#include <q3.h>
#include <stdint.h>

class GL3Renderer : public q3Render {
  float x_, y_, z_;
  float r_, g_, b_;
  float sx_, sy_, sz_;

  Camera camera_ = {};

  class GL3RendererImpl *impl_ = nullptr;

public:
  GL3Renderer();
  ~GL3Renderer();
  void SetPenColor(float r, float g, float b, float a = 1.0f) override;
  void SetPenPosition(float x, float y, float z) override;
  void SetScale(float sx, float sy, float sz) override;
  void Line(float x, float y, float z) override;
  void Cube(const q3Transform &world, const q3Vec3 extent) override;
  void Point() override;

  void BeginFrame(int width, int height, const float *projection, const float *view) override;
  void EndFrame(const float *projection, const float *view) override;
};
