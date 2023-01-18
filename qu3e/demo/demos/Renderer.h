#pragma once
#include <q3.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#endif
#include <gl/GL.h>

struct Light {
  float ambient[4] = {1.0f, 1.0f, 1.0f, 0.5f};
  float diffuse[4] = {0.2f, 0.4f, 0.7f, 1.0f};
  float specular[4] = {1.0f, 1.0f, 1.0f, 1.0f};
};

struct Camera {
  float position[3] = {0.0f, 5.0f, 20.0f};
  float target[3] = {0.0f, 0.0f, 0.0f};
};

class Renderer : public q3Render {
  float x_, y_, z_;
  float sx_, sy_, sz_;
  float nx_, ny_, nz_;

  Camera camera_ = {};
  Light light_ = {};

public:
  Renderer();
  ~Renderer();
  void SetPenColor(float r, float g, float b, float a = 1.0f) override;
  void SetPenPosition(float x, float y, float z) override;
  void SetScale(float sx, float sy, float sz) override;
  void Line(float x, float y, float z) override;
  void Triangle(float x1, float y1, float z1, float x2, float y2, float z2,
                float x3, float y3, float z3) override;
  void SetTriNormal(float x, float y, float z) override;
  void Point() override;

  void BeginFrame(int width, int height);
};