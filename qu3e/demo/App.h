#pragma once
#include "Camera.h"
#include "debug/q3Render.h"

struct Light {
  float ambient[4] = {1.0f, 1.0f, 1.0f, 0.5f};
  float diffuse[4] = {0.2f, 0.4f, 0.7f, 1.0f};
  float specular[4] = {1.0f, 1.0f, 1.0f, 1.0f};
};

class App {
  Camera camera_ = {};
  Light light_ = {};
  class q3Render *renderer_ = nullptr;

public:
  App(struct GLFWwindow *window);
  ~App();
  void KeyDown(char key);
  void Frame(int w, int h);
};
