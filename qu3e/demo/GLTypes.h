#pragma once
#include <stdint.h>

struct Light {
  float ambient[4] = {1.0f, 1.0f, 1.0f, 0.5f};
  float diffuse[4] = {0.2f, 0.4f, 0.7f, 1.0f};
  float specular[4] = {1.0f, 1.0f, 1.0f, 1.0f};
};

struct Camera {
  float position[3] = {0.0f, 5.0f, 20.0f};
  float target[3] = {0.0f, 0.0f, 0.0f};
};
