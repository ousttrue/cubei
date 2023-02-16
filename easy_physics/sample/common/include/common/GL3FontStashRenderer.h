#pragma once
#include <cuber/gl3/shader.h>
#include <string_view>

class Gl3FontStashRenderer {
  struct FONScontext *fs = nullptr;
  int fontNormal;

  std::shared_ptr<cuber::gl3::ShaderProgram> shader_;

public:
  Gl3FontStashRenderer(const char *name, const char *path);
  ~Gl3FontStashRenderer();
  void Draw(int width, int height, std::string_view title);
};
