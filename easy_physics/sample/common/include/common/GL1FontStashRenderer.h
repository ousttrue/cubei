#pragma once
#include <string_view>

class Gl1FontStashRenderer {
  struct FONScontext *fs = nullptr;
  int fontNormal;

public:
  Gl1FontStashRenderer(const char *name, const char *path);
  ~Gl1FontStashRenderer();
  void Draw(int width, int height, std::string_view title);
};
