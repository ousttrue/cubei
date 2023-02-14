#pragma once
#include <string_view>

class FontStashRenderer {
  struct FONScontext *fs = nullptr;
  int fontNormal;

public:
  FontStashRenderer(const char *name, const char *path);
  ~FontStashRenderer();
  void Draw(int width, int height, std::string_view title);
};
