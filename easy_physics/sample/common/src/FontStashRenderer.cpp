#include <Windows.h>
#include <common/FontStashRenderer.h>
#include <common/common.h>
#include <format>
#include <stdexcept>

#include <gl/GL.h>

#define FONTSTASH_IMPLEMENTATION
#include "fontstash.h"
#define GLFONTSTASH_IMPLEMENTATION
#include "glfontstash.h"

static unsigned int glfonsFloatRGBA(float r, float g, float b, float a) {
  return glfonsRGBA((uint8_t)(r * 255), (uint8_t)(g * 255), (uint8_t)(b * 255),
                    (uint8_t)(a * 255));
}

FontStashRenderer::FontStashRenderer(const char *name, const char *path)
    : fontNormal(FONS_INVALID) {
  fs = glfonsCreate(512, 512, FONS_ZERO_TOPLEFT);
  if (fs == NULL) {
    throw std::runtime_error("Could not create stash.");
  }

  fontNormal = fonsAddFont(fs, name, path);
  if (fontNormal == FONS_INVALID) {
    throw std::runtime_error("Could not add font normal.");
  }
}
FontStashRenderer::~FontStashRenderer() { glfonsDelete(fs); }

void FontStashRenderer::Draw(int x, int y, int width, int height,
                             std::string_view title) {
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDisable(GL_TEXTURE_2D);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, width, height, 0, -1, 1);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glDisable(GL_DEPTH_TEST);
  glColor4ub(255, 255, 255, 255);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_CULL_FACE);

  auto white = glfonsRGBA(255, 255, 255, 255);
  auto green = glfonsFloatRGBA(0.5f, 1.0f, 0.5f, 1);
  auto blue = glfonsFloatRGBA(0.5f, 0.5f, 1.0f, 1);

  fonsClearState(fs);
  fonsSetSize(fs, 20.0f);
  fonsSetFont(fs, fontNormal);
  float lh;
  fonsVertMetrics(fs, NULL, NULL, &lh);

  EasyPhysics::EpxFloat dh = 20.0f;
  EasyPhysics::EpxFloat sx = width - 200;
  EasyPhysics::EpxFloat sy = 0;

  // 0.5f, 1.0f, 0.5f,
  fonsSetColor(fs, green);
  fonsDrawText(fs, 0, (sy += dh),
               std::format("Easy Physics : {}", title).c_str(), nullptr);

  fonsSetColor(fs, white);
  fonsDrawText(fs, sx, (sy += dh),
               std::format("{}, {}: {} x {}", x, y, width, height).c_str(),
               nullptr);

  // 0.5f, 0.5f, 1.0f,
  fonsSetColor(fs, blue);
  fonsDrawText(fs, sx, (sy += dh), "F1:Reset", nullptr);
  fonsDrawText(fs, sx, (sy += dh), "F2:Next", nullptr);
  fonsDrawText(fs, sx, (sy += dh), "F3:Play/Stop", nullptr);
  fonsDrawText(fs, sx, (sy += dh), "F4:Step", nullptr);
  fonsDrawText(fs, sx, (sy += dh), "Cursor:Rotate view", nullptr);
  fonsDrawText(fs, sx, (sy += dh), "Ins/Del:Move view", nullptr);
  fonsDrawText(fs, sx, (sy += dh), "L-Click:Fire", nullptr);
}
