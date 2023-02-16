#include <DirectXMath.h>
#include <Windows.h>
#include <common/Gl3FontStashRenderer.h>
#include <common/common.h>
#include <format>
#include <iostream>
#include <stdexcept>

#include <gl/glew.h>

#define FONTSTASH_IMPLEMENTATION
#include "fontstash.h"
#define GLFONTSTASH_IMPLEMENTATION
#include "gl3corefontstash.h"

namespace gl3fons {

// static auto glsl_version = "#version 310 es\nprecision highp float;";
static auto glsl_version = "#version 400";

// https://github.com/suikki/sdf_text_sample
static const char *VS = R"(
uniform mat4 modelView;
uniform mat4 projection;

in vec2 vertexPosition;
in vec2 vertexTexCoord;
in vec4 vertexColor;

out vec2 interpolatedTexCoord;
out vec4 interpolatedColor;

void main() {
  interpolatedColor = vertexColor;
  interpolatedTexCoord = vertexTexCoord;
  gl_Position = projection * modelView * vec4(vertexPosition, 0, 1);
}
)";

static const char *FS = R"(
uniform sampler2D diffuse;

in vec2 interpolatedTexCoord;
in vec4 interpolatedColor;
out vec4 FragColor;

void main() {
  float alpha = texture2D(diffuse, interpolatedTexCoord).a;
  vec4 textColor = interpolatedColor / 255.0;
  FragColor = vec4(textColor.rgb * textColor.a, textColor.a) * alpha; // Premultiplied alpha.
}
)";

std::string_view vss[] = {glsl_version, VS};
std::string_view fss[] = {glsl_version, FS};

} // namespace gl3fons

static unsigned int glfonsFloatRGBA(float r, float g, float b, float a) {
  return glfonsRGBA((uint8_t)(r * 255), (uint8_t)(g * 255), (uint8_t)(b * 255),
                    (uint8_t)(a * 255));
}

Gl3FontStashRenderer::Gl3FontStashRenderer(const char *name, const char *path)
    : fontNormal(FONS_INVALID) {
  fs = glfonsCreate(512, 512, FONS_ZERO_TOPLEFT);
  if (fs == NULL) {
    throw std::runtime_error("Could not create stash.");
  }

  fontNormal = fonsAddFont(fs, name, path);
  if (fontNormal == FONS_INVALID) {
    throw std::runtime_error("Could not add font normal.");
  }

  shader_ = cuber::gl3::ShaderProgram::Create(
      [](auto msg) { std::cout << msg << std::endl; }, gl3fons::vss,
      gl3fons::fss);
}
Gl3FontStashRenderer::~Gl3FontStashRenderer() { glfonsDelete(fs); }

void Gl3FontStashRenderer::Draw(int width, int height, std::string_view title) {

  //
  // Enable blending.
  //
  glEnable(GL_BLEND);
  glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);

  shader_->Bind();

  DirectX::XMFLOAT4X4 projection;
  DirectX::XMStoreFloat4x4(
      &projection,
      DirectX::XMMatrixOrthographicOffCenterRH(0, width, height, 0, -1, 1));
  shader_->SetUniformMatrix([](auto msg) { std::cout << msg << std::endl; },
                            "projection", projection);
  DirectX::XMFLOAT4X4 view;
  DirectX::XMStoreFloat4x4(&view, DirectX::XMMatrixIdentity());
  shader_->SetUniformMatrix([](auto msg) { std::cout << msg << std::endl; },
                            "modelView", view);

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

  // fonsSetColor(fs, white);
  // fonsDrawText(fs, sx, (sy += dh),
  //              std::format("{}, {}: {} x {}", x, y, width, height).c_str(),
  //              nullptr);

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
