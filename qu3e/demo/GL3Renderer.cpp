#include "GL3Renderer.h"
#include "glo.h"
#include <gl/glew.h>
#include <plog/Log.h>
#include <stdint.h>
#include <string_view>
#include <vector>

auto VS = R"(#version 110
uniform mat4 MVP;
attribute vec3 vPos;
attribute vec3 vCol;
varying vec3 color;
void main()
{
    gl_Position = MVP * vec4(vPos, 0.0, 1.0);
    color = vCol;
}
)";

auto FS = R"(#version 110
varying vec3 color;
void main()
{
    gl_FragColor = vec4(color, 1.0);
}
)";

glo::VertexLayout layouts[]{
    {},
};

struct Float3 {
  float x, y, z;
};

struct Vertex {
  Float3 position;
  Float3 color;
};

class GL3RendererImpl {
  uint32_t program_ = 0;
  glo::VBO vbo_;
  glo::VAO vao_;
  Vertex vertices_[65535];
  uint32_t drawCount_ = 0;

public:
  GL3RendererImpl() {
    auto errorMessage = [](const char *msg) { PLOG_FATAL << msg; };
    program_ = glo::BuildShader(VS, FS, errorMessage);
    vbo_.Initialize(sizeof(vertices_), nullptr);
    uint32_t vbos[] = {
        vbo_.vbo_,
    };
    vao_.Initialize(layouts, vbos);
  }
  ~GL3RendererImpl() {}
  void Render() {
    // upload vertices
    vbo_.Upload(sizeof(vertices_), vertices_);
    vao_.Render(drawCount_);
  }
};

GL3Renderer::GL3Renderer() : impl_(new GL3RendererImpl) {}
GL3Renderer::~GL3Renderer() {}
void GL3Renderer::SetPenColor(float r, float g, float b, float a) {}
void GL3Renderer::SetPenPosition(float x, float y, float z) {}
void GL3Renderer::SetScale(float sx, float sy, float sz) {}
void GL3Renderer::Line(float x, float y, float z) {}
void GL3Renderer::Triangle(float x1, float y1, float z1, float x2, float y2,
                           float z2, float x3, float y3, float z3) {}
void GL3Renderer::SetTriNormal(float x, float y, float z) {}
void GL3Renderer::Point() {}

void GL3Renderer::BeginFrame(int width, int height) {}
void GL3Renderer::EndFrame() { impl_->Render(); }
