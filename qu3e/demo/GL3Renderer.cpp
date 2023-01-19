#include "GL3Renderer.h"
#include "glo.h"
#include <gl/glew.h>
#include <plog/Log.h>
#include <stdint.h>
#include <string_view>
#include <vector>

struct Float3 {
  float x, y, z;
};

struct Vertex {
  Float3 position;
  Float3 normal;
  Float3 color;
};

glo::VertexLayout layouts[]{
    glo::VertexLayout{
        .stride = 36,
        .offset = 0,
        .components = 3,
    },
    glo::VertexLayout{
        .stride = 36,
        .offset = 12,
        .components = 3,
    },
    glo::VertexLayout{
        .stride = 36,
        .offset = 24,
        .components = 3,
    },
};

auto VS = R"(#version 400
layout (std140) uniform VertexUbo { 
	mat4 uVP;
};
layout (location=0) in vec3 aPos;
layout (location=1) in vec3 aNormal;
layout (location=2) in vec3 aColor;
out vec3 FragPos;
out vec3 Normal;
out vec3 Color;
void main()
{
    gl_Position = uVP * vec4(aPos, 1.0);
    gl_PointSize=5;
    FragPos = aPos;
    Normal = aNormal;
    Color = aColor;
}
)";
struct VertexUbo {
  DirectX::XMFLOAT4X4 uVP;
};

auto FS = R"(#version 400
layout (std140) uniform FragmentUbo {
  vec4 uLightAmbient;
  vec4 uLightDiffuse;
  vec4 uLightSpecular;
  vec4 uLightPosition;
};
in vec3 FragPos;
in vec3 Normal;
in vec3 Color;
out vec4 FragColor;
void main()
{
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(uLightPosition.xyz - FragPos);
    float shading = Normal==vec3(0, 0, 0) ? 1 : max(dot(norm, lightDir), 0.0);
    FragColor = vec4(Color * shading, 1.0);
}
)";

class GL3RendererImpl {
  uint32_t program_ = 0;
  glo::VBO vbo_;
  glo::VAO vao_;
  std::vector<Vertex> triangles_;
  std::vector<Vertex> lines_;
  std::vector<Vertex> points_;

  glo::UBO<VertexUbo> vertexUbo_ = {0};
  glo::UBO<Light> fragmentUbo_ = {1};

public:
  GL3RendererImpl() {
    auto errorMessage = [](const char *msg) {
      //
      PLOG_FATAL << msg;
    };
    program_ = glo::BuildShader(VS, FS, errorMessage);
    vbo_.Initialize(sizeof(triangles_), nullptr);
    uint32_t vbos[] = {
        vbo_.vbo_,
        vbo_.vbo_,
        vbo_.vbo_,
    };
    vao_.Initialize(layouts, vbos);
    fragmentUbo_.Initialize();
    vertexUbo_.Initialize();
  }
  ~GL3RendererImpl() {}
  void Clear() {
    triangles_.clear();
    lines_.clear();
    points_.clear();
  }
  void PushTriangle(const Vertex &v0, const Vertex &v1, const Vertex &v2) {
    triangles_.push_back(v0);
    triangles_.push_back(v1);
    triangles_.push_back(v2);
  }
  void PushLine(const Vertex &v0, const Vertex &v1) {
    lines_.push_back(v0);
    lines_.push_back(v1);
  }
  void PushPoint(const Vertex &v0) { points_.push_back(v0); }
  void Render(const float m[16]) {
    // upload
    vertexUbo_.value_.uVP = *((const DirectX::XMFLOAT4X4 *)m);
    vertexUbo_.Upload();
    fragmentUbo_.Upload();

    // render
    glUseProgram(program_);
    vertexUbo_.Bind(program_, "VertexUbo");
    fragmentUbo_.Bind(program_, "FragmentUbo");
    {
      vbo_.Initialize(triangles_.size() * sizeof(Vertex), triangles_.data());
      vao_.Render(triangles_.size(), GL_TRIANGLES);
    }
    {
      vbo_.Initialize(lines_.size() * sizeof(Vertex), lines_.data());
      vao_.Render(lines_.size(), GL_LINES);
    }
    {
      glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
      vbo_.Upload(points_.size() * sizeof(Vertex), points_.data());
      vao_.Render(lines_.size(), GL_POINTS);
      glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);
    }
    glUseProgram(0);
  }
};

GL3Renderer::GL3Renderer() : impl_(new GL3RendererImpl) {}
GL3Renderer::~GL3Renderer() {}
void GL3Renderer::SetPenColor(float r, float g, float b, float a) {
  r_ = r;
  g_ = g;
  b_ = b;
}
void GL3Renderer::SetPenPosition(float x, float y, float z) {
  x_ = x;
  y_ = y;
  z_ = z;
}
void GL3Renderer::SetScale(float sx, float sy, float sz) {
  // glPointSize((float)sx);
  sx_ = sx;
  sy_ = sy;
  sz_ = sz;
}
void GL3Renderer::Line(float x, float y, float z) {
  impl_->PushLine(
      Vertex{
          .position{x_, y_, z_},
          .normal{0, 0, 0},
          .color{r_, g_, b_},
      },
      Vertex{
          .position{x, y, z},
          .normal{0, 0, 0},
          .color{r_, g_, b_},
      });
  x_ = x;
  y_ = y;
  z_ = z;
}
void GL3Renderer::Triangle(float x1, float y1, float z1, float x2, float y2,
                           float z2, float x3, float y3, float z3) {
  Float3 rgb = {0.2f, 0.4f, 0.7};
  impl_->PushTriangle(
      Vertex{
          .position{x1, y1, z1},
          .normal{.x = nx_, .y = ny_, .z = nz_},
          .color = rgb,
      },
      Vertex{
          .position{x2, y2, z2},
          .normal{.x = nx_, .y = ny_, .z = nz_},
          .color = rgb,
      },
      Vertex{
          .position{x3, y3, z3},
          .normal{.x = nx_, .y = ny_, .z = nz_},
          .color = rgb,
      });
}
void GL3Renderer::SetTriNormal(float x, float y, float z) {
  nx_ = x;
  ny_ = y;
  nz_ = z;
}
void GL3Renderer::Point() {
  impl_->PushPoint(Vertex{
      .position{x_, y_, z_},
      .normal{0, 0, 0},
      .color{r_, g_, b_},
  });
}
void GL3Renderer::BeginFrame(int width, int height, const float *projection,
                             const float *view) {
  glViewport(0, 0, width, height);
  glClearColor(0.0f, 0.2f, 0.0f, 0.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  impl_->Clear();
}
void GL3Renderer::EndFrame(const float *projection, const float *view) {
  glEnable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glCullFace(GL_BACK);
  glFrontFace(GL_CCW);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);

  auto m_view = DirectX::XMLoadFloat4x4((const DirectX::XMFLOAT4X4 *)view);
  auto m_proj =
      DirectX::XMLoadFloat4x4((const DirectX::XMFLOAT4X4 *)projection);
  auto m = m_view * m_proj;

  DirectX::XMFLOAT4X4 mvp;
  DirectX::XMStoreFloat4x4(&mvp, m);
  impl_->Render(&mvp._11);
}
