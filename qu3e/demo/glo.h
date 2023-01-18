#pragma once
#include <assert.h>
#include <functional>
#include <gl/glew.h>
#include <span>

// OpenGL Object
namespace glo {

using ErrorMessageHandler = std::function<void(const char *)>;

inline uint32_t CompileShader(std::string_view src, GLenum shader_type,
                              const ErrorMessageHandler &errorMessage) {
  auto shader = glCreateShader(shader_type);
  const char *srcs[] = {src.data()};
  int lens[] = {static_cast<int>(src.size())};
  glShaderSource(shader, 1, srcs, lens);
  glCompileShader(shader);
  GLint isCompiled = 0;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &isCompiled);
  if (isCompiled == GL_FALSE) {
    GLint maxLength = 0;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &maxLength);

    // The maxLength includes the NULL character
    std::vector<GLchar> errorLog(maxLength);
    glGetShaderInfoLog(shader, maxLength, &maxLength, &errorLog[0]);

    // Provide the infolog in whatever manor you deem best.
    errorMessage(errorLog.data());

    // Exit with failure.
    glDeleteShader(shader); // Don't leak the shader.
    return 0;
  }
  return shader;
}

inline uint32_t BuildShader(std::string_view vs_src, std::string_view fs_src,
                            const ErrorMessageHandler &errorMessage) {
  auto vs = CompileShader(vs_src, GL_VERTEX_SHADER, errorMessage);
  if (!vs) {
    return 0;
  }
  auto fs = CompileShader(fs_src, GL_FRAGMENT_SHADER, errorMessage);
  if (!fs) {
    return 0;
  }

  auto program = glCreateProgram();
  glAttachShader(program, vs);
  glAttachShader(program, fs);
  glLinkProgram(program);
  glDeleteShader(vs);
  glDeleteShader(fs);

  GLint isLinked = 0;
  glGetProgramiv(program, GL_LINK_STATUS, &isLinked);
  if (isLinked == GL_FALSE) {
    GLint maxLength = 0;
    glGetProgramiv(program, GL_INFO_LOG_LENGTH, &maxLength);

    // The maxLength includes the NULL character
    std::vector<GLchar> infoLog(maxLength);
    glGetProgramInfoLog(program, maxLength, &maxLength, &infoLog[0]);

    // The program is useless now. So delete it.
    glDeleteProgram(program);

    // Provide the infolog in whatever manner you deem best.
    errorMessage(infoLog.data());

    // Exit with failure.
    return 0;
  }

  return program;
}

struct VBO {
  uint32_t vbo_ = 0;
  VBO(const VBO &) = delete;
  VBO &operator=(const VBO &) = delete;
  VBO() { glCreateBuffers(1, &vbo_); }
  ~VBO() { glDeleteBuffers(1, &vbo_); }
  void Initialize(uint32_t byteSize, const void *data) {
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, byteSize, data,
                 data ? GL_STATIC_DRAW : GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
  }
  void Upload(uint32_t byteSize, const void *data) {}
};

struct VertexLayout {
  uint32_t stride;
  uint64_t offset;
  uint32_t components;
  uint32_t type = GL_FLOAT;
  uint32_t normalized = GL_FALSE;
};
struct VAO {
  uint32_t vao_ = 0;
  VAO(const VAO &) = delete;
  VAO &operator=(const VAO &) = delete;
  VAO() { glCreateVertexArrays(1, &vao_); }
  ~VAO() { glDeleteVertexArrays(1, &vao_); }
  void Initialize(std::span<VertexLayout> layouts, std::span<uint32_t> vbos) {
    glBindVertexArray(vao_);
    assert(layouts.size() == vbos.size());
    auto l = layouts.begin();
    auto v = vbos.begin();
    for (int i = 0; i < std::size(layouts); ++i, ++l, ++v) {
      glBindBuffer(GL_ARRAY_BUFFER, *v);
      glEnableVertexAttribArray(i);
      glVertexAttribPointer(i, l->components, l->type, l->normalized, l->stride,
                            (GLvoid *)l->offset);
    }
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
  }
  void Render(uint32_t draw_count) {}
};

} // namespace glo
