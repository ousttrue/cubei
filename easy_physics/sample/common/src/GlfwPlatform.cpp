#include <GLFW/glfw3.h>
#include <common/GlfwPlatform.h>
#include <common/PhysicsSceneSelector.h>
#include <common/ScreenCamera.h>
#include <stdexcept>

void GlfwPlatform::key_callback(GLFWwindow *window, int key, int scancode,
                                int action, int mods) {
  auto self = (GlfwPlatform *)glfwGetWindowUserPointer(window);
  if (action == GLFW_PRESS) {
    auto found = self->m_onKeyPress.find(key);
    if (found != self->m_onKeyPress.end()) {
      found->second(self->x_, self->y_, self->width_, self->height_);
    }
    self->m_isDown.insert(key);
  } else if (action == GLFW_RELEASE) {
    self->m_isDown.erase(key);
  }
}

void GlfwPlatform::cursor_position_callback(GLFWwindow *window, double xpos,
                                            double ypos) {
  auto self = (GlfwPlatform *)glfwGetWindowUserPointer(window);
  self->x_ = xpos - self->width_ / 2;
  self->y_ = -ypos + self->height_ / 2;
}

GlfwPlatform::GlfwPlatform(const char *title) {
  if (!glfwInit()) {
    throw std::runtime_error("glfwInit");
  }
  window_ = glfwCreateWindow(640, 480, title, NULL, NULL);
  if (!window_) {
    throw std::runtime_error("glfwCreateWindow");
  }
  glfwMakeContextCurrent(window_);

  glfwSetWindowUserPointer(window_, this);
  glfwSetKeyCallback(window_, key_callback);
  glfwSetCursorPosCallback(window_, cursor_position_callback);
}

GlfwPlatform::~GlfwPlatform() { glfwTerminate(); }

bool GlfwPlatform::NewFrame(int *x, int *y, int *width, int *height) {
  if (glfwWindowShouldClose(window_)) {
    return false;
  }
  glfwPollEvents();
  for (auto key : m_isDown) {
    auto found = m_onKeyIsDown.find(key);
    if (found != m_onKeyIsDown.end()) {
      found->second(x_, y_, width_, height_);
    }
  }
  glfwGetFramebufferSize(window_, width, height);
  width_ = *width;
  height_ = *height;
  *x = x_;
  *y = y_;
  return true;
}

void GlfwPlatform::EndFrame() { glfwSwapBuffers(window_); }

void GlfwPlatform::BindCamera(ScreenCamera &camera) {
  // keybind: view
  OnKeyIsDown(GLFW_KEY_UP, [&camera](int x, int y, int width, int height) {
    auto [angX, angY, r] = camera.GetViewAngle();
    angX -= 0.05f;
    if (angX < -1.4f)
      angX = -1.4f;
    if (angX > -0.01f)
      angX = -0.01f;
    camera.SetViewAngle(angX, angY, r);
  });
  OnKeyIsDown(GLFW_KEY_DOWN, [&camera](int x, int y, int width, int height) {
    auto [angX, angY, r] = camera.GetViewAngle();
    angX += 0.05f;
    if (angX < -1.4f)
      angX = -1.4f;
    if (angX > -0.01f)
      angX = -0.01f;
    camera.SetViewAngle(angX, angY, r);
  });
  OnKeyIsDown(GLFW_KEY_LEFT, [&camera](int x, int y, int width, int height) {
    auto [angX, angY, r] = camera.GetViewAngle();
    angY -= 0.05f;
    camera.SetViewAngle(angX, angY, r);
  });
  OnKeyIsDown(GLFW_KEY_RIGHT, [&camera](int x, int y, int width, int height) {
    auto [angX, angY, r] = camera.GetViewAngle();
    angY += 0.05f;
    camera.SetViewAngle(angX, angY, r);
  });
  OnKeyIsDown(GLFW_KEY_PAGE_DOWN,
              [&camera](int x, int y, int width, int height) {
                auto [angX, angY, r] = camera.GetViewAngle();
                r *= 1.1f;
                if (r > 500.0f)
                  r = 500.0f;
                camera.SetViewAngle(angX, angY, r);
              });
  OnKeyIsDown(GLFW_KEY_PAGE_UP, [&camera](int x, int y, int width, int height) {
    auto [angX, angY, r] = camera.GetViewAngle();
    r *= 0.9f;
    if (r < 1.0f)
      r = 1.0f;
    camera.SetViewAngle(angX, angY, r);
  });
}

void GlfwPlatform::BindSelector(struct PhysicsSceneSelector &selector,
                                class ScreenCamera &camera) {
  OnKeyPress(GLFW_KEY_F1, [&selector](int x, int y, int width, int height) {
    selector.Reset();
  });
  OnKeyPress(GLFW_KEY_F2, [&selector](int x, int y, int width, int height) {
    selector.Next();
  });
  OnKeyPress(GLFW_KEY_F3, [&selector](int x, int y, int width, int height) {
    selector.Toggle();
  });
  OnKeyPress(GLFW_KEY_F4, [&selector](int x, int y, int width, int height) {
    selector.Run();
  });
  OnKeyPress(GLFW_KEY_SPACE,
             [&camera, &selector](int x, int y, int width, int height) {
               selector.Fire(camera, x, y, width, height);
             });
}
