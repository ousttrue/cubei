#include "physics_func.h"
#include <GLFW/glfw3.h>
#include <common/FontStashRenderer.h>
#include <common/Gl1Renderer.h>
#include <common/GraphicsScene.h>
#include <common/common.h>
#include <format>
#include <functional>
#include <gl/GL.h>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#define SAMPLE_NAME "01_basic"

static bool g_simulating = false;
static int g_sceneId = 0;
PhysicsState g_state = {};
static std::shared_ptr<PhysicsScene> g_scene;

class GlfwPlatform {
  GLFWwindow *window_ = nullptr;
  int x_, y_, width_, height_;
  using KeyCallback = std::function<void(int x, int y, int width, int height)>;
  std::unordered_set<uint32_t> m_isDown;
  std::unordered_map<uint32_t, KeyCallback> m_onKeyIsDown;
  std::unordered_map<uint32_t, KeyCallback> m_onKeyPress;

  static void key_callback(GLFWwindow *window, int key, int scancode,
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

  static void cursor_position_callback(GLFWwindow *window, double xpos,
                                       double ypos) {
    auto self = (GlfwPlatform *)glfwGetWindowUserPointer(window);
    self->x_ = xpos - self->width_ / 2;
    self->y_ = -ypos + self->height_ / 2;
  }

public:
  GlfwPlatform() {
    if (!glfwInit()) {
      throw std::runtime_error("glfwInit");
    }
    window_ = glfwCreateWindow(640, 480, SAMPLE_NAME, NULL, NULL);
    if (!window_) {
      throw std::runtime_error("glfwCreateWindow");
    }
    glfwMakeContextCurrent(window_);

    glfwSetWindowUserPointer(window_, this);
    glfwSetKeyCallback(window_, key_callback);
    glfwSetCursorPosCallback(window_, cursor_position_callback);
  }

  ~GlfwPlatform() { glfwTerminate(); }

  bool NewFrame(int *x, int *y, int *width, int *height) {
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

  void EndFrame() { glfwSwapBuffers(window_); }

  void OnKeyIsDown(uint32_t key, const KeyCallback &callback) {
    m_onKeyIsDown[key] = callback;
  }
  void OnKeyPress(uint32_t key, const KeyCallback &callback) {
    m_onKeyPress[key] = callback;
  }

  void Bind(Gl1Renderer &renderer, GraphicsScene &scene) {
    // keybind: view
    OnKeyIsDown(GLFW_KEY_UP, [&scene](int x, int y, int width, int height) {
      auto [angX, angY, r] = scene.GetViewAngle();
      angX -= 0.05f;
      if (angX < -1.4f)
        angX = -1.4f;
      if (angX > -0.01f)
        angX = -0.01f;
      scene.SetViewAngle(angX, angY, r);
    });
    OnKeyIsDown(GLFW_KEY_DOWN, [&scene](int x, int y, int width, int height) {
      auto [angX, angY, r] = scene.GetViewAngle();
      angX += 0.05f;
      if (angX < -1.4f)
        angX = -1.4f;
      if (angX > -0.01f)
        angX = -0.01f;
      scene.SetViewAngle(angX, angY, r);
    });
    OnKeyIsDown(GLFW_KEY_LEFT, [&scene](int x, int y, int width, int height) {
      auto [angX, angY, r] = scene.GetViewAngle();
      angY -= 0.05f;
      scene.SetViewAngle(angX, angY, r);
    });
    OnKeyIsDown(GLFW_KEY_RIGHT, [&scene](int x, int y, int width, int height) {
      auto [angX, angY, r] = scene.GetViewAngle();
      angY += 0.05f;
      scene.SetViewAngle(angX, angY, r);
    });
    OnKeyIsDown(GLFW_KEY_PAGE_DOWN,
                [&scene](int x, int y, int width, int height) {
                  auto [angX, angY, r] = scene.GetViewAngle();
                  r *= 1.1f;
                  if (r > 500.0f)
                    r = 500.0f;
                  scene.SetViewAngle(angX, angY, r);
                });
    OnKeyIsDown(GLFW_KEY_PAGE_UP,
                [&scene](int x, int y, int width, int height) {
                  auto [angX, angY, r] = scene.GetViewAngle();
                  r *= 0.9f;
                  if (r < 1.0f)
                    r = 1.0f;
                  scene.SetViewAngle(angX, angY, r);
                });

    OnKeyPress(GLFW_KEY_F1, [&renderer](int x, int y, int width, int height) {
      renderer.ReleaseMeshAll();
      physicsCreateScene(g_sceneId, &renderer);
      g_state.Clear();
    });
    OnKeyPress(GLFW_KEY_F2, [&renderer](int x, int y, int width, int height) {
      renderer.ReleaseMeshAll();
      g_scene = physicsCreateScene(++g_sceneId, &renderer);
      g_state.Clear();
    });
    OnKeyPress(GLFW_KEY_F3, [](int x, int y, int width, int height) {
      g_simulating = !g_simulating;
    });
    OnKeyPress(GLFW_KEY_F4, [](int x, int y, int width, int height) {
      g_simulating = true;
    });
    OnKeyPress(GLFW_KEY_SPACE, [&scene](int x, int y, int width, int height) {
      auto wp1 =
          scene.GetWorldPosition({(float)x, (float)y, 0.0f}, width, height);
      auto wp2 =
          scene.GetWorldPosition({(float)x, (float)y, 1.0f}, width, height);
      g_scene->PhysicsFire(wp1, normalize(wp2 - wp1) * 50.0f);
    });
  }
};

// int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR
// lpCmdLine, int nCmdShow)
int main(int argc, char **argv) {

  GlfwPlatform platform;
  Gl1Renderer renderer;
  GraphicsScene scene;
  platform.Bind(renderer, scene);

  FontStashRenderer stash(
      "sans", argc > 1
                  ? argv[1]
                  : "subprojects/fontstash/example/DroidSerif-Regular.ttf");

  g_scene = physicsCreateScene(g_sceneId, &renderer);
  g_state.Clear();

  int x, y, width, height;
  while (platform.NewFrame(&x, &y, &width, &height)) {
    // update
    if (g_simulating) {
      g_scene->Simulate(g_state);
    }
    auto [projection, view] = scene.UpdateProjectionView(width, height);

    // render
    renderer.Begin(width, height, projection, view);
    PhysicsRender(*g_scene, g_state, &renderer, nullptr);

    renderer.Debug2dBegin(width, height);
    stash.Draw(x, y, width, height, g_scene->title_);
    renderer.Debug2dEnd();

    platform.EndFrame();
  }

  return 0;
}
