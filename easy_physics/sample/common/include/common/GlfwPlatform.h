#pragma once
#include <functional>
#include <unordered_map>
#include <unordered_set>

class GlfwPlatform {
  struct GLFWwindow *window_ = nullptr;
  int x_, y_, width_, height_;
  using KeyCallback = std::function<void(int x, int y, int width, int height)>;
  std::unordered_set<uint32_t> m_isDown;
  std::unordered_map<uint32_t, KeyCallback> m_onKeyIsDown;
  std::unordered_map<uint32_t, KeyCallback> m_onKeyPress;

  static void key_callback(GLFWwindow *window, int key, int scancode,
                           int action, int mods);

  static void cursor_position_callback(GLFWwindow *window, double xpos,
                                       double ypos);

public:
  GlfwPlatform(const char *title);
  ~GlfwPlatform();
  bool NewFrame(int *x, int *y, int *width, int *height);
  void EndFrame();
  void OnKeyIsDown(uint32_t key, const KeyCallback &callback) {
    m_onKeyIsDown[key] = callback;
  }
  void OnKeyPress(uint32_t key, const KeyCallback &callback) {
    m_onKeyPress[key] = callback;
  }
  void BindCamera(class ScreenCamera &camera);
  void BindSelector(struct PhysicsSceneSelector &selector,
                    class ScreenCamera &camera);
};
