#pragma once

class App {
public:
  App(struct GLFWwindow *window);
  ~App();
  void KeyDown(char key);
  void Frame(int w, int h);
};
