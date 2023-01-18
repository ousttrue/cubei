#include "App.h"
#include <GLFW/glfw3.h>
#include <iostream>
#include <plog/Appenders/ConsoleAppender.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Init.h>
#include <plog/Log.h>
#include <stdio.h>

bool g_exit = false;
static void glfw_error_callback(int error, const char *description) {
  if (g_exit) {
    return;
  }
  fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

int main(int argc, char **argv) {
  static plog::ConsoleAppender<plog::TxtFormatter> consoleAppender;
  plog::init(plog::debug, &consoleAppender);

  // Setup window
  glfwSetErrorCallback(glfw_error_callback);

  // Starting width / height of the window
  const uint32_t kWindowWidth = 1000;
  const uint32_t kWindowHeight = 600;

  // Initialize GLUT
  if (!glfwInit()) {
    return 1;
  }

  // Setup the window
  auto window = glfwCreateWindow(kWindowWidth, kWindowHeight,
                                 "qu3e Physics by Randy Gaul", NULL, NULL);
  if (!window) {
    glfwTerminate();
    return 2;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);
  std::cout << glGetString(GL_VERSION) << std::endl;

  App app(window);
  glfwSetWindowUserPointer(window, &app);

  while (!glfwWindowShouldClose(window)) { // message loop
    glfwPollEvents();
    int w, h;
    glfwGetFramebufferSize(window, &w, &h);

    app.Frame(w, h);

    glfwSwapBuffers(window);
  }

  g_exit = true;
  glfwDestroyWindow(window);
  glfwTerminate();
}
