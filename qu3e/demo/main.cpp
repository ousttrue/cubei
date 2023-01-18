#include "App.h"
#include <GLFW/glfw3.h>
#include <iostream>
#include <stdio.h>

// static int g_mouseX = -1;
// static int g_mouseY = -1;
// static bool g_mouseLeftDown = false;
// static bool g_mouseRightDown = false;

static void glfw_error_callback(int error, const char *description) {
  fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

void mouse_button_callback(GLFWwindow *window, int button, int action,
                           int mods) {
  // if (action == GLFW_PRESS) {
  //   switch (button) {
  //   case GLFW_MOUSE_BUTTON_LEFT: {
  //     g_mouseLeftDown = true;
  //     DemosMouseLeftDown(g_mouseX, g_mouseY);
  //   } break;
  //   case GLFW_MOUSE_BUTTON_RIGHT: {
  //     g_mouseRightDown = true;
  //   } break;
  //   }
  // } else if (action == GLFW_RELEASE) {
  //   switch (button) {
  //   case GLFW_MOUSE_BUTTON_LEFT: {
  //     g_mouseLeftDown = false;
  //   } break;
  //   case GLFW_MOUSE_BUTTON_RIGHT: {
  //     g_mouseRightDown = false;
  //   } break;
  //   }
  // }
}

static void cursor_position_callback(GLFWwindow *window, double xpos,
                                     double ypos) {
  // g_mouseX = static_cast<int>(xpos);
  // g_mouseY = static_cast<int>(ypos);
  // ImGuiIO &io = ImGui::GetIO();
  // io.MousePos = ImVec2((float)g_mouseX, (float)g_mouseY);
}

static void key_callback(GLFWwindow *window, int key, int scancode, int action,
                         int mods) {
  auto app = (App *)glfwGetWindowUserPointer(window);
  if (action == GLFW_PRESS) {
    switch (key) {
    case 27:
      glfwDestroyWindow(window);
      break;

    default:
      app->KeyDown(key);
    }
  }
}

int main(int argc, char **argv) {
  // Setup window
  glfwSetErrorCallback(glfw_error_callback);

  // Starting width / height of the window
  const uint32_t kWindowWidth = 1000;
  const uint32_t kWindowHeight = 600;

  // Initialize GLUT
  if (!glfwInit()) {
    return -1;
  }

  // Setup the window
  auto window = glfwCreateWindow(kWindowWidth, kWindowHeight,
                                 "qu3e Physics by Randy Gaul", NULL, NULL);
  if (!window) {
    glfwTerminate();
    return -1;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);
  std::cout << glGetString(GL_VERSION) << std::endl;

  // glfwSetWindowSizeCallback(window, window_size_callback);
  glfwSetKeyCallback(window, key_callback);
  glfwSetMouseButtonCallback(window, mouse_button_callback);
  glfwSetCursorPosCallback(window, cursor_position_callback);

  App app(window);
  glfwSetWindowUserPointer(window, &app);

  while (!glfwWindowShouldClose(window)) { // message loop
    glfwPollEvents();
    int w, h;
    glfwGetFramebufferSize(window, &w, &h);

    app.Frame(w, h);

    glfwSwapBuffers(window);
  }

  glfwTerminate();
}
