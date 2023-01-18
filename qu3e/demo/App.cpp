#include "App.h"
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#endif
#include "Demo.h"
#include <GLFW/glfw3.h>
#include <gl/GLU.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <iostream>
#include <stdio.h>

static int g_mouseX = -1;
static int g_mouseY = -1;
static bool g_mouseLeftDown = false;
static bool g_mouseRightDown = false;

static void glfw_error_callback(int error, const char *description) {
  fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

void mouse_button_callback(GLFWwindow *window, int button, int action,
                           int mods) {
  if (action == GLFW_PRESS) {
    switch (button) {
    case GLFW_MOUSE_BUTTON_LEFT: {
      g_mouseLeftDown = true;
      DemosMouseLeftDown(g_mouseX, g_mouseY);
    } break;
    case GLFW_MOUSE_BUTTON_RIGHT: {
      g_mouseRightDown = true;
    } break;
    }
  } else if (action == GLFW_RELEASE) {
    switch (button) {
    case GLFW_MOUSE_BUTTON_LEFT: {
      g_mouseLeftDown = false;
    } break;
    case GLFW_MOUSE_BUTTON_RIGHT: {
      g_mouseRightDown = false;
    } break;
    }
  }
}

static void cursor_position_callback(GLFWwindow *window, double xpos,
                                     double ypos) {
  g_mouseX = static_cast<int>(xpos);
  g_mouseY = static_cast<int>(ypos);
  ImGuiIO &io = ImGui::GetIO();
  io.MousePos = ImVec2((float)g_mouseX, (float)g_mouseY);
}

namespace Camera {
float position[3] = {0.0f, 5.0f, 20.0f};
float target[3] = {0.0f, 0.0f, 0.0f};
}; // namespace Camera

namespace Light {
float ambient[4] = {1.0f, 1.0f, 1.0f, 0.5f};
float diffuse[4] = {0.2f, 0.4f, 0.7f, 1.0f};
float specular[4] = {1.0f, 1.0f, 1.0f, 1.0f};
} // namespace Light

static void key_callback(GLFWwindow *window, int key, int scancode, int action,
                         int mods) {
  const float increment = 0.2f;

  if (action == GLFW_PRESS) {
    switch (key) {
    case 27:
      glfwDestroyWindow(window);
      break;
    case 'p':
      DemosTogglePause();
      break;
    case ' ':
      DemosSingleStep();
      break;
    case 'w':
      Camera::position[2] -= increment;
      Camera::target[2] -= increment;
      break;
    case 's':
      Camera::position[2] += increment;
      Camera::target[2] += increment;
      break;
    case 'a':
      Camera::position[0] -= increment;
      Camera::target[0] -= increment;
      break;
    case 'd':
      Camera::position[0] += increment;
      Camera::target[0] += increment;
      break;
    case 'q':
      Camera::position[1] -= increment;
      Camera::target[1] -= increment;
      break;
    case 'e':
      Camera::position[1] += increment;
      Camera::target[1] += increment;
      break;
    default:
      // printf( "%d\n", key );
      break;
    }
    DemosKeyDown(key);
  } else if (action == GLFW_RELEASE) {
    DemosKeyUp(key);
  }

  ImGuiIO &io = ImGui::GetIO();
  io.KeysDown[key] = true;

  // int mod = glutGetModifiers();
  io.KeyCtrl = (mods & GLFW_MOD_CONTROL) != 0;
  io.KeyShift = (mods & GLFW_MOD_SHIFT) != 0;
}

static void window_size_callback(GLFWwindow *window, int width, int height) {
  if (height <= 0)
    height = 1;

  float aspectRatio = (float)width / (float)height;
  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45.0f, aspectRatio, 0.1f, 10000.0f);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(Camera::position[0], Camera::position[1], Camera::position[2],
            Camera::target[0], Camera::target[1], Camera::target[2], 0.0f, 1.0f,
            0.0f);
}

int InitApp(int argc, char **argv) {
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

  glfwSetWindowSizeCallback(window, window_size_callback);
  glfwSetKeyCallback(window, key_callback);
  glfwSetMouseButtonCallback(window, mouse_button_callback);
  glfwSetCursorPosCallback(window, cursor_position_callback);

  // Setup all the open-gl states we want to use (ones that don't change in
  // the lifetime of the application) Note: These can be changed anywhere, but
  // generally we don't change the back buffer color
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glEnable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glCullFace(GL_BACK);
  glFrontFace(GL_CCW);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);

  // Used FFP to setup lights
  float floats[4];
  for (int i = 0; i < 4; ++i)
    floats[i] = (float)Light::ambient[i];
  glLightfv(GL_LIGHT0, GL_AMBIENT, floats);
  for (int i = 0; i < 4; ++i)
    floats[i] = (float)Light::diffuse[i];
  glLightfv(GL_LIGHT0, GL_DIFFUSE, floats);
  for (int i = 0; i < 4; ++i)
    floats[i] = (float)Light::specular[i];
  glLightfv(GL_LIGHT0, GL_SPECULAR, floats);
  for (int i = 0; i < 4; ++i)
    floats[i] = (float)Camera::position[i];
  glLightfv(GL_LIGHT0, GL_POSITION, floats);
  glEnable(GL_LIGHT0);
  glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

  int w, h;
  glfwGetFramebufferSize(window, &w, &h);

  // Setup Dear ImGui context
  const char *glsl_version = "#version 130";
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  (void)io;
  // Setup Platform/Renderer backends
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  while (!glfwWindowShouldClose(window)) { // message loop
    glfwPollEvents();
    glfwGetFramebufferSize(window, &w, &h);
    window_size_callback(window, w, h);

    DemosUpdate();

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    DemosGui();
    ImGui::Render();

    // Render
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    DemosRender();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwTerminate();
  return 0;
}
