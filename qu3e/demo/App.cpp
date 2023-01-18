#include "App.h"

#include "Renderer.h"

#include <gl/GL.h>
#include "demos/Demo.h"
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <iostream>
#include <stdio.h>

App::App(GLFWwindow *window) {
  // Setup Dear ImGui context
  const char *glsl_version = "#version 130";
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  (void)io;
  // Setup Platform/Renderer backends
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  renderer_ = new Renderer();

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
    floats[i] = (float)light_.ambient[i];
  glLightfv(GL_LIGHT0, GL_AMBIENT, floats);
  for (int i = 0; i < 4; ++i)
    floats[i] = (float)light_.diffuse[i];
  glLightfv(GL_LIGHT0, GL_DIFFUSE, floats);
  for (int i = 0; i < 4; ++i)
    floats[i] = (float)light_.specular[i];
  glLightfv(GL_LIGHT0, GL_SPECULAR, floats);
  for (int i = 0; i < 4; ++i)
    floats[i] = (float)camera_.position[i];
  glLightfv(GL_LIGHT0, GL_POSITION, floats);
  glEnable(GL_LIGHT0);
  glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
}

App::~App() {
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  delete renderer_;
}

void App::KeyDown(char key) {
  const float increment = 0.2f;
  switch (key) {
  case 'p':
    DemosTogglePause();
    break;
  case ' ':
    DemosSingleStep();
    break;
  // case 'w':
  //   Camera::position[2] -= increment;
  //   Camera::target[2] -= increment;
  //   break;
  // case 's':
  //   Camera::position[2] += increment;
  //   Camera::target[2] += increment;
  //   break;
  // case 'a':
  //   Camera::position[0] -= increment;
  //   Camera::target[0] -= increment;
  //   break;
  // case 'd':
  //   Camera::position[0] += increment;
  //   Camera::target[0] += increment;
  //   break;
  // case 'q':
  //   Camera::position[1] -= increment;
  //   Camera::target[1] -= increment;
  //   break;
  // case 'e':
  //   Camera::position[1] += increment;
  //   Camera::target[1] += increment;
  //   break;
  default:
    // printf( "%d\n", key );
    break;
  }
}

int InitApp(int argc, char **argv) {

  // int w, h;
  // glfwGetFramebufferSize(window, &w, &h);

  return 0;
}

void App::Frame(int w, int h) {
  DemosUpdate();

  // Start the Dear ImGui frame
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
  DemosGui();
  ImGui::ShowMetricsWindow();
  ImGui::Render();

  // Render
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  DemosRender(renderer_, w, h, camera_);
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}
