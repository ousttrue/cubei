#include "App.h"

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
}

App::~App() {
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
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

void App::Frame(int w, int h) {
  DemosUpdate();

  // Start the Dear ImGui frame
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
  DemosGui();
  ImGui::ShowMetricsWindow();
  ImGui::Render();

  DemosRender(w, h);
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}
