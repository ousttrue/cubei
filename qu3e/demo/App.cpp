#include "App.h"
#include "GL2Renderer.h"
#include "GL3Renderer.h"
#include <gl/glew.h>

#include "demos/BoxStack.h"
#include "demos/Demo.h"
#include "demos/DropBoxes.h"
#include "demos/RayPush.h"
#include "demos/Test.h"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <Remotery.h>
#include <numbers>

const std::chrono::nanoseconds DELTA =
    std::chrono::nanoseconds(1000000000 / 60);

App::App(GLFWwindow *window) : scene_(new q3Scene) {

  env_.m_dt = std::chrono::duration_cast<
                  std::chrono::duration<float, std::ratio<1, 1>>>(DELTA)
                  .count();

  // Setup Dear ImGui context
  const char *glsl_version = "#version 130";
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  (void)io;
  // Setup Platform/Renderer backends
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  // Globals for maintaining a list of demos
  demos_.push_back(std::make_shared<DropBoxes>());
  demos_.push_back(std::make_shared<RayPush>());
  demos_.push_back(std::make_shared<BoxStack>());
  demos_.push_back(std::make_shared<Test>());

  glewInit();
  renderer_.reset(new GL3Renderer);
  // renderer_.reset(new GL2Renderer);
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

void App::DemosMouseLeftDown(int x, int y) {
  demos_[currentDemo_]->LeftClick(x, y);
}

void App::DemosKeyDown(unsigned char key) {
  demos_[currentDemo_]->KeyDown(key);
}

void App::DemosKeyUp(unsigned char key) { demos_[currentDemo_]->KeyUp(key); }

void App::DemosTogglePause() { paused_ = paused_ ? true : false; }

void App::DemosSingleStep() {
  paused_ = true;
  singleStep_ = true;
}

void App::DemosSceneDump() {
  FILE *fp = fopen(sceneFileName_, "w");
  scene_->Dump(fp, env_);
  fclose(fp);
}

void App::Frame(int w, int h) {
  if (currentDemo_ != lastDemo_) {
    if (lastDemo_ == -1) {
      // first time
      sprintf(sceneFileName_, "q3dump.txt");
    } else {
      demos_[lastDemo_]->Shutdown(scene_.get());
    }
    demos_[currentDemo_]->Init(scene_.get());
    lastDemo_ = currentDemo_;
  }

  auto time = std::chrono::high_resolution_clock::now();
  auto delta = time - time_;
  time_ = time;

  // The time accumulator is used to allow the application to render at
  // a frequency different from the constant frequency the physics sim-
  // ulation is running at (default 60Hz).
  // static float accumulator = 0;
  // accumulator += time;
  // accumulator = q3Clamp01(accumulator);
  // while (accumulator >= dt_)
  {
    rmt_ScopedCPUSample(Qu3eStep, 0);
    if (!paused_) {
      scene_->Step(env_);
      demos_[currentDemo_]->Update(scene_.get(), delta);
    } else {
      if (singleStep_) {
        scene_->Step(env_);
        demos_[currentDemo_]->Update(scene_.get(), DELTA);
        singleStep_ = false;
      }
    }
  }

  {
    rmt_ScopedCPUSample(ImGui, 0);
    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    auto &io = ImGui::GetIO();
    camera_.Resize(io.DisplaySize.x, io.DisplaySize.y);
    if (!io.WantCaptureMouse) {
      // update camera
      auto TO_RAD = static_cast<float>(std::numbers::pi / 180.0);
      if (ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
        camera_.YawPitch(io.MouseDelta.x * TO_RAD, io.MouseDelta.y * TO_RAD);
      }
      if (ImGui::IsMouseDown(ImGuiMouseButton_Middle)) {
        camera_.Shift(io.MouseDelta.x, io.MouseDelta.y);
      }
      camera_.Dolly(io.MouseWheel);
    }

    // ImGui::SetNewWindowDefaultPos(ImVec2(float(w - 300 - 30), 30));
    ImGui::SetNextWindowSize(ImVec2(300, 225), ImGuiCond_Appearing);
    ImGui::Begin("q3Scene Settings");
    ImGui::Combo("Demo", &currentDemo_,
                 "Drop Boxes\0Ray Push\0Box Stack\0Test\0");
    ImGui::Checkbox("Pause", &paused_);
    if (paused_)
      ImGui::Checkbox("Single Step", &singleStep_);
    ImGui::Checkbox("Sleeping", &env_.m_allowSleep);
    ImGui::Checkbox("Friction", &env_.m_enableFriction);
    ImGui::SliderInt("Iterations", &env_.m_iterations, 1, 50);
    int flags = (1 << 0) | (1 << 1) | (1 << 2);
    ImGui::InputText("Dump File Name", sceneFileName_,
                     ((int)(sizeof(sceneFileName_) / sizeof(*sceneFileName_))),
                     flags);
    if (ImGui::Button("Dump Scene")) {
      DemosSceneDump();
    }
    ImGui::End();

    ImGui::ShowMetricsWindow();
    ImGui::Render();
  }

  // render
  {
    rmt_ScopedCPUSample(Render, 0);

    camera_.Update();
    renderer_->BeginFrame(w, h, &camera_.projection._11, &camera_.view._11);
    scene_->Render(renderer_.get());
    demos_[currentDemo_]->Render(renderer_.get());
    renderer_->EndFrame(&camera_.projection._11, &camera_.view._11);
  }

  {
    rmt_ScopedCPUSample(ImGuiRender, 0);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  }
}
