#include "physics_func.h"
#include <array>
#include <common/Gl3FontStashRenderer.h>
#include <common/GlfwPlatform.h>
#include <common/PhysicsSceneSelector.h>
#include <common/ScreenCamera.h>
#include <cuber/gl3/GlCubeRenderer.h>
#include <cuber/gl3/GlLineRenderer.h>
#include <gl/glew.h>
#include <imgui.h>
#include <spanmath/orbitview.h>
#include <spanmath/spanmath.h>

auto SAMPLE_NAME = "01_basic";

int main(int argc, char **argv) {

  // physics
  PhysicsSceneSelector selector(generators);

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  (void)io;
  // io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable
  // Keyboard Controls io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad; //
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  // Enable Gamepad Controls

  // Setup Dear ImGui style
  ImGui::StyleColorsDark();
  // ImGui::StyleColorsLight();

  // GUI
  GlfwPlatform platform;
  if (!platform.Create(SAMPLE_NAME)) {
    return 1;
  }

  spanmath::OrbitView camera;
  camera.shift_[2] = -10.0f;

  // ScreenCamera camera;
  // platform.BindCamera(camera);
  // platform.BindSelector(selector, camera);

  // Renderer
  cuber::gl3::GlCubeRenderer cubeRenderer;
  cuber::gl3::GlLineRenderer lineRenderer;
  Gl3FontStashRenderer stash(
      "sans", argc > 1
                  ? argv[1]
                  : "subprojects/fontstash/example/DroidSerif-Regular.ttf");

  // mainloop
  // int x, y, width, height;
  float clearColor[4] = {0, 0, 0, 0};
  std::array<float, 16> projection;
  std::array<float, 16> view;
  while (platform.NewFrame(clearColor)) {
    ImGuiIO &io = ImGui::GetIO();
    ImGui::NewFrame();

    if (ImGui::IsKeyPressed(ImGuiKey_F2)) {
      selector.Reset();
    }
    if (ImGui::IsKeyPressed(ImGuiKey_F2)) {
      selector.Next();
    }
    if (ImGui::IsKeyPressed(ImGuiKey_F3)) {
      selector.Toggle();
    }
    if (ImGui::IsMouseDown(ImGuiButtonFlags_MouseButtonLeft)) {
      // auto pos = io.MousePos;
    }

    // camera
    camera.SetSize(static_cast<int>(io.DisplaySize.x),
                   static_cast<int>(io.DisplaySize.y));
    if (!io.WantCaptureMouse) {
      if (io.MouseDown[ImGuiMouseButton_Right]) {
        camera.YawPitch(static_cast<int>(io.MouseDelta.x),
                        static_cast<int>(io.MouseDelta.y));
      }
      if (io.MouseDown[ImGuiMouseButton_Middle]) {
        camera.Shift(static_cast<int>(io.MouseDelta.x),
                     static_cast<int>(io.MouseDelta.y));
      }
      camera.Dolly(static_cast<int>(io.MouseWheel));
    }
    camera.Update(spanmath::Mat4(projection), spanmath::Mat4(view));

    ImGui::ShowMetricsWindow();

    // update
    selector.Update();
    auto data = selector.DrawData();

    // render
    {
      cubeRenderer.Render(projection.data(), view.data(), data.boxes);
      // cubeRenderer.Render(projection.data(), view.data(), data.spheres);
      // cubeRenderer.Render(projection.data(), view.data(), data.cylinders);
      // cubeRenderer.Render(projection.data(), view.data(), data.tetrahedrons);
      // lineRenderer.Render(projection, view, data.tetrahedrons);
      // stash.Draw(io.DisplaySize.x, io.DisplaySize.y, selector.Title());

      ImGui::Render();
      platform.EndFrame(ImGui::GetDrawData());
    }
  }

  return 0;
}
