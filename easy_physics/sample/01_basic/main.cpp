#include "physics_func.h"
#include <array>
#include <common/FontStashRenderer.h>
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

  spanmath::OrbitView turntable_;

  // ScreenCamera camera;
  // platform.BindCamera(camera);
  // platform.BindSelector(selector, camera);

  // Renderer
  cuber::gl3::GlCubeRenderer cubeRenderer;
  cuber::gl3::GlLineRenderer lineRenderer;
  // Gl1Renderer renderer;
  FontStashRenderer stash(
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

    // camera
    turntable_.SetSize(static_cast<int>(io.DisplaySize.x),
                       static_cast<int>(io.DisplaySize.y));
    if (!io.WantCaptureMouse) {
      if (io.MouseDown[ImGuiMouseButton_Right]) {
        turntable_.YawPitch(static_cast<int>(io.MouseDelta.x),
                            static_cast<int>(io.MouseDelta.y));
      }
      if (io.MouseDown[ImGuiMouseButton_Middle]) {
        turntable_.Shift(static_cast<int>(io.MouseDelta.x),
                         static_cast<int>(io.MouseDelta.y));
      }
      turntable_.Dolly(static_cast<int>(io.MouseWheel));
    }
    turntable_.Update(spanmath::Mat4(projection), spanmath::Mat4(view));

    // update
    selector.Update();
    auto data = selector.DrawData();
    // auto [projection, view] = camera.UpdateProjectionView(width, height);

    // render
    {
      cubeRenderer.Render(projection.data(), view.data(), data.boxes);
      cubeRenderer.Render(projection.data(), view.data(), data.spheres);
      cubeRenderer.Render(projection.data(), view.data(), data.cylinders);
      cubeRenderer.Render(projection.data(), view.data(), data.tetrahedrons);
      // lineRenderer.Render(projection, view, data.tetrahedrons);
      // stash.Draw(x, y, width, height, selector.Title());

      ImGui::Render();
      platform.EndFrame(ImGui::GetDrawData());
    }
  }

  return 0;
}
