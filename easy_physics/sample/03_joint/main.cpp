#include "physics_func.h"
#include <GLFW/glfw3.h>
#include <common/FontStashRenderer.h>
#include <common/Gl1Renderer.h>
#include <common/GlfwPlatform.h>
#include <common/PhysicsSceneSelector.h>
#include <common/ScreenCamera.h>

using namespace EasyPhysics;

auto SAMPLE_NAME = "03_joint";

int main(int argc, char **argv) {

  PhysicsSceneSelector selector(generators);

  // GUI
  ScreenCamera camera;
  GlfwPlatform platform(SAMPLE_NAME);
  platform.BindCamera(camera);
  platform.BindSelector(selector, camera);

  // Renderer
  Gl1Renderer renderer;
  FontStashRenderer stash(
      "sans", argc > 1
                  ? argv[1]
                  : "subprojects/fontstash/example/DroidSerif-Regular.ttf");

  // mainloop
  int x, y, width, height;
  while (platform.NewFrame(&x, &y, &width, &height)) {
    // update
    selector.Update();
    auto [projection, view] = camera.UpdateProjectionView(width, height);

    // render
    renderer.Begin(width, height, projection, view);
    auto data = selector.DrawData();
    renderer.Render(data);
    stash.Draw(x, y, width, height, selector.Title());
    platform.EndFrame();
  }

  return 0;
}
