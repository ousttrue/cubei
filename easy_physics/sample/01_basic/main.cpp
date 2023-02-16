#include "physics_func.h"
#include <common/FontStashRenderer.h>
#include <common/GlfwPlatform.h>
#include <common/PhysicsSceneSelector.h>
#include <common/ScreenCamera.h>
#include <cuber/gl3/GlCubeRenderer.h>
#include <cuber/gl3/GlLineRenderer.h>
#include <gl/glew.h>

auto SAMPLE_NAME = "01_basic";

int main(int argc, char **argv) {

  // physics
  PhysicsSceneSelector selector(generators);

  // GUI
  GlfwPlatform platform(SAMPLE_NAME);
  glewInit();
  ScreenCamera camera;
  platform.BindCamera(camera);
  platform.BindSelector(selector, camera);

  // Renderer
  cuber::gl3::GlCubeRenderer cubeRenderer;
  cuber::gl3::GlLineRenderer lineRenderer;
  // Gl1Renderer renderer;
  FontStashRenderer stash(
      "sans", argc > 1
                  ? argv[1]
                  : "subprojects/fontstash/example/DroidSerif-Regular.ttf");

  // mainloop
  int x, y, width, height;
  while (platform.NewFrame(&x, &y, &width, &height)) {
    // clear viewport
    glViewport(0, 0, width, height);
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // update
    selector.Update();
    auto data = selector.DrawData();
    auto [projection, view] = camera.UpdateProjectionView(width, height);

    // render
    {
      // renderer.Begin(width, height, projection, view);
      // renderer.Render(data);
      cubeRenderer.Render(projection, view, data.boxes);
      cubeRenderer.Render(projection, view, data.spheres);
      cubeRenderer.Render(projection, view, data.cylinders);
      cubeRenderer.Render(projection, view, data.tetrahedrons);
      // lineRenderer.Render(projection, view, data.tetrahedrons);
      stash.Draw(x, y, width, height, selector.Title());
      platform.EndFrame();
    }
  }

  return 0;
}
