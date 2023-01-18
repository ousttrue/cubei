#include "Demo.h"
#include "Camera.h"
#include "Renderer.h"

#include "BoxStack.h"
#include "DropBoxes.h"
#include "RayPush.h"
#include "Test.h"

#include "Clock.h"
#include <imgui.h>
#include <stdio.h>
#include <gl/GLU.h>

Renderer g_renderer;

Clock g_clock;
// extern float dt;
float dt = 1.0f / 60.0f;
q3Scene scene(dt);
// Is frame by frame stepping enabled?
bool paused = false;
// Can the simulation take a step, while paused is enabled?
bool singleStep = false;
// Globals for running the scene
bool enableSleep = true;
bool enableFriction = true;
int velocityIterations = 10;

int demoCount = 4;
int currentDemo = 3;
char sceneFileName[256] = {0};
int lastDemo = -1;
// Globals for maintaining a list of demos
Demo *demos[] = {
    new DropBoxes(),
    new RayPush(),
    new BoxStack(),
    new Test(),
};

void DemosMouseLeftDown(int x, int y) { demos[currentDemo]->LeftClick(x, y); }

void DemosKeyDown(unsigned char key) { demos[currentDemo]->KeyDown(key); }

void DemosKeyUp(unsigned char key) { demos[currentDemo]->KeyUp(key); }

// In main.cpp
void UpdateScene(float time) {
  // The time accumulator is used to allow the application to render at
  // a frequency different from the constant frequency the physics sim-
  // ulation is running at (default 60Hz).
  static float accumulator = 0;
  accumulator += time;

  accumulator = q3Clamp01(accumulator);
  while (accumulator >= dt) {
    if (!paused) {
      scene.Step();
      demos[currentDemo]->Update(&scene, dt);
    }

    else {
      if (singleStep) {
        scene.Step();
        demos[currentDemo]->Update(&scene, dt);
        singleStep = false;
      }
    }

    accumulator -= dt;
  }
}

void DemosTogglePause() { paused = paused ? true : false; }

void DemosSingleStep() {
  paused = true;
  singleStep = true;
}

void DemosSceneDump() {
  FILE *fp = fopen(sceneFileName, "w");
  scene.Dump(fp);
  fclose(fp);
}

void DemosUpdate() {
  if (currentDemo != lastDemo) {
    if (lastDemo == -1) {
      // first time
      sprintf(sceneFileName, "q3dump.txt");
    } else {
      demos[lastDemo]->Shutdown(&scene);
    }
    demos[currentDemo]->Init(&scene);
    lastDemo = currentDemo;
  }

  float time = g_clock.Start();

  scene.SetAllowSleep(enableSleep);
  scene.SetEnableFriction(enableFriction);
  scene.SetIterations(velocityIterations);

  UpdateScene(time);

  g_clock.Stop();
}

void DemosRender(int width, int height, const Camera &camera) {
  if (height <= 0)
    height = 1;

  float aspectRatio = (float)width / (float)height;
  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45.0f, aspectRatio, 0.1f, 10000.0f);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(camera.position[0], camera.position[1], camera.position[2],
            camera.target[0], camera.target[1], camera.target[2], 0.0f, 1.0f,
            0.0f);

  scene.Render(&g_renderer);
  demos[currentDemo]->Render(&g_renderer);
}

void DemosGui() {
  // ImGui::SetNewWindowDefaultPos(ImVec2(float(w - 300 - 30), 30));
  ImGui::SetNextWindowSize(ImVec2(300, 225), ImGuiCond_Appearing);
  ImGui::Begin("q3Scene Settings");
  ImGui::Combo("Demo", &currentDemo, "Drop Boxes\0Ray Push\0Box Stack\0Test\0");
  ImGui::Checkbox("Pause", &paused);
  if (paused)
    ImGui::Checkbox("Single Step", &singleStep);
  ImGui::Checkbox("Sleeping", &enableSleep);
  ImGui::Checkbox("Friction", &enableFriction);
  ImGui::SliderInt("Iterations", &velocityIterations, 1, 50);
  int flags = (1 << 0) | (1 << 1) | (1 << 2);
  ImGui::InputText("Dump File Name", sceneFileName,
                   ((int)(sizeof(sceneFileName) / sizeof(*sceneFileName))),
                   flags);
  if (ImGui::Button("Dump Scene")) {
    DemosSceneDump();
  }
  ImGui::End();
}