/*
        Copyright (c) 2012 Hiroshi Matsuike

        This software is provided 'as-is', without any express or implied
        warranty. In no event will the authors be held liable for any damages
        arising from the use of this software.

        Permission is granted to anyone to use this software for any purpose,
        including commercial applications, and to alter it and redistribute it
        freely, subject to the following restrictions:

        1. The origin of this software must not be misrepresented; you must not
        claim that you wrote the original software. If you use this software
        in a product, an acknowledgment in the product documentation would be
        appreciated but is not required.

        2. Altered source versions must be plainly marked as such, and must not
   be misrepresented as being the original software.

        3. This notice may not be removed or altered from any source
   distribution.
*/

#include "physics_func.h"
#include <common/common.h>
#include <common/ctrl_func.h>
#include <common/render_func.h>
#include <format>
#include <gl/GL.h>
#include <stdexcept>

#define FONTSTASH_IMPLEMENTATION
#include "fontstash.h"
#define GLFONTSTASH_IMPLEMENTATION
#include "glfontstash.h"

#define SAMPLE_NAME "01_basic"

static bool g_isRunning = true;
static bool g_simulating = false;
static int g_sceneId = 0;
PhysicsState g_state = {};
static std::shared_ptr<PhysicsScene> g_scene;

static void update(Renderer *renderer, Control *ctrl) {
  float angX, angY, r;
  renderer->GetViewAngle(angX, angY, r);

  ctrl->Update();

  if (ctrl->ButtonPressed(BTN_UP)) {
    angX -= 0.05f;
    if (angX < -1.4f)
      angX = -1.4f;
    if (angX > -0.01f)
      angX = -0.01f;
  }

  if (ctrl->ButtonPressed(BTN_DOWN)) {
    angX += 0.05f;
    if (angX < -1.4f)
      angX = -1.4f;
    if (angX > -0.01f)
      angX = -0.01f;
  }

  if (ctrl->ButtonPressed(BTN_LEFT)) {
    angY -= 0.05f;
  }

  if (ctrl->ButtonPressed(BTN_RIGHT)) {
    angY += 0.05f;
  }

  if (ctrl->ButtonPressed(BTN_ZOOM_OUT)) {
    r *= 1.1f;
    if (r > 500.0f)
      r = 500.0f;
  }

  if (ctrl->ButtonPressed(BTN_ZOOM_IN)) {
    r *= 0.9f;
    if (r < 1.0f)
      r = 1.0f;
  }

  if (ctrl->ButtonPressed(BTN_SCENE_RESET) == BTN_STAT_DOWN) {
    renderer->Wait();
    renderer->ReleaseMeshAll();
    physicsCreateScene(g_sceneId, renderer);
    g_state.Clear();
  }

  if (ctrl->ButtonPressed(BTN_SCENE_NEXT) == BTN_STAT_DOWN) {
    renderer->Wait();
    renderer->ReleaseMeshAll();
    g_scene = physicsCreateScene(++g_sceneId, renderer);
    g_state.Clear();
  }

  if (ctrl->ButtonPressed(BTN_SIMULATION) == BTN_STAT_DOWN) {
    g_simulating = !g_simulating;
  }

  if (ctrl->ButtonPressed(BTN_STEP) == BTN_STAT_DOWN) {
    g_simulating = true;
  } else if (ctrl->ButtonPressed(BTN_STEP) == BTN_STAT_UP ||
             ctrl->ButtonPressed(BTN_STEP) == BTN_STAT_KEEP) {
    g_simulating = false;
  }

  if (ctrl->ButtonPressed(BTN_PICK) == BTN_STAT_DOWN) {
    int sx, sy;
    ctrl->GetCursorPosition(sx, sy);
    EasyPhysics::EpxVector3 wp1((float)sx, (float)sy, 0.0f);
    EasyPhysics::EpxVector3 wp2((float)sx, (float)sy, 1.0f);
    wp1 = renderer->GetWorldPosition(wp1);
    wp2 = renderer->GetWorldPosition(wp2);
    g_scene->PhysicsFire(wp1, normalize(wp2 - wp1) * 50.0f);
  }

  renderer->SetViewAngle(angX, angY, r);
}

static unsigned int glfonsFloatRGBA(float r, float g, float b, float a) {
  return glfonsRGBA((uint8_t)(r * 255), (uint8_t)(g * 255), (uint8_t)(b * 255),
                    (uint8_t)(a * 255));
}

class Font {
  FONScontext *fs = NULL;
  int fontNormal = FONS_INVALID;

public:
  Font(const char *name, const char *path) {
    fs = glfonsCreate(512, 512, FONS_ZERO_TOPLEFT);
    if (fs == NULL) {
      throw std::runtime_error("Could not create stash.");
    }

    fontNormal = fonsAddFont(fs, name, path);
    if (fontNormal == FONS_INVALID) {
      throw std::runtime_error("Could not add font normal.");
    }
  }
  ~Font() { glfonsDelete(fs); }

  void Draw(int width, int height, std::string_view title) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_TEXTURE_2D);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, width, height, 0, -1, 1);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glDisable(GL_DEPTH_TEST);
    glColor4ub(255, 255, 255, 255);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_CULL_FACE);

    auto white = glfonsRGBA(255, 255, 255, 255);
    auto green = glfonsFloatRGBA(0.5f, 1.0f, 0.5f, 1);
    auto blue = glfonsFloatRGBA(0.5f, 0.5f, 1.0f, 1);

    fonsClearState(fs);
    fonsSetSize(fs, 20.0f);
    fonsSetFont(fs, fontNormal);
    float lh;
    fonsVertMetrics(fs, NULL, NULL, &lh);

    EasyPhysics::EpxFloat dh = 20.0f;
    EasyPhysics::EpxFloat sx = width - 200;
    EasyPhysics::EpxFloat sy = 0;

    // 0.5f, 1.0f, 0.5f,
    fonsSetColor(fs, green);
    fonsDrawText(fs, 0, (sy += dh),
                 std::format("Easy Physics : {}", title).c_str(), nullptr);
    // 0.5f, 0.5f, 1.0f,
    fonsSetColor(fs, blue);
    fonsDrawText(fs, sx, (sy += dh), "F1:Reset", nullptr);
    fonsDrawText(fs, sx, (sy += dh), "F2:Next", nullptr);
    fonsDrawText(fs, sx, (sy += dh), "F3:Play/Stop", nullptr);
    fonsDrawText(fs, sx, (sy += dh), "F4:Step", nullptr);
    fonsDrawText(fs, sx, (sy += dh), "Cursor:Rotate view", nullptr);
    fonsDrawText(fs, sx, (sy += dh), "Ins/Del:Move view", nullptr);
    fonsDrawText(fs, sx, (sy += dh), "L-Click:Fire", nullptr);
  }
};

///////////////////////////////////////////////////////////////////////////////
// WinMain

// int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR
// lpCmdLine, int nCmdShow)
int main(int argc, char **argv) {

  Control ctrl;
  Renderer renderer(SAMPLE_NAME);
  // FontRenderer font(renderer.GetDC());

  Font stash("sans",
             argc > 1 ? argv[1]
                      : "subprojects/fontstash/example/DroidSerif-Regular.ttf");

  g_scene = physicsCreateScene(g_sceneId, &renderer);
  g_state.Clear();

  MSG msg;
  while (g_isRunning) {
    if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {
      if (msg.message == WM_QUIT) {
        g_isRunning = false;
      } else {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
      }
    } else {
      update(&renderer, &ctrl);
      if (g_simulating) {
        g_scene->Simulate(g_state);
      }

      renderer.Begin();
      PhysicsRender(*g_scene, g_state, &renderer, nullptr);

      // int width, height;
      // renderer.GetScreenSize(width, height);
      renderer.Debug2dBegin();
      int width, height;
      renderer.GetScreenSize(width, height);
      stash.Draw(width, height, g_scene->title_);
      renderer.Debug2dEnd();

      renderer.End();
    }
  }

  return (msg.wParam);
}
