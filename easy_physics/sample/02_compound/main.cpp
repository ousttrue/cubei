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
#include <common/FontStashRenderer.h>
#include <common/Gl1Renderer.h>
#include <common/common.h>
#include <common/ctrl_func.h>
#include <common/win32window.h>

using namespace EasyPhysics;

#define SAMPLE_NAME "03_compound"

static bool g_isRunning = true;
int g_sceneId = 0;
bool g_simulating = false;

static void render(Gl1Renderer *renderer) {
  for (int i = 0; i < physicsGetNumRigidbodies(); i++) {
    const EpxState &state = physicsGetState(i);
    const EpxCollidable &collidable = physicsGetCollidable(i);

    EpxTransform3 rigidBodyTransform(state.m_orientation, state.m_position);

    for (int j = 0; j < collidable.m_numShapes; j++) {
      const EpxShape &shape = collidable.m_shapes[j];
      EpxTransform3 shapeTransform(shape.m_offsetOrientation,
                                   shape.m_offsetPosition);
      EpxTransform3 worldTransform = rigidBodyTransform * shapeTransform;

      renderer->Mesh(worldTransform, EpxVector3(1, 1, 1), (int)shape.userData);
    }
  }

  renderer->DebugBegin();

  // 衝突点の表示
  const EpxVector3 colorA(1, 0, 0);
  const EpxVector3 colorB(0, 0, 1);
  const EpxVector3 colorLine(0, 1, 1);

  for (int i = 0; i < physicsGetNumContacts(); i++) {
    const EpxContact &contact = physicsGetContact(i);
    const EpxState &stateA = physicsGetState(physicsGetRigidBodyAInContact(i));
    const EpxState &stateB = physicsGetState(physicsGetRigidBodyBInContact(i));
    for (unsigned int j = 0; j < contact.m_numContacts; j++) {
      const EpxContactPoint &contactPoint = contact.m_contactPoints[j];
      EpxVector3 pointA =
          stateA.m_position + rotate(stateA.m_orientation, contactPoint.pointA);
      EpxVector3 pointB =
          stateB.m_position + rotate(stateB.m_orientation, contactPoint.pointB);
      EpxVector3 normal = contactPoint.normal;
      renderer->DebugPoint(pointA, colorA);
      renderer->DebugPoint(pointB, colorB);
      renderer->DebugLine(pointA, pointA + 0.1f * normal, colorLine);
      renderer->DebugLine(pointB, pointB - 0.1f * normal, colorLine);
    }
  }

  renderer->DebugEnd();
}

static void update(Gl1Renderer *renderer, Control *ctrl, int width,
                   int height) {
  auto [angX, angY, r] = renderer->GetViewAngle();

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
    // renderer->Wait();
    renderer->ReleaseMeshAll();
    physicsCreateScene(g_sceneId, renderer);
  }

  if (ctrl->ButtonPressed(BTN_SCENE_NEXT) == BTN_STAT_DOWN) {
    // renderer->Wait();
    renderer->ReleaseMeshAll();
    physicsCreateScene(++g_sceneId, renderer);
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
    EpxVector3 wp1((float)sx, (float)sy, 0.0f);
    EpxVector3 wp2((float)sx, (float)sy, 1.0f);
    wp1 = renderer->GetWorldPosition(wp1, width, height);
    wp2 = renderer->GetWorldPosition(wp2, width, height);
    physicsFire(wp1, normalize(wp2 - wp1) * 50.0f);
  }

  renderer->SetViewAngle(angX, angY, r);
}

///////////////////////////////////////////////////////////////////////////////
// WinMain

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance,
                   LPSTR lpCmdLine, int nCmdShow) {

  Win32Window window(hInstance, SAMPLE_NAME, 640, 480);
  physicsInit();
  Gl1Renderer renderer;
  physicsCreateScene(g_sceneId, &renderer);
  Control ctrl;
  FontStashRenderer stash(
      "sans", "subprojects/fontstash/example/DroidSerif-Regular.ttf");

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
      auto [width, height] = window.GetSize();

      update(&renderer, &ctrl, width, height);
      if (g_simulating) {
        physicsSimulate();
      }

      renderer.Begin(width, height);
      render(&renderer);

      int sx, sy;
      ctrl.GetCursorPosition(sx, sy);
      renderer.Debug2dBegin(width, height);
      stash.Draw(sx, sy, width, height, physicsGetSceneTitle(g_sceneId));
      renderer.Debug2dEnd();

      window.SwapBuffers();
    }
  }

  physicsRelease();

  return (msg.wParam);
}
