﻿/*
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

#include "../common/common.h"
#include "../common/ctrl_func.h"
#include "../common/font_render_func.h"
#include "../common/render_func.h"
#include "physics_func.h"

#define SAMPLE_NAME "01_basic"

static bool s_isRunning = true;

int sceneId = 0;
bool simulating = false;

static void render(FontRenderer *font) {
  renderBegin();

  for (int i = 0; i < physicsGetNumRigidbodies(); i++) {
    auto &state = physicsGetState(i);
    auto &collidable = physicsGetCollidable(i);

    EasyPhysics::EpxTransform3 rigidBodyTransform(state.m_orientation,
                                                  state.m_position);

    for (int j = 0; j < collidable.m_numShapes; j++) {
      auto shape = collidable.m_shapes[j];
      EasyPhysics::EpxTransform3 shapeTransform(shape.m_offsetOrientation,
                                                shape.m_offsetPosition);
      EasyPhysics::EpxTransform3 worldTransform =
          rigidBodyTransform * shapeTransform;

      renderMesh(worldTransform, EasyPhysics::EpxVector3(1, 1, 1),
                 (int)shape.userData);
    }
  }

  renderDebugBegin();

  // 衝突点の表示
  const EasyPhysics::EpxVector3 colorA(1, 0, 0);
  const EasyPhysics::EpxVector3 colorB(0, 0, 1);
  const EasyPhysics::EpxVector3 colorLine(0, 1, 1);

  for (int i = 0; i < physicsGetNumContacts(); i++) {
    auto &contact = physicsGetContact(i);
    auto &stateA = physicsGetState(physicsGetRigidBodyAInContact(i));
    auto &stateB = physicsGetState(physicsGetRigidBodyBInContact(i));
    for (unsigned int j = 0; j < contact.m_numContacts; j++) {
      auto &contactPoint = contact.m_contactPoints[j];
      auto pointA =
          stateA.m_position + rotate(stateA.m_orientation, contactPoint.pointA);
      auto pointB =
          stateB.m_position + rotate(stateB.m_orientation, contactPoint.pointB);
      auto normal = contactPoint.normal;
      renderDebugPoint(pointA, colorA);
      renderDebugPoint(pointB, colorB);
      renderDebugLine(pointA, pointA + 0.1f * normal, colorLine);
      renderDebugLine(pointB, pointB - 0.1f * normal, colorLine);
    }
  }

  renderDebugEnd();

  renderDebug2dBegin();

  int width, height;
  renderGetScreenSize(width, height);

  EasyPhysics::EpxFloat dh = 20.0f;
  EasyPhysics::EpxFloat sx = -width * 0.5f + 20.0f;
  EasyPhysics::EpxFloat sy = height * 0.5f - 10.0f;

  font->Print((int)sx, (int)(sy -= dh), 0.5f, 1.0f, 0.5f, "Easy Physics : %s",
              physicsGetSceneTitle(sceneId));
  font->Print((int)sx, (int)(sy -= dh), 0.5f, 0.5f, 1.0f, "F1:Reset");
  font->Print((int)sx, (int)(sy -= dh), 0.5f, 0.5f, 1.0f, "F2:Next");
  font->Print((int)sx, (int)(sy -= dh), 0.5f, 0.5f, 1.0f, "F3:Play/Stop");
  font->Print((int)sx, (int)(sy -= dh), 0.5f, 0.5f, 1.0f, "F4:Step");
  font->Print((int)sx, (int)(sy -= dh), 0.5f, 0.5f, 1.0f, "Cursor:Rotate view");
  font->Print((int)sx, (int)(sy -= dh), 0.5f, 0.5f, 1.0f, "Ins/Del:Move view");
  font->Print((int)sx, (int)(sy -= dh), 0.5f, 0.5f, 1.0f, "L-Click:Fire");

  renderDebug2dEnd();

  renderEnd();
}

static int init(void) {
  renderInit(SAMPLE_NAME);
  physicsInit();

  return 0;
}

static int shutdown(void) {
  renderRelease();
  physicsRelease();

  return 0;
}

static void update(Control *ctrl) {
  float angX, angY, r;
  renderGetViewAngle(angX, angY, r);

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
    renderWait();
    renderReleaseMeshAll();
    physicsCreateScene(sceneId);
  }

  if (ctrl->ButtonPressed(BTN_SCENE_NEXT) == BTN_STAT_DOWN) {
    renderWait();
    renderReleaseMeshAll();
    physicsCreateScene(++sceneId);
  }

  if (ctrl->ButtonPressed(BTN_SIMULATION) == BTN_STAT_DOWN) {
    simulating = !simulating;
  }

  if (ctrl->ButtonPressed(BTN_STEP) == BTN_STAT_DOWN) {
    simulating = true;
  } else if (ctrl->ButtonPressed(BTN_STEP) == BTN_STAT_UP ||
             ctrl->ButtonPressed(BTN_STEP) == BTN_STAT_KEEP) {
    simulating = false;
  }

  if (ctrl->ButtonPressed(BTN_PICK) == BTN_STAT_DOWN) {
    int sx, sy;
    ctrl->GetCursorPosition(sx, sy);
    EasyPhysics::EpxVector3 wp1((float)sx, (float)sy, 0.0f);
    EasyPhysics::EpxVector3 wp2((float)sx, (float)sy, 1.0f);
    wp1 = renderGetWorldPosition(wp1);
    wp2 = renderGetWorldPosition(wp2);
    physicsFire(wp1, normalize(wp2 - wp1) * 50.0f);
  }

  renderSetViewAngle(angX, angY, r);
}

///////////////////////////////////////////////////////////////////////////////
// WinMain

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance,
                   LPSTR lpCmdLine, int nCmdShow) {
  init();

  physicsCreateScene(sceneId);

  FontRenderer font;
  Control ctrl;

  MSG msg;
  while (s_isRunning) {
    if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {
      if (msg.message == WM_QUIT) {
        s_isRunning = false;
      } else {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
      }
    } else {
      update(&ctrl);
      if (simulating)
        physicsSimulate();
      render(&font);
    }
  }

  shutdown();

  return (msg.wParam);
}
