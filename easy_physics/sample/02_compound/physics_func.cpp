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

using namespace EasyPhysics;

std::shared_ptr<PhysicsScene> createSceneCompound() {
  auto physics = std::make_shared<PhysicsScene>("compound shape");

  // 地面
  {
    auto body = physics->AddBody();
    EpxVector3 scale(10.0f, 1.0f, 10.0f);
    body.state.m_motionType = EpxMotionTypeStatic;
    body.state.m_position = EpxVector3(0.0f, -scale[1], 0.0f);
    physics->AddShape(body.id, EasyPhysics::EpxShapeType::Box, scale);
  }

  // 複合形状
  for (int i = 0; i < 5; i++) {
    auto body = physics->AddBody();
    body.state.m_position = EpxVector3(0.0f, 3.0f + 2.0f * i, 0.0f);
    body.state.m_orientation = EpxQuat::rotationY(0.5f * i);
    body.body.m_mass = 1.0f;
    body.body.m_inertia = epxCalcInertiaBox(EpxVector3(1.0f, 1.0f, 0.5f), 1.0f);

    {
      EpxVector3 scale(1.0f, 0.125f, 0.125f);
      physics->AddShape(body.id, EasyPhysics::EpxShapeType::Box, scale, false);
    }

    {
      EpxVector3 scale(0.5f);
      auto shape = physics->AddShape(body.id, EasyPhysics::EpxShapeType::Sphere,
                                     scale, false);
      shape->m_offsetPosition = EpxVector3(-1.0f, 0.0f, 0.0f);
    }

    {
      EpxVector3 scale(1.0f, 0.25f, 0.25f);
      auto shape = physics->AddShape(
          body.id, EasyPhysics::EpxShapeType::Cylinder, scale, false);
      shape->m_offsetPosition = EpxVector3(1.0f, 0.0f, 0.0f);
      shape->m_offsetOrientation = EpxQuat::rotationZ(0.5f * EPX_PI);
    }

    physics->FinishShape(body.id);
  }

  return physics;
}

std::shared_ptr<PhysicsScene> createSceneDaruma() {
  auto physics = std::make_shared<PhysicsScene>("daruma");

  // 地面
  {
    auto body = physics->AddBody();
    EpxVector3 scale(10.0f, 1.0f, 10.0f);
    body.state.m_motionType = EpxMotionTypeStatic;
    body.state.m_position = EpxVector3(0.0f, -scale[1], 0.0f);
    physics->AddShape(body.id, EasyPhysics::EpxShapeType::Box, scale);
  }

  // ダルマ複合形状
  for (int i = 0; i <= 5; i++) {
    auto body = physics->AddBody();
    body.state.m_position = EpxVector3(-5.0f + i * 2.0f, 3.0f, 0.0f);
    body.state.m_angularVelocity =
        EpxVector3(rand() % 5, rand() % 5, rand() % 5);
    body.body.m_mass = 3.0f;
    body.body.m_inertia = epxCalcInertiaSphere(1.5f, 3.0f);

    {
      EpxVector3 scale(1.0f);
      auto shape = physics->AddShape(body.id, EasyPhysics::EpxShapeType::Sphere,
                                     scale, false);
      shape->m_offsetPosition = EpxVector3(0.0f, 1.0f, 0.0f);
    }

    {
      EpxVector3 scale(0.5f);
      auto shape = physics->AddShape(body.id, EasyPhysics::EpxShapeType::Sphere,
                                     scale, false);
      shape->m_offsetPosition = EpxVector3(0.0f, 2.0f, 0.0f);
    }

    physics->FinishShape(body.id);
  }

  return physics;
}

std::shared_ptr<PhysicsScene> createSceneStackingPole() {
  auto physics = std::make_shared<PhysicsScene>("stacking 1");

  // 地面
  {
    auto body = physics->AddBody();
    EpxVector3 scale(10.0f, 1.0f, 10.0f);
    body.state.m_motionType = EpxMotionTypeStatic;
    body.state.m_position = EpxVector3(0.0f, -scale[1], 0.0f);
    physics->AddShape(body.id, EasyPhysics::EpxShapeType::Box, scale);
  }

  // ボックスを積み上げる
  const EpxVector3 brickScale(0.5f);
  for (int i = 0; i < 7; i++) {
    auto body = physics->AddBody();
    body.state.m_position = EpxVector3(0.0f, brickScale[1] + i, 0.0f);
    body.body.m_mass = 1.0f;
    body.body.m_inertia = epxCalcInertiaBox(brickScale, 1.0f);
    physics->AddShape(body.id, EasyPhysics::EpxShapeType::Box, brickScale);
  }

  return physics;
}

std::shared_ptr<PhysicsScene> createSceneStackingWall() {
  auto physics = std::make_shared<PhysicsScene>("stacking 2");

  // 地面
  {
    auto body = physics->AddBody();
    EpxVector3 scale(10.0f, 1.0f, 10.0f);
    body.state.m_motionType = EpxMotionTypeStatic;
    body.state.m_position = EpxVector3(0.0f, -scale[1], 0.0f);
    physics->AddShape(body.id, EasyPhysics::EpxShapeType::Box, scale);
  }

  // ボックスを積み上げる
  const EpxVector3 brickScale(0.5f);
  const EpxVector3 offsetPosition(0.0f, brickScale[1], 0.0f);
  const EpxFloat diff = brickScale[1] * 1.1f;
  int stackSize = 10;
  EpxFloat offset = -stackSize * (diff * 2.0f) * 0.5f;
  EpxVector3 pos(0.0f, diff, 0.0f);

  while (stackSize) {
    for (int i = 0; i < stackSize; i++) {
      auto body = physics->AddBody();
      pos[0] = offset + i * diff * 2.0f;
      body.state.m_position = offsetPosition + pos;
      body.body.m_mass = 1.0f;
      body.body.m_inertia = epxCalcInertiaBox(brickScale, 1.0f);
      physics->AddShape(body.id, EasyPhysics::EpxShapeType::Box, brickScale);
    }
    offset += diff;
    pos[1] += (diff * 2.0f);
    stackSize--;
  }

  return physics;
}
