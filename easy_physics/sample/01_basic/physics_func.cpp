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
#include <stdexcept>

using namespace EasyPhysics;

static std::shared_ptr<PhysicsScene> createSceneTwoBox(Geometry &scene) {
  auto physics = std::make_shared<PhysicsScene>("basic rigid bodies");

  // 地面
  {
    auto body = physics->AddBody();
    EpxVector3 scale(10.0f, 1.0f, 10.0f);
    // 剛体を表現するための各種データを初期化
    body.state.m_motionType = EpxMotionTypeStatic;
    body.state.m_position = EpxVector3(0.0f, -scale[1], 0.0f);
    physics->AddShape(scene, body.id, EpxShapeType::Box, scale);
  }

  // ボックス
  {
    auto body = physics->AddBody();
    EpxVector3 scale(2.0f, 0.25f, 1.0f);
    // 剛体を表現するための各種データを初期化
    body.state.m_position = EpxVector3(0.0f, scale[1], 0.0f);
    body.body.m_mass = 1.0f;
    body.body.m_inertia = epxCalcInertiaBox(scale, 1.0f);
    physics->AddShape(scene, body.id, EpxShapeType::Box, scale);
  }

  {
    auto body = physics->AddBody();
    EpxVector3 scale(2.0f, 0.25f, 1.0f);
    // 剛体を表現するための各種データを初期化
    body.state.m_position = EpxVector3(0.0f, 3.0f, 0.0f);
    body.state.m_orientation =
        EpxQuat::rotationZ(2.0f) * EpxQuat::rotationY(0.7f);
    body.body.m_mass = 1.0f;
    body.body.m_inertia = epxCalcInertiaBox(scale, 1.0f);
    physics->AddShape(scene, body.id, EpxShapeType::Box, scale);
  }

  return physics;
}

static std::shared_ptr<PhysicsScene> createSceneFriction(Geometry &scene) {
  auto physics = std::make_shared<PhysicsScene>("friction test");

  const EpxFloat angle = 0.4f;

  // 地面
  {
    auto body = physics->AddBody();
    EpxVector3 scale(10.0f, 0.5f, 10.0f);
    body.state.m_motionType = EpxMotionTypeStatic;
    body.state.m_position = EpxVector3(0.0f, -scale[1], 0.0f);
    body.state.m_orientation = EpxQuat::rotationX(angle);
    body.body.m_friction = 0.4f;
    physics->AddShape(scene, body.id, EpxShapeType::Box, scale);
  }

  const EpxVector3 brickScale(0.5f, 0.125f, 0.5f);

  for (int i = 0; i < 5; i++) {
    auto body = physics->AddBody();
    // 剛体を表現するための各種データを初期化
    body.state.m_position = EpxVector3(2.0f * (i - 2), 3.0f, 0.0f);
    body.state.m_orientation = EpxQuat::rotationX(angle);
    body.body.m_mass = 2.0f;
    body.body.m_inertia = epxCalcInertiaBox(brickScale, 2.0f);
    body.body.m_friction = 0.25f * i;
    physics->AddShape(scene, body.id, EpxShapeType::Box, brickScale);
  }

  return physics;
}

static std::shared_ptr<PhysicsScene> createSceneRestitution(Geometry &scene) {
  auto physics = std::make_shared<PhysicsScene>("restitution test");

  // 地面
  {
    auto body = physics->AddBody();
    EpxVector3 scale(10.0f, 0.5f, 10.0f);
    body.state.m_motionType = EpxMotionTypeStatic;
    body.state.m_position = EpxVector3(0.0f, -scale[1], 0.0f);
    physics->AddShape(scene, body.id, EpxShapeType::Box, scale);
  }

  const EpxVector3 scale(0.5f);

  for (int i = 0; i < 5; i++) {
    auto body = physics->AddBody();
    // 剛体を表現するための各種データを初期化
    body.state.m_position = EpxVector3(2.0f * (i - 2), 5.0f, 0.0f);
    body.body.m_mass = 2.0f;
    body.body.m_inertia = epxCalcInertiaSphere(1.0f, 2.0f);
    body.body.m_restitution = 0.25f * i;
    physics->AddShape(scene, body.id, EpxShapeType::Box, scale);
  }

  return physics;
}

static std::shared_ptr<PhysicsScene> createSceneGeometries(Geometry &scene) {
  auto physics = std::make_shared<PhysicsScene>("various convex shapes");

  // 地面
  {
    auto body = physics->AddBody();
    EpxVector3 scale(10.0f, 1.0f, 10.0f);
    body.state.m_motionType = EpxMotionTypeStatic;
    body.state.m_position = EpxVector3(0.0f, -scale[1], 0.0f);
    physics->AddShape(scene, body.id, EpxShapeType::Box, scale);
  }

  srand(9999);

  const int width = 5;
  for (int i = 0; i < width; i++) {
    for (int j = 0; j < width; j++) {
      auto body = physics->AddBody();

      EpxVector3 scale(0.1f + (rand() % 90) / 100.0f,
                       0.1f + (rand() % 90) / 100.0f,
                       0.1f + (rand() % 90) / 100.0f);

      body.state.m_position =
          EpxVector3(2 * (j - width / 2), 2.0f + 2 * i, 0.0f);
      body.body.m_mass = 1.0f;
      body.body.m_inertia = epxCalcInertiaBox(scale, 1.0f);

      switch ((i * width + j) % 4) {
      case 0:
        physics->AddShape(scene, body.id, EpxShapeType::Sphere, scale);
        break;

      case 1:
        physics->AddShape(scene, body.id, EpxShapeType::Box, scale);
        break;

      case 2:
        physics->AddShape(scene, body.id, EpxShapeType::Cylinder, scale);
        break;

      case 3:
        physics->AddShape(scene, body.id, EpxShapeType::Tetrahedron, scale);
        break;
      }
    }
  }

  return physics;
}

static const int maxScenes = 4;
std::shared_ptr<PhysicsScene> physicsCreateScene(int sceneId, Geometry &scene) {
  std::shared_ptr<PhysicsScene> physics;
  switch (sceneId % maxScenes) {
  case 0:
    physics = createSceneTwoBox(scene);
    break;

  case 1:
    physics = createSceneFriction(scene);
    break;

  case 2:
    physics = createSceneRestitution(scene);
    break;

  case 3:
    physics = createSceneGeometries(scene);
    break;

  default:
    throw std::out_of_range("0-3");
  }

  physics->CreateFireBody(scene);
  return physics;
}
