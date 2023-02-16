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

std::shared_ptr<PhysicsScene> createSceneBallJoint() {
  auto physics = std::make_shared<PhysicsScene>("ball joint");

  // 地面
  {
    auto body = physics->AddBody();
    EpxVector3 scale(10.0f, 1.0f, 10.0f);
    body.state.m_motionType = EpxMotionTypeStatic;
    body.state.m_position = EpxVector3(0.0f, -scale[1], 0.0f);
    physics->AddShape(body.id, EasyPhysics::EpxShapeType::Box, scale);
  }

  {
    auto body = physics->AddBody();
    EpxVector3 scale(0.25f, 0.25f, 2.0f);
    body.state.m_position = EpxVector3(0.0f, scale[1], 0.0f);
    body.body.m_mass = 1.0f;
    body.body.m_inertia = epxCalcInertiaBox(scale, 1.0f);
    physics->AddShape(body.id, EpxShapeType::Box, scale);
  }

  // ボールジョイントにより接続されたボックス
  auto joint = physics->AddJoint();

  {
    auto body = physics->AddBody();
    EpxVector3 scale(1.0f, 0.25f, 0.25f);
    body.state.m_position = EpxVector3(-scale[0], 3.0f, 0.0f);
    body.body.m_mass = 1.0f;
    body.body.m_inertia = epxCalcInertiaBox(scale, 1.0f);
    physics->AddShape(body.id, EpxShapeType::Cylinder, scale);
    joint->rigidBodyA = body.id;
    joint->anchorA = EpxVector3(scale[0], 0.0f, 0.0f);
  }

  {
    auto body = physics->AddBody();
    EpxVector3 scale(1.0f, 0.25f, 0.25f);
    body.state.m_position = EpxVector3(scale[0], 3.0f, 0.0f);
    body.body.m_mass = 1.0f;
    body.body.m_inertia = epxCalcInertiaBox(scale, 1.0f);
    physics->AddShape(body.id, EpxShapeType::Cylinder, scale);
    joint->rigidBodyB = body.id;
    joint->anchorB = EpxVector3(-scale[0], 0.0f, 0.0f);
  }

  return physics;
}

std::shared_ptr<PhysicsScene> createSceneHingeJoint() {
  auto physics = std::make_shared<PhysicsScene>("hinge joint");

  // ボールジョイントx2でヒンジジョイントを再現
  auto joint0 = physics->AddJoint();
  auto joint1 = physics->AddJoint();

  {
    auto body = physics->AddBody();
    EpxVector3 scale(1.25f, 0.25f, 0.25f);
    body.state.m_motionType = EpxMotionTypeStatic;
    body.state.m_position = EpxVector3(0.0f, 1.0f, 0.0f);
    auto shape =
        physics->AddShape(body.id, EpxShapeType::Cylinder, scale, false);
    shape->m_offsetOrientation = EpxQuat::rotationZ(0.5f * EPX_PI);
    physics->FinishShape(body.id);
    joint0->rigidBodyA = body.id;
    joint0->anchorA = EpxVector3(0.0f, 1.0f, 0.0f);
    joint1->rigidBodyA = body.id;
    joint1->anchorA = EpxVector3(0.0f, -1.0f, 0.0f);
  }

  {
    auto body = physics->AddBody();
    EpxVector3 scale(2.0f, 1.0f, 0.25f);
    body.state.m_position = EpxVector3(0.0f, 1.0f, 0.0f);
    body.state.m_angularVelocity = EpxVector3(0.0f, 10.0f, 0.0f);
    body.body.m_mass = 10.0f;
    // 回転方向に安定するように慣性テンソル計算用の形状を調整
    body.body.m_inertia =
        epxCalcInertiaBox(EpxVector3(0.5f, 3.0f, 0.5f), 10.0f);
    physics->AddShape(body.id, EpxShapeType::Box, scale);
    joint0->rigidBodyB = body.id;
    joint0->anchorB = EpxVector3(0.0f, 1.0f, 0.0f);
    joint1->rigidBodyB = body.id;
    joint1->anchorB = EpxVector3(0.0f, -1.0f, 0.0f);
  }

  return physics;
}

std::shared_ptr<PhysicsScene> createSceneFixedJoint() {
  auto physics = std::make_shared<PhysicsScene>("fixed joint");

  // ボールジョイントx3で固定ジョイントを再現
  auto joint0 = physics->AddJoint();
  auto joint1 = physics->AddJoint();
  auto joint2 = physics->AddJoint();

  {
    auto body = physics->AddBody();
    EpxVector3 scale(1.0f, 2.5f, 2.5f);
    body.state.m_motionType = EpxMotionTypeStatic;
    body.state.m_position = EpxVector3(-1.0f, 1.0f, 0.0f);
    physics->AddShape(body.id, EpxShapeType::Box, scale);
    joint0->rigidBodyA = body.id;
    joint1->rigidBodyA = body.id;
    joint2->rigidBodyA = body.id;
    joint0->anchorA = EpxVector3(2.0f, 5.0f, 0.0f);
    joint1->anchorA = EpxVector3(2.0f, -5.0f, 1.0f);
    joint2->anchorA = EpxVector3(2.0f, -5.0f, -1.0f);
  }

  {
    auto body = physics->AddBody();
    EpxVector3 scale(1.0f, 0.25f, 0.25f);
    body.state.m_position = EpxVector3(1.0f, 1.0f, 0.0f);
    body.body.m_mass = 10.0f;
    // 安定するように慣性テンソル計算用の形状を大きめに設定
    body.body.m_inertia = epxCalcInertiaBox(10.0f * scale, 10.0f);
    physics->AddShape(body.id, EpxShapeType::Box, scale);
    joint0->rigidBodyB = body.id;
    joint1->rigidBodyB = body.id;
    joint2->rigidBodyB = body.id;
    // 固定ジョイントが安定するように、接続位置を剛体の重心に配置
    joint0->anchorB = EpxVector3(0.0f, 5.0f, 0.0f);
    joint1->anchorB = EpxVector3(0.0f, -5.0f, 1.0f);
    joint2->anchorB = EpxVector3(0.0f, -5.0f, -1.0f);
  }

  return physics;
}

int createGear(const std::shared_ptr<PhysicsScene> &physics,
               const EpxVector3 &offsetPosition,
               const EpxQuat &offsetOrientation) {
  EpxFloat cogsWidth = 0.25f;

  // ボールジョイントx2でヒンジジョイントを再現
  auto joint0 = physics->AddJoint();
  auto joint1 = physics->AddJoint();

  {
    auto body = physics->AddBody();
    EpxVector3 scale(0.3f, 0.5f, 0.5f);
    body.state.m_motionType = EpxMotionTypeStatic;
    body.state.m_position = offsetPosition + EpxVector3(0.0f, 1.0f, 0.0f);
    body.state.m_orientation = offsetOrientation;
    auto shape =
        physics->AddShape(body.id, EpxShapeType::Cylinder, scale, false);
    shape->m_offsetOrientation = EpxQuat::rotationY(0.5f * EPX_PI);
    physics->FinishShape(body.id);
    joint0->rigidBodyA = body.id;
    joint0->anchorA = EpxVector3(0.0f, 0.0f, 5.0f);
    joint1->rigidBodyA = body.id;
    joint1->anchorA = EpxVector3(0.0f, 0.0f, -5.0f);
  }

  {
    auto body = physics->AddBody();
    EpxVector3 scale(0.25f, 2.0f, 2.0f);
    body.state.m_position = offsetPosition + EpxVector3(0.0f, 1.0f, 0.0f);
    body.state.m_orientation = offsetOrientation;
    body.body.m_mass = 10.0f;
    // 回転方向に安定するように慣性テンソル計算用の形状を調整
    body.body.m_inertia =
        epxCalcInertiaBox(EpxVector3(2.5f, 2.5f, 25.0f), 10.0f);

    {
      auto shape =
          physics->AddShape(body.id, EpxShapeType::Cylinder, scale, false);
      shape->m_offsetOrientation = EpxQuat::rotationY(0.5f * EPX_PI);
    }
    for (int i = 0; i < 4; i++) {
      const EpxVector3 cogsScale(2.5f, cogsWidth, 0.25f);
      auto shape =
          physics->AddShape(body.id, EpxShapeType::Box, cogsScale, false);
      shape->m_offsetOrientation = EpxQuat::rotationZ(i * 0.25f * EPX_PI);
    }
    physics->FinishShape(body.id);

    // ギアジョイントが安定するように、接続位置を広めに配置
    joint0->rigidBodyB = body.id;
    joint0->anchorB = EpxVector3(0.0f, 0.0f, 5.0f);
    joint1->rigidBodyB = body.id;
    joint1->anchorB = EpxVector3(0.0f, 0.0f, -5.0f);

    return body.id;
  }
}

std::shared_ptr<PhysicsScene> createSceneGearJoint() {
  auto physics = std::make_shared<PhysicsScene>("gear joint");

  // ギア大
  createGear(physics, EpxVector3(2.0f, 1.0f, 0.0f), EpxQuat::identity());

  // ギア小
  int gearId = createGear(physics, EpxVector3(-2.5f, 1.0f, 0.0f),
                          EpxQuat::rotationZ(0.125f * EPX_PI));
  physics->states[gearId].m_angularVelocity = EpxVector3(0.0f, 0.0f, -10.0f);

  return physics;
}

std::shared_ptr<PhysicsScene> createSceneChain() {
  auto physics = std::make_shared<PhysicsScene>("chain joint");

  const EpxVector3 chainScale(0.125f, 0.5f, 0.125f);
  const EpxVector3 ballScale(1.5f);

  // ダミー
  // int dummyId;
  auto dummy = physics->AddBody();
  {
    EpxVector3 scale(0.01f);
    dummy.state.m_motionType = EpxMotionTypeStatic;
    dummy.state.m_position = EpxVector3(0.0f, 0.0f, 0.0f);
    physics->AddShape(dummy.id, EpxShapeType::Box, scale);
  }

  // 1kgの鎖の先に100kgの錘を接続

  // 慣性テンソル調整なし
  for (int i = 0; i < 5; i++) {
    auto body = physics->AddBody();
    body.state.m_position = EpxVector3(-2.5f, 5.0f, 0.0f) +
                            EpxVector3(0.0f, -i * chainScale[1] * 2.0f, 0.0f);
    body.body.m_mass = 1.0f;
    body.body.m_inertia = epxCalcInertiaBox(chainScale, 1.0f);
    physics->AddShape(body.id, EpxShapeType::Box, chainScale);

    auto joint = physics->AddJoint();
    if (i > 0) {
      joint->rigidBodyA = body.id - 1;
      joint->anchorA = EpxVector3(0.0f, -chainScale[1], 0.0f);
    } else {
      joint->rigidBodyA = dummy.id;
      joint->anchorA =
          body.state.m_position + EpxVector3(0.0f, chainScale[1], 0.0f);
    }
    joint->rigidBodyB = body.id;
    joint->anchorB = EpxVector3(0.0f, chainScale[1], 0.0f);
  }

  {
    auto body = physics->AddBody();
    body.state.m_position =
        physics->states[body.id - 1].m_position +
        EpxVector3(0.0f, -chainScale[1] - ballScale[1], 0.0f);
    body.body.m_mass = 10.0f;
    body.body.m_inertia = epxCalcInertiaSphere(ballScale[1], 10.0f);
    physics->AddShape(body.id, EpxShapeType::Sphere, ballScale);

    auto joint = physics->AddJoint();
    joint->rigidBodyA = body.id - 1;
    joint->rigidBodyB = body.id;
    joint->anchorA = EpxVector3(0.0f, -chainScale[1], 0.0f);
    joint->anchorB = EpxVector3(0.0f, ballScale[1], 0.0f);
  }

  // 慣性テンソル調整あり
  for (int i = 0; i < 5; i++) {
    auto body = physics->AddBody();
    body.state.m_position = EpxVector3(2.5f, 5.0f, 0.0f) +
                            EpxVector3(0.0f, -i * chainScale[1] * 2.0f, 0.0f);
    body.body.m_mass = 1.0f;
    body.body.m_inertia =
        epxCalcInertiaBox(15.0f * chainScale, 1.0f); // 慣性テンソル増加
    physics->AddShape(body.id, EpxShapeType::Box, chainScale);

    auto joint = physics->AddJoint();
    if (i > 0) {
      joint->rigidBodyA = body.id - 1;
      joint->anchorA = EpxVector3(0.0f, -chainScale[1], 0.0f);
    } else {
      joint->rigidBodyA = dummy.id;
      joint->anchorA =
          body.state.m_position + EpxVector3(0.0f, chainScale[1], 0.0f);
    }
    joint->rigidBodyB = body.id;
    joint->anchorB = EpxVector3(0.0f, chainScale[1], 0.0f);
  }

  {
    auto body = physics->AddBody();

    body.state.m_position =
        physics->states[body.id - 1].m_position +
        EpxVector3(0.0f, -chainScale[1] - ballScale[1], 0.0f);

    body.body.m_mass = 10.0f;
    body.body.m_inertia = epxCalcInertiaSphere(ballScale[1], 10.0f);

    physics->AddShape(body.id, EpxShapeType::Sphere, ballScale);

    auto joint = physics->AddJoint();
    joint->rigidBodyA = body.id - 1;
    joint->rigidBodyB = body.id;
    joint->anchorA = EpxVector3(0.0f, -chainScale[1], 0.0f);
    joint->anchorB = EpxVector3(0.0f, ballScale[1], 0.0f);
  }

  return physics;
}
