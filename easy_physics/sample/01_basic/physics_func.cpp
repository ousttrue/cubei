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
#include "../common/common.h"
#include "../common/geometry_data.h"
#include <stdlib.h>
#include <tuple>

using namespace EasyPhysics;

///////////////////////////////////////////////////////////////////////////////
// シミュレーション定義

const int maxRigidBodies = 500;
const int maxJoints = 100;

const float timeStep = 0.016f;
const float contactBias = 0.1f;
const float contactSlop = 0.001f;
const int iteration = 10;
const EpxVector3 gravity(0.0f, -9.8f, 0.0f);

///////////////////////////////////////////////////////////////////////////////
// シミュレーションデータ

// 剛体
EpxState states[maxRigidBodies];
EpxRigidBody bodies[maxRigidBodies];
EpxCollidable collidables[maxRigidBodies];
EpxUInt32 g_numRigidBodies = 0;

// ジョイント
EpxBallJoint joints[maxJoints];
EpxUInt32 g_numJoints = 0;

// state
PhysicsState g_state = {};

static int g_frame = 0;

///////////////////////////////////////////////////////////////////////////////
// メモリアロケータ

class DefaultAllocator : public EpxAllocator {
public:
  void *allocate(size_t bytes) { return malloc(bytes); }

  void deallocate(void *p) { free(p); }
} allocator;

///////////////////////////////////////////////////////////////////////////////
// シミュレーション関数

struct Perf {
  unsigned long long count;
  int frame;

  void setFrame(int f) { frame = f; }

  void begin() { count = perfGetCount(); }

  void end(const char *msg) {
    unsigned long long count2 = perfGetCount();
    float msec = perfGetTimeMillisecond(count, count2);
    if (frame % 100 == 0) {
      epxPrintf("%s : %.2f msec\n", msg, msec);
    }
  }
};

void physicsSimulate() {
  Perf perf;
  perf.setFrame(g_frame);

  g_state.NewFrame();

  perf.begin();
  for (EpxUInt32 i = 0; i < g_numRigidBodies; i++) {
    EpxVector3 externalForce = gravity * bodies[i].m_mass;
    EpxVector3 externalTorque(0.0f);
    epxApplyExternalForce(states[i], bodies[i], externalForce, externalTorque,
                          timeStep);
  }
  perf.end("apply force");

  perf.begin();
  g_state.CurrentPair().numPairs =
      epxBroadPhase({states, g_numRigidBodies}, {collidables, g_numRigidBodies},
                    g_state.OtherPair().Span(), g_state.CurrentPair().MaxSpan(),
                    &allocator, NULL, NULL);
  perf.end("broadphase");

  perf.begin();
  epxDetectCollision(states, collidables, g_numRigidBodies,
                     g_state.CurrentPair().Span());
  perf.end("collision");

  perf.begin();
  epxSolveConstraints(
      states, bodies, g_numRigidBodies, g_state.CurrentPair().Span(), joints,
      g_numJoints, iteration, contactBias, contactSlop, timeStep, &allocator);
  perf.end("solver");

  perf.begin();
  epxIntegrate(states, g_numRigidBodies, timeStep);
  perf.end("integrate");

  // epxPrintf("--- frame %d -------------\n",frame);
  // for(int i=0;i<numPairs[pairSwap];i++) {
  //	EpxPair &p = pairs[pairSwap][i];
  //	epxPrintf("RB %u,%u CP
  //%u\n",p.rigidBodyA,p.rigidBodyB,p.contact->m_numContacts);
  // }

  g_frame++;
}

///////////////////////////////////////////////////////////////////////////////
// シーンの作成

static int fireRigidBodyId;

static void createFireBody(Renderer *renderer) {
  fireRigidBodyId = g_numRigidBodies++;

  EpxVector3 scale(0.5f);

  states[fireRigidBodyId].reset();
  states[fireRigidBodyId].m_motionType = EpxMotionTypeStatic;
  states[fireRigidBodyId].m_position = EpxVector3(999.0f);
  bodies[fireRigidBodyId].reset();
  bodies[fireRigidBodyId].m_mass = 1.0f;
  bodies[fireRigidBodyId].m_inertia = epxCalcInertiaBox(scale, 1.0f);
  collidables[fireRigidBodyId].reset();

  EpxShape shape;
  shape.reset();

  epxCreateConvexMesh(&shape.m_geometry, sphere_vertices, sphere_numVertices,
                      sphere_indices, sphere_numIndices, scale);
  shape.userData = (void *)createRenderMesh(renderer, &shape.m_geometry);

  collidables[fireRigidBodyId].addShape(shape);
  collidables[fireRigidBodyId].finish();
}

static int AddBody() {
  int id = g_numRigidBodies++;
  states[id].reset();
  bodies[id].reset();
  collidables[id].reset();
  return id;
}

static void AddBoxShape(Renderer *renderer, int id, const EpxVector3 &scale) {
  // 凸メッシュを作成
  EpxShape shape;
  shape.reset();
  epxCreateConvexMesh(&shape.m_geometry, box_vertices, box_numVertices,
                      box_indices, box_numIndices, scale);

  // 同時に描画用メッシュを作成、ポインタをユーザーデータに保持
  // 描画用メッシュは、終了時に自動的に破棄される
  shape.userData = (void *)createRenderMesh(renderer, &shape.m_geometry);

  // 凸メッシュ形状を登録
  collidables[id].addShape(shape);
  collidables[id].finish();
}

static void AddSphereShape(Renderer *renderer, int id,
                           const EpxVector3 &scale) {
  // 凸メッシュを作成
  EpxShape shape;
  shape.reset();
  epxCreateConvexMesh(&shape.m_geometry, sphere_vertices, sphere_numVertices,
                      sphere_indices, sphere_numIndices, scale);

  // 同時に描画用メッシュを作成、ポインタをユーザーデータに保持
  // 描画用メッシュは、終了時に自動的に破棄される
  shape.userData = (void *)createRenderMesh(renderer, &shape.m_geometry);

  // 凸メッシュ形状を登録
  collidables[id].addShape(shape);
  collidables[id].finish();
}

static void AddCylinderShape(Renderer *renderer, int id,
                             const EpxVector3 &scale) {
  // 凸メッシュを作成
  EpxShape shape;
  shape.reset();
  epxCreateConvexMesh(&shape.m_geometry, cylinder_vertices,
                      cylinder_numVertices, cylinder_indices,
                      cylinder_numIndices, scale);

  // 同時に描画用メッシュを作成、ポインタをユーザーデータに保持
  // 描画用メッシュは、終了時に自動的に破棄される
  shape.userData = (void *)createRenderMesh(renderer, &shape.m_geometry);

  // 凸メッシュ形状を登録
  collidables[id].addShape(shape);
  collidables[id].finish();
}

static void AddTetrahedronShape(Renderer *renderer, int id,
                                const EpxVector3 &scale) {
  // 凸メッシュを作成
  EpxShape shape;
  shape.reset();
  epxCreateConvexMesh(&shape.m_geometry, tetrahedron_vertices,
                      tetrahedron_numVertices, tetrahedron_indices,
                      tetrahedron_numIndices, scale);

  // 同時に描画用メッシュを作成、ポインタをユーザーデータに保持
  // 描画用メッシュは、終了時に自動的に破棄される
  shape.userData = (void *)createRenderMesh(renderer, &shape.m_geometry);

  // 凸メッシュ形状を登録
  collidables[id].addShape(shape);
  collidables[id].finish();
}

static void createSceneTwoBox(Renderer *renderer) {
  // 地面
  {
    auto id = AddBody();
    EpxVector3 scale(10.0f, 1.0f, 10.0f);
    // 剛体を表現するための各種データを初期化
    states[id].m_motionType = EpxMotionTypeStatic;
    states[id].m_position = EpxVector3(0.0f, -scale[1], 0.0f);
    AddBoxShape(renderer, id, scale);
  }

  // ボックス
  {
    auto id = AddBody();
    EpxVector3 scale(2.0f, 0.25f, 1.0f);
    // 剛体を表現するための各種データを初期化
    states[id].m_position = EpxVector3(0.0f, scale[1], 0.0f);
    bodies[id].m_mass = 1.0f;
    bodies[id].m_inertia = epxCalcInertiaBox(scale, 1.0f);
    AddBoxShape(renderer, id, scale);
  }

  {
    auto id = AddBody();
    EpxVector3 scale(2.0f, 0.25f, 1.0f);
    // 剛体を表現するための各種データを初期化
    states[id].m_position = EpxVector3(0.0f, 3.0f, 0.0f);
    states[id].m_orientation =
        EpxQuat::rotationZ(2.0f) * EpxQuat::rotationY(0.7f);
    bodies[id].m_mass = 1.0f;
    bodies[id].m_inertia = epxCalcInertiaBox(scale, 1.0f);
    AddBoxShape(renderer, id, scale);
  }
}

static void createSceneFriction(Renderer *renderer) {
  const EpxFloat angle = 0.4f;

  // 地面
  {
    int id = AddBody();
    EpxVector3 scale(10.0f, 0.5f, 10.0f);
    states[id].m_motionType = EpxMotionTypeStatic;
    states[id].m_position = EpxVector3(0.0f, -scale[1], 0.0f);
    states[id].m_orientation = EpxQuat::rotationX(angle);
    bodies[id].m_friction = 0.4f;
    AddBoxShape(renderer, id, scale);
  }

  const EpxVector3 brickScale(0.5f, 0.125f, 0.5f);

  for (int i = 0; i < 5; i++) {
    int id = AddBody();
    // 剛体を表現するための各種データを初期化
    states[id].m_position = EpxVector3(2.0f * (i - 2), 3.0f, 0.0f);
    states[id].m_orientation = EpxQuat::rotationX(angle);
    bodies[id].m_mass = 2.0f;
    bodies[id].m_inertia = epxCalcInertiaBox(brickScale, 2.0f);
    bodies[id].m_friction = 0.25f * i;
    AddBoxShape(renderer, id, brickScale);
  }
}

static void createSceneRestitution(Renderer *renderer) {
  // 地面
  {
    int id = AddBody();
    EpxVector3 scale(10.0f, 0.5f, 10.0f);
    states[id].m_motionType = EpxMotionTypeStatic;
    states[id].m_position = EpxVector3(0.0f, -scale[1], 0.0f);
    AddBoxShape(renderer, id, scale);
  }

  const EpxVector3 scale(0.5f);

  for (int i = 0; i < 5; i++) {
    int id = AddBody();
    // 剛体を表現するための各種データを初期化
    states[id].m_position = EpxVector3(2.0f * (i - 2), 5.0f, 0.0f);
    bodies[id].m_mass = 2.0f;
    bodies[id].m_inertia = epxCalcInertiaSphere(1.0f, 2.0f);
    bodies[id].m_restitution = 0.25f * i;
    AddBoxShape(renderer, id, scale);
  }
}

static void createSceneGeometries(Renderer *renderer) {
  // 地面
  {
    int id = AddBody();
    EpxVector3 scale(10.0f, 1.0f, 10.0f);
    states[id].m_motionType = EpxMotionTypeStatic;
    states[id].m_position = EpxVector3(0.0f, -scale[1], 0.0f);
    AddBoxShape(renderer, id, scale);
  }

  srand(9999);

  const int width = 5;
  for (int i = 0; i < width; i++) {
    for (int j = 0; j < width; j++) {
      int id = AddBody();

      EpxVector3 scale(0.1f + (rand() % 90) / 100.0f,
                       0.1f + (rand() % 90) / 100.0f,
                       0.1f + (rand() % 90) / 100.0f);

      states[id].m_position =
          EpxVector3(2 * (j - width / 2), 2.0f + 2 * i, 0.0f);
      bodies[id].m_mass = 1.0f;
      bodies[id].m_inertia = epxCalcInertiaBox(scale, 1.0f);

      switch ((i * width + j) % 4) {
      case 0:
        AddSphereShape(renderer, id, scale);
        break;

      case 1:
        AddBoxShape(renderer, id, scale);
        break;

      case 2:
        AddCylinderShape(renderer, id, scale);
        break;

      case 3:
        AddTetrahedronShape(renderer, id, scale);
        break;
      }
    }
  }
}

int g_sceneId = 0;
static const int maxScenes = 4;
static const char titles[][32] = {
    "basic rigid bodies",
    "friction test",
    "restitution test",
    "various convex shapes",
};
void physicsNextScene() { ++g_sceneId; }
const char *physicsGetSceneTitle() { return titles[g_sceneId % maxScenes]; }

void physicsCreateScene(Renderer *renderer) {
  g_frame = 0;

  g_numRigidBodies = 0;
  g_numJoints = 0;
  g_state.Clear();

  switch (g_sceneId % maxScenes) {
  case 0:
    createSceneTwoBox(renderer);
    break;

  case 1:
    createSceneFriction(renderer);
    break;

  case 2:
    createSceneRestitution(renderer);
    break;

  case 3:
    createSceneGeometries(renderer);
    break;
  }

  createFireBody(renderer);
}

///////////////////////////////////////////////////////////////////////////////
// 初期化、解放

bool physicsInit() { return true; }

void physicsRelease() {}

///////////////////////////////////////////////////////////////////////////////
// 外部から物理データへのアクセス

int physicsGetNumRigidbodies() { return g_numRigidBodies; }

const EpxState &physicsGetState(int i) { return states[i]; }

const EpxRigidBody &physicsGetRigidBody(int i) { return bodies[i]; }

const EpxCollidable &physicsGetCollidable(int i) { return collidables[i]; }

int physicsGetNumContacts() { return g_state.physicsGetNumContacts(); }

const EasyPhysics::EpxContact &physicsGetContact(int i) {
  return g_state.physicsGetContact(i);
}

EpxUInt32 physicsGetRigidBodyAInContact(int i) {
  return g_state.physicsGetRigidBodyAInContact(i);
}

EpxUInt32 physicsGetRigidBodyBInContact(int i) {
  return g_state.physicsGetRigidBodyBInContact(i);
}

void physicsFire(const EasyPhysics::EpxVector3 &position,
                 const EasyPhysics::EpxVector3 &velocity) {
  states[fireRigidBodyId].m_motionType = EpxMotionTypeActive;
  states[fireRigidBodyId].m_position = position;
  states[fireRigidBodyId].m_linearVelocity = velocity;
  states[fireRigidBodyId].m_angularVelocity = EpxVector3(0.0f);
}
