#include <common/Gl1Renderer.h>
#include <common/common.h>
#include <common/PhysicsScene.h>
#include <stdexcept>

using namespace EasyPhysics;

// シミュレーション定義
const float contactBias = 0.1f;
const float contactSlop = 0.001f;
const int iteration = 10;
const EpxVector3 gravity(0.0f, -9.8f, 0.0f);
const float timeStep = 0.016f;

// ブロードフェーズコールバック
EpxBool PhysicsScene::broadPhaseCallback(EpxUInt32 rigidBodyIdA,
                                         EpxUInt32 rigidBodyIdB,
                                         void *userData) {
  // ジョイントで接続された剛体のペアは作成しない
  for (EpxUInt32 i = 0; i < g_numJoints; i++) {
    if ((joints[i].rigidBodyA == rigidBodyIdA &&
         joints[i].rigidBodyB == rigidBodyIdB) ||
        (joints[i].rigidBodyA == rigidBodyIdB &&
         joints[i].rigidBodyB == rigidBodyIdA)) {
      return false;
    }
  }
  return true;
}

class DefaultAllocator : public EpxAllocator {
public:
  void *allocate(size_t bytes) { return malloc(bytes); }

  void deallocate(void *p) { free(p); }
} allocator;

void Perf::begin() { count = perfGetCount(); }

void Perf::end(const char *msg) {
  unsigned long long count2 = perfGetCount();
  float msec = perfGetTimeMillisecond(count, count2);
  if (frame % 100 == 0) {
    epxPrintf("%s : %.2f msec\n", msg, msec);
  }
}

PhysicsBody PhysicsScene::AddBody() {
  auto id = g_numRigidBodies++;
  bodies[id].reset();
  states[id].reset();
  collidables[id].reset();
  return {
      id,
      bodies[id],
      states[id],
      collidables[id],
  };
}

EpxShape *PhysicsScene::AddShape(int id, EpxShapeType type,
                                 const EpxVector3 &scale, bool finish) {
  // 凸メッシュを作成
  EpxShape shape;
  epxCreateConvexMesh(&shape.m_geometry, type, scale);

  // 凸メッシュ形状を登録
  auto added = collidables[id].addShape(shape);
  if (finish) {
    collidables[id].finish();
  }
  return added;
}

void PhysicsScene::FinishShape(int id) { collidables[id].finish(); }

void PhysicsScene::Simulate() {
  auto perf = state_.NewFrame();
  ApplyForce(perf);
  BroadPhase(perf);
  Collision(perf);
  Solver(perf);
  Integrate(perf);
  // epxPrintf("--- frame %d -------------\n",frame);
  // for(int i=0;i<numPairs[pairSwap];i++) {
  //	EpxPair &p = pairs[pairSwap][i];
  //	epxPrintf("RB %u,%u CP
  //%u\n",p.rigidBodyA,p.rigidBodyB,p.contact->m_numContacts);
  // }
  state_.EndFrame();
}

void PhysicsScene::ApplyForce(Perf &perf) {
  perf.begin();
  for (EpxUInt32 i = 0; i < g_numRigidBodies; i++) {
    EpxVector3 externalForce = gravity * bodies[i].m_mass;
    EpxVector3 externalTorque(0.0f);
    epxApplyExternalForce(states[i], bodies[i], externalForce, externalTorque,
                          timeStep);
  }
  perf.end("apply force");
}

void PhysicsScene::BroadPhase(Perf &perf) {
  perf.begin();
  state_.CurrentPair().numPairs = epxBroadPhase(
      {states, g_numRigidBodies}, {collidables, g_numRigidBodies},
      state_.OtherPair().Span(), state_.CurrentPair().MaxSpan(), &allocator,
      NULL,
      std::bind(&PhysicsScene::broadPhaseCallback, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));
  perf.end("broadphase");
}

void PhysicsScene::Collision(Perf &perf) {
  perf.begin();
  epxDetectCollision(states, collidables, g_numRigidBodies,
                     state_.CurrentPair().Span());
  perf.end("collision");
}

void PhysicsScene::Solver(Perf &perf) {
  perf.begin();
  epxSolveConstraints(
      states, bodies, g_numRigidBodies, state_.CurrentPair().Span(), joints,
      g_numJoints, iteration, contactBias, contactSlop, timeStep, &allocator);
  perf.end("solver");
}

void PhysicsScene::Integrate(Perf &perf) {
  perf.begin();
  epxIntegrate(states, g_numRigidBodies, timeStep);
  perf.end("integrate");
}

void PhysicsScene::CreateFireBody() {
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
  epxCreateConvexMesh(&shape.m_geometry, EpxShapeType::Sphere, scale);

  collidables[fireRigidBodyId].addShape(shape);
  collidables[fireRigidBodyId].finish();
}

void PhysicsScene::PhysicsFire(const EasyPhysics::EpxVector3 &position,
                               const EasyPhysics::EpxVector3 &velocity) {
  epxPrintf("fire pos %f %f %f vel %f %f %f\n", position[0], position[1],
            position[2], velocity[0], velocity[1], velocity[2]);
  states[fireRigidBodyId].m_motionType = EpxMotionTypeActive;
  states[fireRigidBodyId].m_position = position;
  states[fireRigidBodyId].m_linearVelocity = velocity;
  states[fireRigidBodyId].m_angularVelocity = EpxVector3(0.0f);
}

DrawDataSpan PhysicsScene::GetDrawData() const {

  draw_.Clear();

  for (int i = 0; i < g_numRigidBodies; i++) {
    auto &state = states[i];
    auto &collidable = collidables[i];

    EasyPhysics::EpxTransform3 rigidBodyTransform(state.m_orientation,
                                                  state.m_position);

    for (int j = 0; j < collidable.m_numShapes; j++) {
      auto &shape = collidable.m_shapes[j];
      auto wMtx = shape.WorldShapeMatrix(rigidBodyTransform);
      draw_.Push(shape.m_geometry.m_shapeType, wMtx);
    }
  }

  // 衝突点の表示
  const EasyPhysics::EpxVector3 colorA(1, 0, 0);
  const EasyPhysics::EpxVector3 colorB(0, 0, 1);
  const EasyPhysics::EpxVector3 colorLine(0, 1, 1);

  for (int i = 0; i < state_.physicsGetNumContacts(); i++) {
    auto &contact = state_.physicsGetContact(i);
    auto &stateA = states[state_.physicsGetRigidBodyAInContact(i)];
    auto &stateB = states[state_.physicsGetRigidBodyBInContact(i)];
    for (unsigned int j = 0; j < contact.m_numContacts; j++) {
      auto &contactPoint = contact.m_contactPoints[j];
      auto pointA =
          stateA.m_position + rotate(stateA.m_orientation, contactPoint.pointA);
      auto pointB =
          stateB.m_position + rotate(stateB.m_orientation, contactPoint.pointB);
      auto normal = contactPoint.normal;

      draw_.PushPoint({pointA, colorA});
      draw_.PushPoint({pointB, colorB});
      draw_.PushLine({pointA, pointA + 0.1f * normal, colorLine});
      draw_.PushLine({pointB, pointB - 0.1f * normal, colorLine});
    }
  }

  return draw_.Span();
}
