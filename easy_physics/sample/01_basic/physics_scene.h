#pragma once
#include <EpxInclude.h>
#include <string>

struct Perf {
  int frame;
  unsigned long long count;
  void setFrame(int f) { frame = f; }
  void begin();
  void end(const char *msg);
};

struct PhysicsState {
  const static int maxPairs = 5000;
  struct PhysicsPair {
    EasyPhysics::EpxUInt32 numPairs;
    EasyPhysics::EpxPair pairs[maxPairs];
    std::span<EasyPhysics::EpxPair> Span() { return {pairs, numPairs}; }
    std::span<EasyPhysics::EpxPair> MaxSpan() { return {pairs, maxPairs}; }
  };
  unsigned int g_pairSwap = 0;
  PhysicsPair g_pairs[2];
  int g_frame = 0;

  void Clear() {
    g_frame = 0;
    g_pairs[0].numPairs = g_pairs[1].numPairs = 0;
    g_pairSwap = 0;
  }
  Perf NewFrame() {
    g_pairSwap = 1 - g_pairSwap;
    return {
        .frame = g_frame,
    };
  }
  void EndFrame() { g_frame++; }
  PhysicsPair &CurrentPair() { return g_pairs[g_pairSwap]; }
  PhysicsPair &OtherPair() { return g_pairs[1 - g_pairSwap]; }

  int physicsGetNumContacts() { return g_pairs[g_pairSwap].numPairs; }

  const EasyPhysics::EpxContact &physicsGetContact(int i) {
    return *g_pairs[g_pairSwap].pairs[i].contact;
  }

  uint32_t physicsGetRigidBodyAInContact(int i) {
    return g_pairs[g_pairSwap].pairs[i].rigidBodyA;
  }

  uint32_t physicsGetRigidBodyBInContact(int i) {
    return g_pairs[g_pairSwap].pairs[i].rigidBodyB;
  }
};

struct PhysicsBody {
  EasyPhysics::EpxUInt32 id;
  EasyPhysics::EpxRigidBody &body;
  EasyPhysics::EpxState &state;
  EasyPhysics::EpxCollidable &collidable;
};

struct PhysicsScene {
  std::string title_;

  const static int maxRigidBodies = 500;
  const static int maxJoints = 100;

  // rigid bodies
  EasyPhysics::EpxState states[maxRigidBodies];
  EasyPhysics::EpxRigidBody bodies[maxRigidBodies];
  EasyPhysics::EpxCollidable collidables[maxRigidBodies];
  EasyPhysics::EpxUInt32 g_numRigidBodies = 0;

  // joints
  EasyPhysics::EpxBallJoint joints[maxJoints];
  EasyPhysics::EpxUInt32 g_numJoints = 0;

  PhysicsScene(const char *name) : title_(name) {}

  // build
  PhysicsBody AddBody();
  void AddBoxShape(class Renderer *renderer, int id,
                   const EasyPhysics::EpxVector3 &scale);
  void AddSphereShape(Renderer *renderer, int id,
                      const EasyPhysics::EpxVector3 &scale);
  void AddCylinderShape(Renderer *renderer, int id,
                        const EasyPhysics::EpxVector3 &scale);
  void AddTetrahedronShape(Renderer *renderer, int id,
                           const EasyPhysics::EpxVector3 &scale);

  // simulation
  void ApplyForce(Perf &perf);
  void BroadPhase(Perf &perf, PhysicsState &state);
  void Collision(Perf &perf, PhysicsState &g_state);
  void Solver(Perf &perf, PhysicsState &g_state);
  void Integrate(Perf &perf);

  // fire
  int fireRigidBodyId;
  void CreateFireBody(Renderer *renderer);
  void PhysicsFire(const EasyPhysics::EpxVector3 &position,
                   const EasyPhysics::EpxVector3 &velocity);
};
