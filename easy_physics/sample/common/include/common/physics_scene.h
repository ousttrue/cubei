#pragma once
#include "Gl1Renderer.h"
#include <EpxInclude.h>
#include <common/DrawData.h>
#include <string>
#include <vector>

struct Perf {
  int frame;
  unsigned long long count;
  void setFrame(int f) { frame = f; }
  void begin();
  void end(const char *msg);
};

struct PhysicsBody {
  EasyPhysics::EpxUInt32 id;
  EasyPhysics::EpxRigidBody &body;
  EasyPhysics::EpxState &state;
  EasyPhysics::EpxCollidable &collidable;
};

struct PhysicsScene {
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

    PhysicsState() {
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

    int physicsGetNumContacts() const { return g_pairs[g_pairSwap].numPairs; }

    const EasyPhysics::EpxContact &physicsGetContact(int i) const {
      return *g_pairs[g_pairSwap].pairs[i].contact;
    }

    uint32_t physicsGetRigidBodyAInContact(int i) const {
      return g_pairs[g_pairSwap].pairs[i].rigidBodyA;
    }

    uint32_t physicsGetRigidBodyBInContact(int i) const {
      return g_pairs[g_pairSwap].pairs[i].rigidBodyB;
    }
  };
  PhysicsState state_;
  DrawData data_;
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

  //
  PhysicsScene(const char *name) : title_(name) {}

  // build
  PhysicsBody AddBody();
  EasyPhysics::EpxShape *AddShape(int id, EasyPhysics::EpxShapeType type,
                                  const EasyPhysics::EpxVector3 &scale,
                                  bool finish = true);

  // simulation
  void Simulate();
  void ApplyForce(Perf &perf);
  void BroadPhase(Perf &perf);
  void Collision(Perf &perf);
  void Solver(Perf &perf);
  void Integrate(Perf &perf);

  // fire
  int fireRigidBodyId;
  void CreateFireBody();
  void PhysicsFire(const EasyPhysics::EpxVector3 &position,
                   const EasyPhysics::EpxVector3 &velocity);

  // draw data
  mutable std::vector<Shape> shapes_;
  mutable std::vector<Point> points_;
  mutable std::vector<Line> lines_;
  void ClearDrawData() const {
    shapes_.clear();
    points_.clear();
    lines_.clear();
  }
  DrawData GetDrawData() const;
};
