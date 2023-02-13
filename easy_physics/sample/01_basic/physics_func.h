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

#ifndef PHYSICS_FUNC_H
#define PHYSICS_FUNC_H

#include "../../easy_physics/EpxInclude.h"

struct PhysicsScene {};

const int maxPairs = 5000;
struct PhysicsState {
  struct PhysicsPair {
    EasyPhysics::EpxUInt32 numPairs;
    EasyPhysics::EpxPair pairs[maxPairs];
    std::span<EasyPhysics::EpxPair> Span() { return {pairs, numPairs}; }
    std::span<EasyPhysics::EpxPair> MaxSpan() { return {pairs, maxPairs}; }
  };
  unsigned int g_pairSwap = 0;
  PhysicsPair g_pairs[2];

  void Clear() {
    g_pairs[0].numPairs = g_pairs[1].numPairs = 0;
    g_pairSwap = 0;
  }
  void NewFrame() { g_pairSwap = 1 - g_pairSwap; }
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

// シミュレーション関数
bool physicsInit();
void physicsRelease();
void physicsSimulate();

void physicsCreateScene(class Renderer *renderer);
void physicsNextScene();
// シーンのタイトル名を取得する
const char *physicsGetSceneTitle();

// 剛体に関連するデータを取得する関数
int physicsGetNumRigidbodies();
const EasyPhysics::EpxState &physicsGetState(int i);
const EasyPhysics::EpxRigidBody &physicsGetRigidBody(int i);
const EasyPhysics::EpxCollidable &physicsGetCollidable(int i);

// 衝突情報を取得する関数
int physicsGetNumContacts();
const EasyPhysics::EpxContact &physicsGetContact(int i);
EasyPhysics::EpxUInt32 physicsGetRigidBodyAInContact(int i);
EasyPhysics::EpxUInt32 physicsGetRigidBodyBInContact(int i);

// ワールドへの干渉
void physicsFire(const EasyPhysics::EpxVector3 &position,
                 const EasyPhysics::EpxVector3 &velocity);

#endif // PHYSICS_FUNC_H
