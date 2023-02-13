/*
Physics Effects Copyright(C) 2012 Sony Computer Entertainment Inc.
All rights reserved.

Physics Effects is open software; you can redistribute it and/or
modify it under the terms of the BSD License.

Physics Effects is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the BSD License for more details.

A copy of the BSD License is distributed with
Physics Effects under the filename: physics_effects_license.txt
*/

#ifndef __PHYSICS_FUNC_H__
#define __PHYSICS_FUNC_H__

#define SCE_PFX_USE_PERFCOUNTER

#include "physics_effects.h"

using namespace sce::PhysicsEffects;

//E Simulation
//J シミュレーション
bool physicsInit();
void physicsRelease();
void physicsCreateScene(int sceneId);
void physicsSimulate();

//E Picking
//J ピッキング
PfxVector3 physicsPickStart(const PfxVector3 &p1,const PfxVector3 &p2);
void physicsPickUpdate(const PfxVector3 &p);
void physicsPickEnd();

//E Change parameters
//J パラメータの取得
int physicsGetNumRigidbodies();
const PfxRigidState& physicsGetState(int id);
const PfxRigidBody& physicsGetBody(int id);
const PfxCollidable& physicsGetCollidable(int id);

// Get contact information
int physicsGetNumContacts();
const PfxContactManifold &physicsGetContact(int id);

// Get simulation island
const PfxIsland* physicsGetIslands();

#endif /* __PHYSICS_FUNC_H__ */
