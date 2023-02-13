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

#include "high_level/pfx_high_level_include.h"

using namespace sce::PhysicsEffects;

//J シミュレーション
//E Simulation
bool physicsInit();
void physicsRelease();
void physicsCreateScene(int sceneId);
void physicsSimulate();
void physicsResetWorld();

//J ピッキング
//E picking
PfxVector3 physicsPickStart(const PfxVector3 &p1,const PfxVector3 &p2);
void physicsPickUpdate(const PfxVector3 &p);
void physicsPickEnd();

// Get ray information
int physicsGetNumRays();
const PfxRayInput& physicsGetRayInput(int id);
const PfxRayOutput& physicsGetRayOutput(int id);

// Get render mesh index
int physicsGetRenderMeshId(void *collisionMesh);

#endif /* __RENDER_FUNC_H__ */
