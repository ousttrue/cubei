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

#include "physics_effects.h"

using namespace sce::PhysicsEffects;

#ifndef __CREATE_RENDER_MESH_H__
#define __CREATE_RENDER_MESH_H__

// Create a render mesh from PfxConvexMesh
int createRenderMesh(PfxConvexMesh *convexMesh);

// Create a render mesh from PfxLargeTriMesh
int createRenderMesh(PfxLargeTriMesh *largeMesh);

#endif /* __CREATE_RENDER_MESH_H__ */
