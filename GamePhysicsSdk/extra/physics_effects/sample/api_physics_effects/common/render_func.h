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

#ifndef __RENDER_FUNC_H__
#define __RENDER_FUNC_H__

#include "common.h"
#include "physics_effects.h"

using namespace sce::PhysicsEffects;

#define	DISPLAY_WIDTH			640
#define	DISPLAY_HEIGHT			480

///////////////////////////////////////////////////////////////////////////////
// Draw Primitives

void renderInit(const char *title=NULL);
void renderRelease();
void renderBegin();
void renderEnd();

void renderSphere(
	const PfxTransform3 &transform,
	const PfxVector3 &color,
	const PfxFloatInVec &radius);

void renderBox(
	const PfxTransform3 &transform,
	const PfxVector3 &color,
	const PfxVector3 &halfExtent);

void renderCylinder(
	const PfxTransform3 &transform,
	const PfxVector3 &color,
	const PfxFloatInVec &radius,
	const PfxFloatInVec &halfLength);

void renderCapsule(
	const PfxTransform3 &transform,
	const PfxVector3 &color,
	const PfxFloatInVec &radius,
	const PfxFloatInVec &halfLength);

int renderInitMesh(
	const float *vtx,unsigned int vtxStrideBytes,
	const float *nml,unsigned int nmlStrideBytes,
	const unsigned short *tri,unsigned int triStrideBytes,
	int numVtx,int numTri);

void renderReleaseMeshAll();

void renderMesh(
	const PfxTransform3 &transform,
	const PfxVector3 &color,
	int meshId);

///////////////////////////////////////////////////////////////////////////////
// Debug Drawing

void renderDebugBegin();
void renderDebugEnd();

void renderEnableDepthTest();
void renderDisableDepthTest();

void renderDebugPoint(
	const PfxVector3 &position,
	const PfxVector3 &color);

void renderDebugLine(
	const PfxVector3 &position1,
	const PfxVector3 &position2,
	const PfxVector3 &color);

void renderDebugArc(
	const PfxVector3 &center,
	const PfxVector3 &normal,
	const float radius,
	const float start_rad,
	const float end_rad,
	const PfxVector3 &color);

void renderDebugArc( 
	const PfxVector3 &pos,
	const PfxVector3 &axis,
	const PfxVector3 &dir,
	const float radius,
	const float start_rad,
	const float end_rad,
	const PfxVector3 &color);

void renderDebugAabb(
	const PfxVector3 &center,
	const PfxVector3 &extent,
	const PfxVector3 &color);

void renderDebugBox(
	const PfxTransform3 &transform,
	const PfxVector3 &extent,
	const PfxVector3 &color);

///////////////////////////////////////////////////////////////////////////////
// 2D 

void renderDebug2dBegin();
void renderDebug2dEnd();

///////////////////////////////////////////////////////////////////////////////
// Render Parameter

void renderGetViewAngle(float &angleX,float &angleY,float &radius);
void renderSetViewAngle(float angleX,float angleY,float radius);
void renderResize(int width,int height);

void renderGetViewTarget(PfxVector3 &targetPos);
void renderSetViewTarget(const PfxVector3 &targetPos);
void renderGetViewRadius(float &radius);
void renderSetViewRadius(float radius);
void renderLookAtTarget(const PfxVector3 &viewPos,const PfxVector3 &viewTarget);

void renderGetScreenSize(int &width,int &height);

PfxVector3 renderGetWorldPosition(const PfxVector3 &screenPos);
PfxVector3 renderGetScreenPosition(const PfxVector3 &worldPos);

void renderWait();

#endif /* __RENDER_FUNC_H__ */
