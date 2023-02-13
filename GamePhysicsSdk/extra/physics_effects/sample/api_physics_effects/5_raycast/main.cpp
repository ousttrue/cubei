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

#include "../common/common.h"
#include "../common/ctrl_func.h"
#include "../common/render_func.h"
#include "../common/perf_func.h"
#include "physics_func.h"
#include "barrel.h"
#include "landscape.h"

#define	SAMPLE_NAME "api_physics_effects/5_raycast"


//#define ENABLE_DEBUG_DRAW

#ifdef ENABLE_DEBUG_DRAW
PfxDebugRender *s_debugRender = NULL;
#endif

static bool s_isRunning = true;

int sceneId = 0;
bool simulating = false;

int landscapeMeshId;
int convexMeshId;

static void render(void)
{
	renderBegin();

	const PfxVector3 colorWhite(1.0f);
	const PfxVector3 colorGray(0.7f);

	for(int i=0;i<physicsGetNumRigidbodies();i++) {
		const PfxRigidState &state = physicsGetState(i);
		const PfxCollidable &coll = physicsGetCollidable(i);

		PfxVector3 color = state.isAsleep()?colorGray:colorWhite;

		PfxTransform3 rbT(state.getOrientation(), state.getPosition());

		PfxShapeIterator itrShape(coll);
		for(PfxUInt32 j=0;j<coll.getNumShapes();j++,++itrShape) {
			const PfxShape &shape = *itrShape;
			PfxTransform3 offsetT = shape.getOffsetTransform();
			PfxTransform3 worldT = rbT * offsetT;

			switch(shape.getType()) {
				case kPfxShapeSphere:
				renderSphere(
					worldT,
					color,
					PfxFloatInVec(shape.getSphere().m_radius));
				break;

				case kPfxShapeBox:
				renderBox(
					worldT,
					color,
					shape.getBox().m_half);
				break;

				case kPfxShapeCapsule:
				renderCapsule(
					worldT,
					color,
					PfxFloatInVec(shape.getCapsule().m_radius),
					PfxFloatInVec(shape.getCapsule().m_halfLen));
				break;

				case kPfxShapeCylinder:
				renderCylinder(
					worldT,
					color,
					PfxFloatInVec(shape.getCylinder().m_radius),
					PfxFloatInVec(shape.getCylinder().m_halfLen));
				break;

				case kPfxShapeConvexMesh:
				renderMesh(
					worldT,
					color,
					convexMeshId);
				break;

				case kPfxShapeLargeTriMesh:
				renderMesh(
					worldT,
					color,
					landscapeMeshId);
				break;

				default:
				break;
			}
		}
	}

	renderDebugBegin();
	
#ifdef ENABLE_DEBUG_DRAW
	extern PfxVector3 worldCenter, worldExtent;	
	s_debugRender->renderWorld(
		worldCenter,
		worldExtent);	
	s_debugRender->renderAabb(
		&physicsGetState(0),
		&physicsGetCollidable(0),
		physicsGetNumRigidbodies());
	s_debugRender->renderIsland(
		physicsGetIslands(),
		&physicsGetState(0),
		&physicsGetCollidable(0));
	s_debugRender->renderContact(
		physicsGetContactBuff(),
		physicsGetPairBuff(),
		&physicsGetState(0),
		physicsGetNumContacts());
	s_debugRender->renderLargeMesh(
		&physicsGetState(0),
		&physicsGetCollidable(0),
		physicsGetNumRigidbodies(),
		SCE_PFX_DRENDER_MESH_FLG_EDGE|SCE_PFX_DRENDER_MESH_FLG_NORMAL);
	s_debugRender->renderLargeMeshByFlag(
		&physicsGetState(0),
		&physicsGetCollidable(0),
		SCE_PFX_DRENDER_MESH_FLG_EDGE);
#endif

	for(int i=0;i<physicsGetNumRays();i++) {
		const PfxRayInput& rayInput = physicsGetRayInput(i);
		const PfxRayOutput& rayOutput = physicsGetRayOutput(i);
		if(rayOutput.m_contactFlag) {
			renderDebugLine(
				rayInput.m_startPosition,
				rayOutput.m_contactPoint,
				PfxVector3(1.0f,0.0f,1.0f));
			renderDebugLine(
				rayOutput.m_contactPoint,
				rayOutput.m_contactPoint+rayOutput.m_contactNormal,
				PfxVector3(1.0f,0.0f,1.0f));
		}
		else {
			renderDebugLine(rayInput.m_startPosition,
				rayInput.m_startPosition+rayInput.m_direction,
				PfxVector3(0.5f,0.0f,0.5f));
		}
	}

	extern bool doAreaRaycast;
	extern PfxVector3 areaCenter;
	extern PfxVector3 areaExtent;

	if(doAreaRaycast) {
		renderDebugAabb(areaCenter,areaExtent,PfxVector3(0,0,1));
	}

	renderDebugEnd();

	renderEnd();
}

static int init(void)
{
	perfInit();
	ctrlInit();
	renderInit(SAMPLE_NAME);
	physicsInit();

	landscapeMeshId = renderInitMesh(
		LargeMeshVtx,sizeof(float)*6,
		LargeMeshVtx+3,sizeof(float)*6,
		LargeMeshIdx,sizeof(unsigned short)*3,
		LargeMeshVtxCount,LargeMeshIdxCount/3);

	convexMeshId = renderInitMesh(
		BarrelVtx,sizeof(float)*6,
		BarrelVtx+3,sizeof(float)*6,
		BarrelIdx,sizeof(unsigned short)*3,
		BarrelVtxCount,BarrelIdxCount/3);

#ifdef ENABLE_DEBUG_DRAW
	s_debugRender = new PfxDebugRender;
	s_debugRender->setDebugRenderPointFunc(renderDebugPoint);
	s_debugRender->setDebugRenderLineFunc(renderDebugLine);
	s_debugRender->setDebugRenderArcFunc(renderDebugArc);
	s_debugRender->setDebugRenderAabbFunc(renderDebugAabb);
	s_debugRender->setDebugRenderBoxFunc(renderDebugBox);
	s_debugRender->resetVisible(physicsGetMaxNumRigidbodies());
#endif

	return 0;
}

static int shutdown(void)
{
#ifdef ENABLE_DEBUG_DRAW
	delete s_debugRender;
#endif

	ctrlRelease();
	renderRelease();
	physicsRelease();
	perfRelease();

	return 0;
}

static void update(void)
{
	float angX,angY,r;
	renderGetViewAngle(angX,angY,r);

	ctrlUpdate();
	
	if(ctrlButtonPressed(BTN_UP)) {
		angX -= 0.05f;
		if(angX < -1.4f) angX = -1.4f;
		if(angX > -0.01f) angX = -0.01f;
	}

	if(ctrlButtonPressed(BTN_DOWN)) {
		angX += 0.05f;
		if(angX < -1.4f) angX = -1.4f;
		if(angX > -0.01f) angX = -0.01f;
	}

	if(ctrlButtonPressed(BTN_LEFT)) {
		angY -= 0.05f;
	}

	if(ctrlButtonPressed(BTN_RIGHT)) {
		angY += 0.05f;
	}

	if(ctrlButtonPressed(BTN_ZOOM_OUT)) {
		r *= 1.1f;
		if(r > 500.0f) r = 500.0f;
	}

	if(ctrlButtonPressed(BTN_ZOOM_IN)) {
		r *= 0.9f;
		if(r < 1.0f) r = 1.0f;
	}

	if(ctrlButtonPressed(BTN_SCENE_RESET) == BTN_STAT_DOWN) {
		renderWait();
		physicsCreateScene(sceneId);
	}

	if(ctrlButtonPressed(BTN_SCENE_NEXT) == BTN_STAT_DOWN) {
		renderWait();
		physicsCreateScene(++sceneId);
	}

	if(ctrlButtonPressed(BTN_SIMULATION) == BTN_STAT_DOWN) {
		simulating = !simulating;
	}

	if(ctrlButtonPressed(BTN_STEP) == BTN_STAT_DOWN) {
		simulating = true;
	}
	else if(ctrlButtonPressed(BTN_STEP) == BTN_STAT_UP || ctrlButtonPressed(BTN_STEP) == BTN_STAT_KEEP) {
		simulating = false;
	}

	renderSetViewAngle(angX,angY,r);
}

#ifndef _WIN32

///////////////////////////////////////////////////////////////////////////////
// Main

int main(void)
{
	init();

	physicsCreateScene(sceneId);
	
	printf("## %s: INIT SUCCEEDED ##\n", SAMPLE_NAME);

	while (s_isRunning) {
		update();
		if(simulating) physicsSimulate();
		render();

		perfSync();
	}

	shutdown();

	printf("## %s: FINISHED ##\n", SAMPLE_NAME);

	return 0;
}

#else

///////////////////////////////////////////////////////////////////////////////
// WinMain

int WINAPI WinMain(HINSTANCE hInstance,HINSTANCE hPrevInstance,LPSTR lpCmdLine,int nCmdShow)
{
	init();
	
	physicsCreateScene(sceneId);
	
	SCE_PFX_PRINTF("## %s: INIT SUCCEEDED ##\n", SAMPLE_NAME);
	
	MSG msg;
	while(s_isRunning) {
		if(PeekMessage(&msg,NULL,0,0,PM_REMOVE)) {
			if(msg.message==WM_QUIT) {
				s_isRunning = false;
			}
			else {
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}
		}
		else {
			update();
			if(simulating) physicsSimulate();
			render();

			perfSync();
		}
	}

	shutdown();

	SCE_PFX_PRINTF("## %s: FINISHED ##\n", SAMPLE_NAME);

	return (msg.wParam);
}

#endif
