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

#define	SAMPLE_NAME "api_physics_effects/4_motion_type"

//#define ENABLE_DEBUG_DRAW

#ifdef ENABLE_DEBUG_DRAW
	#define ENABLE_DEBUG_DRAW_CONTACT
	#define ENABLE_DEBUG_DRAW_AABB
	#define ENABLE_DEBUG_DRAW_ISLAND
#endif

static bool s_isRunning = true;

int sceneId = 0;
bool simulating = false;

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
		for(int j=0;j<coll.getNumShapes();j++,++itrShape) {
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

				default:
				break;
			}
		}
	}

	renderDebugBegin();
	
	#ifdef ENABLE_DEBUG_DRAW_CONTACT
	for(int i=0;i<physicsGetNumContacts();i++) {
		const PfxContactManifold &contact = physicsGetContact(i);
		const PfxRigidState &stateA = physicsGetState(contact.getRigidBodyIdA());
		const PfxRigidState &stateB = physicsGetState(contact.getRigidBodyIdB());

		for(int j=0;j<contact.getNumContacts();j++) {
			const PfxContactPoint &cp = contact.getContactPoint(j);
			PfxVector3 pA = stateA.getPosition()+rotate(stateA.getOrientation(),pfxReadVector3(cp.m_localPointA));

			renderDebugPoint(pA,PfxVector3(0,0,1));
		}
	}
	#endif
	
	#ifdef ENABLE_DEBUG_DRAW_AABB
	for(int i=0;i<physicsGetNumRigidbodies();i++) {
		const PfxRigidState &state = physicsGetState(i);
		const PfxCollidable &coll = physicsGetCollidable(i);

		PfxVector3 center = state.getPosition() + coll.getCenter();
		PfxVector3 half = absPerElem(PfxMatrix3(state.getOrientation())) * coll.getHalf();
		
		renderDebugAabb(center,half,PfxVector3(1,0,0));
	}
	#endif

	#ifdef ENABLE_DEBUG_DRAW_ISLAND
	const PfxIsland *island = physicsGetIslands();
	if(island) {
		for(PfxUInt32 i=0;i<pfxGetNumIslands(island);i++) {
			PfxIslandUnit *islandUnit = pfxGetFirstUnitInIsland(island,i);
			PfxVector3 aabbMin(SCE_PFX_FLT_MAX);
			PfxVector3 aabbMax(-SCE_PFX_FLT_MAX);
			for(;islandUnit!=NULL;islandUnit=pfxGetNextUnitInIsland(islandUnit)) {
				const PfxRigidState &state = physicsGetState(pfxGetUnitId(islandUnit));
				const PfxCollidable &coll = physicsGetCollidable(pfxGetUnitId(islandUnit));
				PfxVector3 center = state.getPosition() + coll.getCenter();
				PfxVector3 half = absPerElem(PfxMatrix3(state.getOrientation())) * coll.getHalf();
				aabbMin = minPerElem(aabbMin,center-half);
				aabbMax = maxPerElem(aabbMax,center+half);
			}
			renderDebugAabb((aabbMax+aabbMin)*0.5f,(aabbMax-aabbMin)*0.5f,PfxVector3(0,1,0));
		}
	}
	#endif
	
	renderDebugEnd();

	renderEnd();
}

static int init(void)
{
	perfInit();
	ctrlInit();
	renderInit(SAMPLE_NAME);
	physicsInit();

	return 0;
}

static int shutdown(void)
{
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
