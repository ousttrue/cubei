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

#define	SAMPLE_NAME "api_physics_effects/1_simple"


static bool s_isRunning = true;

int sceneId = 0;
bool simulating = false;

int landscapeMeshId;
int convexMeshId;

static void render(void)
{
	renderBegin();
	
	for(int i=0;i<physicsGetNumRigidbodies();i++) {
		const PfxRigidState &state = physicsGetState(i);
		const PfxCollidable &coll = physicsGetCollidable(i);

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
					PfxVector3(1,1,1),
					PfxFloatInVec(shape.getSphere().m_radius));
				break;

				case kPfxShapeBox:
				renderBox(
					worldT,
					PfxVector3(1,1,1),
					shape.getBox().m_half);
				break;

				case kPfxShapeCapsule:
				renderCapsule(
					worldT,
					PfxVector3(1,1,1),
					PfxFloatInVec(shape.getCapsule().m_radius),
					PfxFloatInVec(shape.getCapsule().m_halfLen));
				break;

				case kPfxShapeCylinder:
				renderCylinder(
					worldT,
					PfxVector3(1,1,1),
					PfxFloatInVec(shape.getCylinder().m_radius),
					PfxFloatInVec(shape.getCylinder().m_halfLen));
				break;

				case kPfxShapeConvexMesh:
				renderMesh(
					worldT,
					PfxVector3(1,1,1),
					convexMeshId);
				break;

				case kPfxShapeLargeTriMesh:
				renderMesh(
					worldT,
					PfxVector3(1,1,1),
					landscapeMeshId);
				break;

				default:
				break;
			}
		}
	}

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
