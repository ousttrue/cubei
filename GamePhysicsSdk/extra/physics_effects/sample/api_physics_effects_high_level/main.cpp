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

#include "../api_physics_effects/common/common.h"
#include "../api_physics_effects/common/ctrl_func.h"
#include "../api_physics_effects/common/render_func.h"
#include "../api_physics_effects/common/perf_func.h"

#include "physics_func.h"
#include "town.h"
#include "high_level/pfx_high_level_include.h"

#define	SAMPLE_NAME "PhysicsEffects2.0"

//E Enable ENABLE_DEBUG_DRAW to draw debug shapes
#ifdef ENABLE_DEBUG_DRAW
	PfxDebugRender *g_debugRender = NULL;
#endif

static bool g_isRunning = true;

int sceneId = 0;
bool simulating = false;

PfxVector3 ray_p1,ray_p2;

static PfxVector3 pickPos(0.0f);

// Rigid body world
extern PfxRigidBodyWorld *world;

static void render(void)
{
	renderBegin();

	const PfxVector3 colorWhite(1.0f);
	const PfxVector3 colorGray(0.7f);
	
	for(int i=0;i<(int)world->getRigidBodyCount();i++) {
		if(world->isRemovedRigidBody(i)) continue;

		const PfxRigidState &state = world->getRigidState(i);
		const PfxCollidable &coll = world->getCollidable(i);

		PfxVector3 color = state.isAsleep()?colorGray:colorWhite;

		PfxTransform3 rbT(state.getOrientation(), state.getPosition());

		PfxShapeIterator itrShape(coll);
		for(int j=0;j<(int)coll.getNumShapes();j++,++itrShape) {
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
				case kPfxShapeLargeTriMesh:
				{
					int meshId = -1;
					if(shape.getType() == kPfxShapeConvexMesh) {
						meshId = physicsGetRenderMeshId((void*)shape.getConvexMesh());
					}
					else {
						meshId = physicsGetRenderMeshId((void*)shape.getLargeTriMesh());
					}
					
					if(meshId >= 0) {
						renderMesh(worldT,color,meshId);
					}
				}
				break;

				default:
				break;
			}
		}
	}

	renderDebugBegin();

	if(physicsGetNumRays() > 0) {
		// Scene5
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

		extern PfxVector3 areaCenter;
		extern PfxVector3 areaExtent;
		renderDebugAabb(areaCenter,areaExtent,PfxVector3(0.0f,0.0f,1.0f));
	}

#ifdef ENABLE_DEBUG_DRAW
	//g_debugRender->renderWorld(
	//	world->getWorldCenter(),
	//	world->getWorldExtent());
	//g_debugRender->renderAabb(
	//	world->getRigidStatePtr(),
	//	world->getCollidablePtr(),
	//	world->getRigidBodyCount());
	//g_debugRender->renderIsland(
	//	world->getIsland(),
	//	world->getRigidStatePtr(),
	//	world->getCollidablePtr());
	g_debugRender->renderContact(
		world->getContactManifoldPtr(),
		world->getContactPairPtr(),
		world->getRigidStatePtr(),
		world->getContactCount());
	//g_debugRender->renderJoint(
	//	world->getJointPtr(),
	//	world->getRigidStatePtr(),
	//	world->getJointCount());
	//g_debugRender->renderLargeMesh(
	//	world->getRigidStatePtr(),
	//	world->getCollidablePtr(),
	//	world->getRigidBodyCount(),
	//	SCE_PFX_DRENDER_MESH_FLG_EDGE | SCE_PFX_DRENDER_MESH_FLG_NORMAL);
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

	float angX,angY,r;
	renderGetViewAngle(angX,angY,r);
	r *= 0.5f;
	renderSetViewAngle(angX,angY,r);

#ifdef ENABLE_DEBUG_DRAW
	g_debugRender = new PfxDebugRender;
	g_debugRender->setDebugRenderPointFunc(renderDebugPoint);
	g_debugRender->setDebugRenderLineFunc(renderDebugLine);
	g_debugRender->setDebugRenderArcFunc(renderDebugArc);
	g_debugRender->setDebugRenderAabbFunc(renderDebugAabb);
	g_debugRender->setDebugRenderBoxFunc(renderDebugBox);
	g_debugRender->resetVisible(world->getRigidBodyCapacity());
#endif

	return 0;
}

static int shutdown(void)
{
#ifdef ENABLE_DEBUG_DRAW
	delete g_debugRender;
#endif

	ctrlRelease();
	renderRelease();
	physicsRelease();
	perfRelease();

	return 0;
}

void releaseAllMeshes()
{
	//	meshes to display
	renderReleaseMeshAll();
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
		renderReleaseMeshAll();
		physicsCreateScene(sceneId);
	}

	if(ctrlButtonPressed(BTN_SCENE_NEXT) == BTN_STAT_DOWN) {
		renderWait();
		renderReleaseMeshAll();
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

	int w,h;
	renderGetScreenSize(w,h);
	ctrlSetScreenSize(w,h);
	
	if(ctrlButtonPressed(BTN_PICK) == BTN_STAT_DOWN) {
		int sx,sy;
		ctrlGetCursorPosition(sx,sy);
		PfxVector3 wp1((float)sx,(float)sy,0.0f);
		PfxVector3 wp2((float)sx,(float)sy,1.0f);
		wp1 = renderGetWorldPosition(wp1);
		wp2 = renderGetWorldPosition(wp2);
		pickPos = physicsPickStart(wp1,wp2);

		ray_p1 = wp1;
		ray_p2 = wp2;
	}
	else if(ctrlButtonPressed(BTN_PICK) == BTN_STAT_KEEP) {
		int sx,sy;
		ctrlGetCursorPosition(sx,sy);
		PfxVector3 sp = renderGetScreenPosition(pickPos);
		PfxVector3 wp((float)sx,(float)sy,sp[2]);
		wp = renderGetWorldPosition(wp);
		physicsPickUpdate(wp);
	}
	else if(ctrlButtonPressed(BTN_PICK) == BTN_STAT_UP) {
		physicsPickEnd();
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

	while (g_isRunning) {
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
#ifdef _WIN64
	SCE_PFX_ASSERT( SCE_PFX_IS_RUNNING_ON_64BIT_ENV()  );
#endif

	init();
	
	physicsCreateScene(sceneId);
	
	SCE_PFX_PRINTF("## %s: INIT SUCCEEDED ##\n", SAMPLE_NAME);
	
	MSG msg;
	while(g_isRunning) {
		if(PeekMessage(&msg,NULL,0,0,PM_REMOVE)) {
			if(msg.message==WM_QUIT) {
				g_isRunning = false;
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
