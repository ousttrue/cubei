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

#define	SAMPLE_NAME "api_physics_effects/6_joint"

//#define ENABLE_DEBUG_DRAW

#ifdef ENABLE_DEBUG_DRAW
PfxDebugRender *s_debugRender = NULL;
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

#ifdef ENABLE_DEBUG_DRAW
	renderDebugBegin();
	s_debugRender->renderJoint(physicsGetJoints(), &physicsGetState(0), physicsGetNumJoints());
	renderDebugEnd();
#endif

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

	int w,h;
	renderGetScreenSize(w,h);
	ctrlSetScreenSize(w,h);

	static PfxVector3 pickPos(0.0f);

	if(ctrlButtonPressed(BTN_PICK) == BTN_STAT_DOWN) {
		int sx,sy;
		ctrlGetCursorPosition(sx,sy);
		PfxVector3 wp1((float)sx,(float)sy,0.0f);
		PfxVector3 wp2((float)sx,(float)sy,1.0f);
		wp1 = renderGetWorldPosition(wp1);
		wp2 = renderGetWorldPosition(wp2);
		pickPos = physicsPickStart(wp1,wp2);
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
