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

	2. Altered source versions must be plainly marked as such, and must not be
	misrepresented as being the original software.

	3. This notice may not be removed or altered from any source distribution.
*/

#include <stdlib.h>
#include "../common/common.h"
#include "../common/geometry_data.h"
#include "physics_func.h"

using namespace EasyPhysics;

///////////////////////////////////////////////////////////////////////////////
// シミュレーション定義

const int maxRigidBodies	= 500;
const int maxJoints			= 100;
const int maxPairs			= 5000;
const float timeStep		= 0.016f;
const float contactBias		= 0.1f;
const float contactSlop		= 0.001f;
const int iteration			= 10;
const EpxVector3 gravity(0.0f,-9.8f,0.0f);

///////////////////////////////////////////////////////////////////////////////
// シミュレーションデータ

// 剛体
EpxState states[maxRigidBodies];
EpxRigidBody bodies[maxRigidBodies];
EpxCollidable collidables[maxRigidBodies];
EpxUInt32 numRigidBodies = 0;

// ジョイント
EpxBallJoint joints[maxJoints];
EpxUInt32 numJoints = 0;

// ペア
unsigned int pairSwap;
EpxUInt32 numPairs[2];
EpxPair pairs[2][maxPairs];

static int frame = 0;

///////////////////////////////////////////////////////////////////////////////
// メモリアロケータ

class DefaultAllocator : public EpxAllocator
{
public:
	void *allocate(size_t bytes)
	{
		return malloc(bytes);
	}
	
	void deallocate(void *p)
	{
		free(p);
	}
} allocator;

///////////////////////////////////////////////////////////////////////////////
// ブロードフェーズコールバック

EpxBool broadPhaseCallback(EpxUInt32 rigidBodyIdA,EpxUInt32 rigidBodyIdB,void *userData)
{
	// ジョイントで接続された剛体のペアは作成しない
	for(EpxUInt32 i=0;i<numJoints;i++) {
		if((joints[i].rigidBodyA == rigidBodyIdA && joints[i].rigidBodyB == rigidBodyIdB) || 
		   (joints[i].rigidBodyA == rigidBodyIdB && joints[i].rigidBodyB == rigidBodyIdA)) {
			return false;
		}
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////
// シミュレーション関数

struct Perf {
	unsigned long long count;
	int frame;

	void setFrame(int f)
	{
		frame = f;
	}

	void begin()
	{
		count = perfGetCount();
	}

	void end(const char *msg)
	{
		unsigned long long count2 = perfGetCount();
		float msec = perfGetTimeMillisecond(count,count2);
		if(frame % 100 == 0) {
			epxPrintf("%s : %.2f msec\n",msg,msec);
		}
	}
};

void physicsSimulate()
{
	Perf perf;
	perf.setFrame(frame);

	pairSwap = 1 - pairSwap;
	
	perf.begin();
	for(EpxUInt32 i=0;i<numRigidBodies;i++) {
		EpxVector3 externalForce = gravity * bodies[i].m_mass;
		EpxVector3 externalTorque(0.0f);
		epxApplyExternalForce(states[i],bodies[i],externalForce,externalTorque,timeStep);
	}
	perf.end("apply force");
	
	perf.begin();
	epxBroadPhase(
		states,collidables,numRigidBodies,
		pairs[1-pairSwap],numPairs[1-pairSwap],
		pairs[pairSwap],numPairs[pairSwap],
		maxPairs,&allocator,NULL,broadPhaseCallback);
	perf.end("broadphase");
	
	perf.begin();
	epxDetectCollision(
		states,collidables,numRigidBodies,
		pairs[pairSwap],numPairs[pairSwap]);
	perf.end("collision");
	
	perf.begin();
	epxSolveConstraints(
		states,bodies,numRigidBodies,
		pairs[pairSwap],numPairs[pairSwap],
		joints,numJoints,
		iteration,contactBias,contactSlop,timeStep,&allocator);
	perf.end("solver");
	
	perf.begin();
	epxIntegrate(states,numRigidBodies,timeStep);
	perf.end("integrate");

	frame++;
}

///////////////////////////////////////////////////////////////////////////////
// シーンの作成

static int fireRigidBodyId;

void createFireBody()
{
	fireRigidBodyId = numRigidBodies++;

	EpxVector3 scale(0.5f);		

	states[fireRigidBodyId].reset();
	states[fireRigidBodyId].m_motionType = EpxMotionTypeStatic;
	states[fireRigidBodyId].m_position = EpxVector3(999.0f);
	bodies[fireRigidBodyId].reset();
	bodies[fireRigidBodyId].m_mass = 1.0f;
	bodies[fireRigidBodyId].m_inertia = epxCalcInertiaBox(scale,1.0f);
	collidables[fireRigidBodyId].reset();

	EpxShape shape;
	shape.reset();

	epxCreateConvexMesh(&shape.m_geometry,sphere_vertices,sphere_numVertices,sphere_indices,sphere_numIndices,scale);
	shape.userData = (void*)createRenderMesh(&shape.m_geometry);

	collidables[fireRigidBodyId].addShape(shape);
	collidables[fireRigidBodyId].finish();
}

void createSceneBallJoint()
{
	// 地面
	{
		int id = numRigidBodies++;
		
		EpxVector3 scale(10.0f,1.0f,10.0f);
		
		states[id].reset();
		states[id].m_motionType = EpxMotionTypeStatic;
		states[id].m_position = EpxVector3(0.0f,-scale[1],0.0f);
		bodies[id].reset();
		collidables[id].reset();
		
		EpxShape shape;
		shape.reset();
		
		epxCreateConvexMesh(&shape.m_geometry,box_vertices,box_numVertices,box_indices,box_numIndices,scale);
		
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
		
		collidables[id].addShape(shape);
		collidables[id].finish();
	}

	{
		int id = numRigidBodies++;

		EpxVector3 scale(0.25f,0.25f,2.0f);

		states[id].reset();
		states[id].m_position = EpxVector3(0.0f,scale[1],0.0f);
		bodies[id].reset();
		bodies[id].m_mass = 1.0f;
		bodies[id].m_inertia = epxCalcInertiaBox(scale,1.0f);
		collidables[id].reset();
	
		EpxShape shape;
		shape.reset();
	
		epxCreateConvexMesh(&shape.m_geometry,box_vertices,box_numVertices,box_indices,box_numIndices,scale);
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
	
		collidables[id].addShape(shape);
		collidables[id].finish();
	}

	// ボールジョイントにより接続されたボックス
	EpxBallJoint &joint = joints[numJoints++];
	joint.reset();

	{
		int id = numRigidBodies++;

		EpxVector3 scale(1.0f,0.25f,0.25f);

		states[id].reset();
		states[id].m_position = EpxVector3(-scale[0],3.0f,0.0f);
		bodies[id].reset();
		bodies[id].m_mass = 1.0f;
		bodies[id].m_inertia = epxCalcInertiaBox(scale,1.0f);
		collidables[id].reset();
	
		EpxShape shape;
		shape.reset();
	
		epxCreateConvexMesh(&shape.m_geometry,cylinder_vertices,cylinder_numVertices,cylinder_indices,cylinder_numIndices,scale);
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
	
		collidables[id].addShape(shape);
		collidables[id].finish();
		
		joint.rigidBodyA = id;
		joint.anchorA = EpxVector3(scale[0],0.0f,0.0f);
	}

	{
		int id = numRigidBodies++;

		EpxVector3 scale(1.0f,0.25f,0.25f);

		states[id].reset();
		states[id].m_position = EpxVector3(scale[0],3.0f,0.0f);
		bodies[id].reset();
		bodies[id].m_mass = 1.0f;
		bodies[id].m_inertia = epxCalcInertiaBox(scale,1.0f);
		collidables[id].reset();
	
		EpxShape shape;
		shape.reset();
	
		epxCreateConvexMesh(&shape.m_geometry,cylinder_vertices,cylinder_numVertices,cylinder_indices,cylinder_numIndices,scale);
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
	
		collidables[id].addShape(shape);
		collidables[id].finish();

		joint.rigidBodyB = id;
		joint.anchorB = EpxVector3(-scale[0],0.0f,0.0f);
	}
}

void createSceneHingeJoint()
{
	// ボールジョイントx2でヒンジジョイントを再現
	EpxBallJoint &joint0 = joints[numJoints++];
	EpxBallJoint &joint1 = joints[numJoints++];
	joint0.reset();
	joint1.reset();

	{
		int id = numRigidBodies++;

		EpxVector3 scale(1.25f,0.25f,0.25f);

		states[id].reset();
		states[id].m_motionType = EpxMotionTypeStatic;
		states[id].m_position = EpxVector3(0.0f,1.0f,0.0f);
		bodies[id].reset();
		collidables[id].reset();
	
		EpxShape shape;
		shape.reset();
		shape.m_offsetOrientation = EpxQuat::rotationZ(0.5f*EPX_PI);
		epxCreateConvexMesh(&shape.m_geometry,cylinder_vertices,cylinder_numVertices,cylinder_indices,cylinder_numIndices,scale);
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
	
		collidables[id].addShape(shape);
		collidables[id].finish();
		
		joint0.rigidBodyA = id;
		joint0.anchorA = EpxVector3(0.0f,1.0f,0.0f);
		joint1.rigidBodyA = id;
		joint1.anchorA = EpxVector3(0.0f,-1.0f,0.0f);
	}

	{
		int id = numRigidBodies++;

		EpxVector3 scale(2.0f,1.0f,0.25f);

		states[id].reset();
		states[id].m_position = EpxVector3(0.0f,1.0f,0.0f);
		states[id].m_angularVelocity = EpxVector3(0.0f,10.0f,0.0f);
		bodies[id].reset();
		bodies[id].m_mass = 10.0f;
		// 回転方向に安定するように慣性テンソル計算用の形状を調整
		bodies[id].m_inertia = epxCalcInertiaBox(EpxVector3(0.5f,3.0f,0.5f),10.0f);
		collidables[id].reset();
	
		EpxShape shape;
		shape.reset();
	
		epxCreateConvexMesh(&shape.m_geometry,box_vertices,box_numVertices,box_indices,box_numIndices,scale);
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
	
		collidables[id].addShape(shape);
		collidables[id].finish();
		
		joint0.rigidBodyB = id;
		joint0.anchorB = EpxVector3(0.0f,1.0f,0.0f);
		joint1.rigidBodyB = id;
		joint1.anchorB = EpxVector3(0.0f,-1.0f,0.0f);
	}
}

void createSceneFixedJoint()
{
	// ボールジョイントx3で固定ジョイントを再現
	EpxBallJoint &joint0 = joints[numJoints++];
	EpxBallJoint &joint1 = joints[numJoints++];
	EpxBallJoint &joint2 = joints[numJoints++];
	joint0.reset();
	joint1.reset();
	joint2.reset();

	{
		int id = numRigidBodies++;

		EpxVector3 scale(1.0f,2.5f,2.5f);

		states[id].reset();
		states[id].m_motionType = EpxMotionTypeStatic;
		states[id].m_position = EpxVector3(-1.0f,1.0f,0.0f);
		bodies[id].reset();
		collidables[id].reset();
	
		EpxShape shape;
		shape.reset();
		epxCreateConvexMesh(&shape.m_geometry,box_vertices,box_numVertices,box_indices,box_numIndices,scale);
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
	
		collidables[id].addShape(shape);
		collidables[id].finish();
		
		joint0.rigidBodyA = id;
		joint1.rigidBodyA = id;
		joint2.rigidBodyA = id;
		joint0.anchorA = EpxVector3(2.0f,5.0f,0.0f);
		joint1.anchorA = EpxVector3(2.0f,-5.0f,1.0f);
		joint2.anchorA = EpxVector3(2.0f,-5.0f,-1.0f);
	}

	{
		int id = numRigidBodies++;

		EpxVector3 scale(1.0f,0.25f,0.25f);

		states[id].reset();
		states[id].m_position = EpxVector3(1.0f,1.0f,0.0f);
		bodies[id].reset();
		bodies[id].m_mass = 10.0f;
		// 安定するように慣性テンソル計算用の形状を大きめに設定
		bodies[id].m_inertia = epxCalcInertiaBox(10.0f*scale,10.0f);
		collidables[id].reset();
	
		EpxShape shape;
		shape.reset();
	
		epxCreateConvexMesh(&shape.m_geometry,box_vertices,box_numVertices,box_indices,box_numIndices,scale);
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
	
		collidables[id].addShape(shape);
		collidables[id].finish();
		
		joint0.rigidBodyB = id;
		joint1.rigidBodyB = id;
		joint2.rigidBodyB = id;
		// 固定ジョイントが安定するように、接続位置を剛体の重心に配置
		joint0.anchorB = EpxVector3(0.0f,5.0f,0.0f);
		joint1.anchorB = EpxVector3(0.0f,-5.0f,1.0f);
		joint2.anchorB = EpxVector3(0.0f,-5.0f,-1.0f);
	}
}

int createGear(const EpxVector3 &offsetPosition,const EpxQuat &offsetOrientation)
{
	int gearId;

	EpxFloat cogsWidth = 0.25f;

	// ボールジョイントx2でヒンジジョイントを再現
	EpxBallJoint &joint0 = joints[numJoints++];
	EpxBallJoint &joint1 = joints[numJoints++];
	joint0.reset();
	joint1.reset();

	{
		int id = numRigidBodies++;

		EpxVector3 scale(0.3f,0.5f,0.5f);

		states[id].reset();
		states[id].m_motionType = EpxMotionTypeStatic;
		states[id].m_position = offsetPosition + EpxVector3(0.0f,1.0f,0.0f);
		states[id].m_orientation = offsetOrientation;
		bodies[id].reset();
		collidables[id].reset();
	
		EpxShape shape;
		shape.reset();
		shape.m_offsetOrientation = EpxQuat::rotationY(0.5f*EPX_PI);
		epxCreateConvexMesh(&shape.m_geometry,cylinder_vertices,cylinder_numVertices,cylinder_indices,cylinder_numIndices,scale);
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
	
		collidables[id].addShape(shape);
		collidables[id].finish();
		
		joint0.rigidBodyA = id;
		joint0.anchorA = EpxVector3(0.0f,0.0f,5.0f);
		joint1.rigidBodyA = id;
		joint1.anchorA = EpxVector3(0.0f,0.0f,-5.0f);
	}

	{
		int id = numRigidBodies++;

		EpxVector3 scale(0.25f,2.0f,2.0f);

		states[id].reset();
		states[id].m_position = offsetPosition + EpxVector3(0.0f,1.0f,0.0f);
		states[id].m_orientation = offsetOrientation;
		bodies[id].reset();
		bodies[id].m_mass = 10.0f;
		// 回転方向に安定するように慣性テンソル計算用の形状を調整
		bodies[id].m_inertia = epxCalcInertiaBox(EpxVector3(2.5f,2.5f,25.0f),10.0f);
		collidables[id].reset();
	
		{
			EpxShape shape;
			shape.reset();
			shape.m_offsetOrientation = EpxQuat::rotationY(0.5f*EPX_PI);
			epxCreateConvexMesh(&shape.m_geometry,cylinder_vertices,cylinder_numVertices,cylinder_indices,cylinder_numIndices,scale);
			shape.userData = (void*)createRenderMesh(&shape.m_geometry);
			collidables[id].addShape(shape);
		}

		for(int i=0;i<4;i++) {
			const EpxVector3 cogsScale(2.5f,cogsWidth,0.25f);
			EpxShape shape;
			shape.reset();
			shape.m_offsetOrientation = EpxQuat::rotationZ(i*0.25f*EPX_PI);
			epxCreateConvexMesh(&shape.m_geometry,box_vertices,box_numVertices,box_indices,box_numIndices,cogsScale);
			shape.userData = (void*)createRenderMesh(&shape.m_geometry);
			collidables[id].addShape(shape);
		}

		collidables[id].finish();

		// ギアジョイントが安定するように、接続位置を広めに配置
		joint0.rigidBodyB = id;
		joint0.anchorB = EpxVector3(0.0f,0.0f,5.0f);
		joint1.rigidBodyB = id;
		joint1.anchorB = EpxVector3(0.0f,0.0f,-5.0f);

		gearId = id;
	}

	return gearId;
}

void createSceneGearJoint()
{
	// ギア大
	createGear(EpxVector3(2.0f,1.0f,0.0f),EpxQuat::identity());

	// ギア小
	int gearId = createGear(EpxVector3(-2.5f,1.0f,0.0f),EpxQuat::rotationZ(0.125f*EPX_PI));
	states[gearId].m_angularVelocity = EpxVector3(0.0f,0.0f,-10.0f);
}

void createSceneChain()
{
	const EpxVector3 chainScale(0.125f,0.5f,0.125f);
	const EpxVector3 ballScale(1.5f);

	// ダミー
	int dummyId;
	{
		dummyId = numRigidBodies++;
		
		EpxVector3 scale(0.01f);
		
		states[dummyId].reset();
		states[dummyId].m_motionType = EpxMotionTypeStatic;
		states[dummyId].m_position = EpxVector3(0.0f,0.0f,0.0f);
		bodies[dummyId].reset();
		collidables[dummyId].reset();
		
		EpxShape shape;
		shape.reset();
		
		epxCreateConvexMesh(&shape.m_geometry,box_vertices,box_numVertices,box_indices,box_numIndices,scale);
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
		
		collidables[dummyId].addShape(shape);
		collidables[dummyId].finish();
	}

	// 1kgの鎖の先に100kgの錘を接続
	
	// 慣性テンソル調整なし
	for(int i=0;i<5;i++) {
		int id = numRigidBodies++;

		states[id].reset();
		states[id].m_position = EpxVector3(-2.5f,5.0f,0.0f) + EpxVector3(0.0f,-i*chainScale[1]*2.0f,0.0f);
		bodies[id].reset();
		bodies[id].m_mass = 1.0f;
		bodies[id].m_inertia = epxCalcInertiaBox(chainScale,1.0f);
		collidables[id].reset();
	
		EpxShape shape;
		shape.reset();
	
		epxCreateConvexMesh(&shape.m_geometry,box_vertices,box_numVertices,box_indices,box_numIndices,chainScale);
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
	
		collidables[id].addShape(shape);
		collidables[id].finish();
		
		EpxBallJoint &joint = joints[numJoints++];
		joint.reset();
		if(i>0) {
			joint.rigidBodyA = id-1;
			joint.anchorA = EpxVector3(0.0f,-chainScale[1],0.0f);
		}
		else {
			joint.rigidBodyA = dummyId;
			joint.anchorA = states[id].m_position + EpxVector3(0.0f,chainScale[1],0.0f);
		}
		joint.rigidBodyB = id;
		joint.anchorB = EpxVector3(0.0f,chainScale[1],0.0f);
	}

	{
		int id = numRigidBodies++;
		
		states[id].reset();
		states[id].m_position = states[id-1].m_position + EpxVector3(0.0f,-chainScale[1]-ballScale[1],0.0f);
		bodies[id].reset();
		bodies[id].m_mass = 10.0f;
		bodies[id].m_inertia = epxCalcInertiaSphere(ballScale[1],10.0f);
		collidables[id].reset();
		
		EpxShape shape;
		shape.reset();
		
		epxCreateConvexMesh(&shape.m_geometry,sphere_vertices,sphere_numVertices,sphere_indices,sphere_numIndices,ballScale);
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
		
		collidables[id].addShape(shape);
		collidables[id].finish();

		EpxBallJoint &joint = joints[numJoints++];
		joint.reset();
		joint.rigidBodyA = id-1;
		joint.rigidBodyB = id;
		joint.anchorA = EpxVector3(0.0f,-chainScale[1],0.0f);
		joint.anchorB = EpxVector3(0.0f,ballScale[1],0.0f);
	}

	// 慣性テンソル調整あり
	for(int i=0;i<5;i++) {
		int id = numRigidBodies++;

		states[id].reset();
		states[id].m_position = EpxVector3(2.5f,5.0f,0.0f) + EpxVector3(0.0f,-i*chainScale[1]*2.0f,0.0f);
		bodies[id].reset();
		bodies[id].m_mass = 1.0f;
		bodies[id].m_inertia = epxCalcInertiaBox(15.0f * chainScale,1.0f); // 慣性テンソル増加
		collidables[id].reset();
	
		EpxShape shape;
		shape.reset();
	
		epxCreateConvexMesh(&shape.m_geometry,box_vertices,box_numVertices,box_indices,box_numIndices,chainScale);
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
	
		collidables[id].addShape(shape);
		collidables[id].finish();
		
		EpxBallJoint &joint = joints[numJoints++];
		joint.reset();
		if(i>0) {
			joint.rigidBodyA = id-1;
			joint.anchorA = EpxVector3(0.0f,-chainScale[1],0.0f);
		}
		else {
			joint.rigidBodyA = dummyId;
			joint.anchorA = states[id].m_position + EpxVector3(0.0f,chainScale[1],0.0f);
		}
		joint.rigidBodyB = id;
		joint.anchorB = EpxVector3(0.0f,chainScale[1],0.0f);
	}

	{
		int id = numRigidBodies++;
		
		states[id].reset();
		states[id].m_position = states[id-1].m_position + EpxVector3(0.0f,-chainScale[1]-ballScale[1],0.0f);
		bodies[id].reset();
		bodies[id].m_mass = 10.0f;
		bodies[id].m_inertia = epxCalcInertiaSphere(ballScale[1],10.0f);
		collidables[id].reset();
		
		EpxShape shape;
		shape.reset();
		
		epxCreateConvexMesh(&shape.m_geometry,sphere_vertices,sphere_numVertices,sphere_indices,sphere_numIndices,ballScale);
		shape.userData = (void*)createRenderMesh(&shape.m_geometry);
		
		collidables[id].addShape(shape);
		collidables[id].finish();

		EpxBallJoint &joint = joints[numJoints++];
		joint.reset();
		joint.rigidBodyA = id-1;
		joint.rigidBodyB = id;
		joint.anchorA = EpxVector3(0.0f,-chainScale[1],0.0f);
		joint.anchorB = EpxVector3(0.0f,ballScale[1],0.0f);
	}
}

static const int maxScenes = 5;
static const char titles[][32] = {
	"ball joint",
	"hinge joint",
	"fixed joint",
	"gear joint",
	"chain joint",
};

const char *physicsGetSceneTitle(int i)
{
	return titles[i%maxScenes];
}

void physicsCreateScene(int sceneId)
{
	frame = 0;
	
	numRigidBodies = 0;
	numJoints = 0;
	numPairs[0] = numPairs[1] = 0;
	pairSwap = 0;
	
	switch(sceneId%maxScenes) {
		case 0:
		createSceneBallJoint();
		break;
		
		case 1:
		createSceneHingeJoint();
		break;

		case 2:
		createSceneFixedJoint();
		break;

		case 3:
		createSceneGearJoint();
		break;

		case 4:
		createSceneChain();
		break;
	}

	createFireBody();
}

///////////////////////////////////////////////////////////////////////////////
// 初期化、解放

bool physicsInit()
{
	return true;
}

void physicsRelease()
{
}

///////////////////////////////////////////////////////////////////////////////
// 外部から物理データへのアクセス

int physicsGetNumRigidbodies()
{
	return numRigidBodies;
}

const EpxState &physicsGetState(int i)
{
	return states[i];
}

const EpxRigidBody &physicsGetRigidBody(int i)
{
	return bodies[i];
}

const EpxCollidable &physicsGetCollidable(int i)
{
	return collidables[i];
}

int physicsGetNumContacts()
{
	return numPairs[pairSwap];
}

const EasyPhysics::EpxContact &physicsGetContact(int i)
{
	return *pairs[pairSwap][i].contact;
}

EpxUInt32 physicsGetRigidBodyAInContact(int i)
{
	return pairs[pairSwap][i].rigidBodyA;
}

EpxUInt32 physicsGetRigidBodyBInContact(int i)
{
	return pairs[pairSwap][i].rigidBodyB;
}

void physicsFire(const EasyPhysics::EpxVector3 &position,const EasyPhysics::EpxVector3 &velocity)
{
	epxPrintf("fire pos %f %f %f vel %f %f %f\n",
		position[0],position[1],position[2],velocity[0],velocity[1],velocity[2]);

	states[fireRigidBodyId].m_motionType = EpxMotionTypeActive;
	states[fireRigidBodyId].m_position = position;
	states[fireRigidBodyId].m_linearVelocity = velocity;
	states[fireRigidBodyId].m_angularVelocity = EpxVector3(0.0f);
}
