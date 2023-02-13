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

#ifndef _SCE_PFX_RIGIDBODY_WORLD_H
#define _SCE_PFX_RIGIDBODY_WORLD_H

#include "physics_effects.h"

namespace sce{
namespace PhysicsEffects{

///////////////////////////////////////////////////////////////////////////////
// Simulation flag

#define SCE_PFX_ENABLE_SLEEP					0x01

///////////////////////////////////////////////////////////////////////////////
// PfxRigidBodyWorldParam

struct PfxRigidBodyWorldParam {
	// Pool memory
	void *poolBuff;
	PfxUInt32 poolBytes;

	// Number of objects
	PfxUInt32 maxRigidBodies;
	PfxUInt32 maxJoints;
	PfxUInt32 maxShapes;
	PfxUInt32 maxContacts;
	PfxUInt32 maxNonContactPairs;
	SCE_PFX_PADDING(1,4)

	// World config
	PfxVector3 worldCenter;
	PfxVector3 worldExtent;
	PfxVector3 gravity;

	// Simulation config
	PfxFloat timeStep;
	PfxFloat separateBias;
	PfxUInt32 iteration;
	PfxUInt32 sleepCount;
	PfxFloat sleepVelocity;

	PfxUInt32 simulationFlag;

	SCE_PFX_PADDING(2,2)

	PfxRigidBodyWorldParam()
	{
		maxRigidBodies = 500;
		maxJoints = 500;
		maxShapes = 100;
		maxContacts = 4000;
		maxNonContactPairs = 100;

		worldCenter = PfxVector3(0.0f);
		worldExtent = PfxVector3(500.0f);
		gravity = PfxVector3(0.0f,-9.8f,0.0f);

		timeStep = 0.0166666666f;
		separateBias = 0.2f;
		iteration = 5;

		sleepCount = 180;
		sleepVelocity = 0.1f;

		poolBuff = NULL;
		poolBytes = 0;

		simulationFlag = 0;
	}
};

///////////////////////////////////////////////////////////////////////////////
// PfxAabbInput

//J AABBオーバーラップ用 PfxAabbInput構造体
//E AABB input structure used by findAabbOverlap()
struct PfxAabbInput {
	PfxVector3 center;
	PfxVector3 extent;
	PfxUInt32 contactFilterSelf;
	PfxUInt32 contactFilterTarget;
	SCE_PFX_PADDING(1,8)

	void reset()
	{
		contactFilterSelf = contactFilterTarget = 0xffffffff;
	}
};

///////////////////////////////////////////////////////////////////////////////
// PfxRigidBodyWorld class

class PfxRigidBodyWorld
{
protected:
	//J プールメモリ
	//E Pool memory
	PfxHeapManager m_pool;

	//J ワールド
	//E World
	PfxVector3 m_worldCenter;
	PfxVector3 m_worldExtent;
	PfxVector3 m_gravity;

	//J 剛体
	//E Rigid body
	PfxPoolArray<PfxRigidState> m_states;
	PfxPoolArray<PfxRigidBody>  m_bodies;
	PfxPoolArray<PfxCollidable> m_collidables;
	PfxSolverBody *m_solverBodies;

	SCE_PFX_PADDING(2,12)

	//J 形状
	//E Shape
	PfxPoolArray<PfxShape> m_shapes;

	//J ジョイント
	//E Joint
	PfxPoolArray<PfxConstraintPair> m_jointPairs;
	PfxPoolArray<PfxJoint> m_joints;

	//J プロキシ
	//E Proxies
	PfxBroadphaseProxy *m_proxies[6];

	SCE_PFX_PADDING(3,8)

	//J コンタクト
	//E Contacts
	PfxPoolArray<PfxContactManifold> m_contacts;

	//J アイランド
	//E Island
	PfxIsland *m_island;
	PfxUInt32 m_islandBytes;
	void *m_islandBuff;

	//J ノンコンタクトペア
	//E Non contact pairs
	PfxUInt32 m_numNonContactPairs;
	PfxUInt32 m_maxNonContactPairs;
	PfxUInt32 *m_nonContactPairs;

	//J キャッシュされるペアバッファ
	//E Cached pair buffers
	PfxUInt32 m_numPairs[2];
	PfxBroadphasePair *m_pairsBuff[2];

	PfxFloat m_timeStep;
	PfxFloat m_separateBias;
	PfxUInt32 m_iteration;
	PfxUInt32 m_sleepCount;
	PfxFloat m_sleepVelocity;
	PfxUInt32 m_simulationFlag;

	//J 内部使用パラメータ
	//E Parameters for internal use
	PfxUInt32 m_pairSwap;
	PfxUInt32 m_frame;
	PfxUInt32 m_numProxiesInWorld;

	SCE_PFX_PADDING(4,4)

	PfxVector3 m_areaCenter;
	PfxVector3 m_areaExtent;
	
	void printPairs(const char *msg,PfxBroadphasePair *pairs,int n);
	
public:
	void broadphase();

	void collision();

	void constraintSolver();

	void sleepOrWakeup();

	void integrate();

public:
	//J poolBuffにセットすべきバッファのサイズを取得する
	//E Get the size of bytes which should be given to poolBuff
	static PfxUInt32 getRigidBodyWorldBytes(const PfxRigidBodyWorldParam &param);

	// Constractor / Destructor
	PfxRigidBodyWorld(const PfxRigidBodyWorldParam &param);
	~PfxRigidBodyWorld();

	// Initialize / Finalize
	void initialize();
	void finalize();

	// Reset
	void reset();
	
	// Simulation
	void simulate();
	
	// Raycast / AABB overlap
	
	//J	レイキャスト及び、AABBオーバラップはsetCastArea()でキャストする範囲を指定してください。
	//J	※指定が無い場合は、デフォルトのワールドサイズが使用されます。
	//J	※setCastArea()は内部のプロキシ配列を書き換えます。
	//E Call setCastArea() to specify an area before calling castRays() or findAabbOverlap().
	//E * If setCastArea() isn't called, the default world size is used.
	//E * setCastArea() overrides internal broadphase proxy arrays.
	void setCastArea(const PfxVector3 &areaCenter,const PfxVector3 &areaExtent);
	void castSingleRay(const PfxRayInput &rayIn,PfxRayOutput &rayOut);
	void castRays(PfxRayInput *rayInputs,PfxRayOutput *rayOutputs,int numRays);

	//J	AABBオーバラップは結果を受け取るバッファとインデックスの最大数を引数に指定します。
	//J	AABBと交差した剛体のインデックスがintersectBuffに、数はnumIntesectionに格納されます。
	//J	処理が正常に完了した場合はSCE_PFX_OKを返しますが、バッファが不足した場合は
	//J SCE_PFX_ERR_OUT_OF_BUFFERを返します。
	//E Set the buffer which receives a result and maximum number of indices, then call findAabbOverlap().
	//E If AABB overlap is found, overlapped indices are added into intersectBuff and the number of overlap 
	//E is stored in numIntesection.
	//E If the process is finished correctly, the function returns SCE_PFX_OK.
	//E But it returns SCE_PFX_ERR_OUT_OF_BUFFER, if a given buffer is short.
	PfxInt32 findAabbOverlap(const PfxAabbInput &aabbInput,
		PfxUInt16 *intersectBuff,PfxUInt32 &numIntesection,PfxUInt32 maxIntesection);

	// Non contact pair
	void appendNonContactPair(PfxUInt16 rigidBodyIdA,PfxUInt16 rigidBodyIdB);
	void removeNonContactPair(PfxUInt16 rigidBodyIdA,PfxUInt16 rigidBodyIdB);
	PfxBool checkNonContactPair(PfxUInt16 rigidBodyIdA,PfxUInt16 rigidBodyIdB);

	// World operation
	const PfxVector3 &getWorldCenter() const {return m_worldCenter;}

	const PfxVector3 &getWorldExtent() const {return m_worldExtent;}

	const PfxVector3 &getGravity() const {return m_gravity;}

	PfxFloat getTimeStep() const {return m_timeStep;}

	PfxFloat getSeparateBias() const {return m_separateBias;}

	PfxUInt32 getIterationCount() const {return m_iteration;}

	void setWorldCenter(const PfxVector3 &worldCenter) {m_worldCenter = worldCenter;}

	void setWorldExtent(const PfxVector3 &worldExtent) {m_worldExtent = worldExtent;}

	void setGravity(const PfxVector3 &gravity) {m_gravity = gravity;}

	void setTimeStep(PfxFloat timeStep) {m_timeStep = timeStep;}

	void setSeparateBias(PfxFloat bias) {m_separateBias = bias;}

	void setIterationCount(PfxUInt32 iteration) {m_iteration = iteration;}
	
	// Rigid body operation
	inline PfxUInt32 addRigidBody(const PfxRigidState &state,const PfxRigidBody &body,const PfxCollidable &collidable);
	
	inline PfxUInt32 addJoint(const PfxJoint &joint);
	
	inline void updateJoint(PfxUInt16 jointId);
	
	inline void removeRigidBody(PfxUInt16 rigidBodyId);

	inline void removeJoint(PfxUInt16 jointId);

	inline bool isRemovedRigidBody(PfxUInt16 rigidBodyId) const;

	inline bool isRemovedJoint(PfxUInt16 jointId) const;

	PfxUInt32 getRigidBodyCount() const {return m_states.length();}

	PfxUInt32 getRigidBodyCapacity() const {return m_states.capacity();}

	PfxUInt32 getJointCount() const {return m_joints.length();}
	
	PfxUInt32 getJointCapacity() const {return m_joints.capacity();}

	PfxUInt32 getContactCount()	const {return m_numPairs[m_pairSwap];}

	PfxUInt32 getContactCapacity() const {return m_contacts.capacity();}

	PfxRigidState &getRigidState(PfxUInt16 rigidBodyId) {return m_states[rigidBodyId];}
	const PfxRigidState	&getRigidState(PfxUInt16 rigidBodyId) const {return m_states[rigidBodyId];}
	
	PfxRigidBody &getRigidBody(PfxUInt16 rigidBodyId) {return m_bodies[rigidBodyId];}
	const PfxRigidBody &getRigidBody(PfxUInt16 rigidBodyId) const {return m_bodies[rigidBodyId];}
	
	PfxCollidable &getCollidable(PfxUInt16 rigidBodyId) {return m_collidables[rigidBodyId];}
	const PfxCollidable &getCollidable(PfxUInt16 rigidBodyId) const {return m_collidables[rigidBodyId];}
	
	PfxJoint &getJoint(PfxUInt16 jointId) {return m_joints[jointId];}
	const PfxJoint &getJoint(PfxUInt16 jointId) const {return m_joints[jointId];}
	
	PfxContactManifold &getContactManifold(PfxUInt16 contactId) {return m_contacts[pfxGetConstraintId(m_pairsBuff[m_pairSwap][contactId])];}
	const PfxContactManifold &getContactManifold(PfxUInt16 contactId) const {return m_contacts[pfxGetConstraintId(m_pairsBuff[m_pairSwap][contactId])];}

	PfxRigidState *getRigidStatePtr() {return m_states.ptr();}
	
	PfxRigidBody *getRigidBodyPtr() {return m_bodies.ptr();}
	
	PfxCollidable *getCollidablePtr() {return m_collidables.ptr();}
	
	PfxJoint *getJointPtr() {return m_joints.ptr();}
	
	PfxContactManifold *getContactManifoldPtr() {return m_contacts.ptr();}

	PfxBroadphasePair *getContactPairPtr() {return m_pairsBuff[m_pairSwap];}

	// Simulation island
	PfxIsland *getIsland() {return m_island;}
	const PfxIsland *getIsland() const {return m_island;}

	// Collidable setup/release
	inline void setupCollidable(PfxCollidable &collidable,PfxUInt32 numShapes);
	inline void releaseCollidable(PfxCollidable &collidable);

	static void* operator new(size_t size) {
		return SCE_PFX_UTIL_ALLOC(16,size);
	}
	static void operator delete(void* pv) {
		SCE_PFX_UTIL_FREE(pv);
	}
};

#include "pfx_rigidbody_world_implementation.h"

} // namespace PhysicsEffects
} // namespace sce

#endif // _SCE_PFX_RIGIDBODY_WORLD_H
