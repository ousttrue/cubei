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

#include <new>

//E Enable performance counter
#ifdef NDEBUG
	#define SCE_PFX_USE_PERFCOUNTER
#endif

#include "pfx_rigidbody_world.h"

namespace sce{
namespace PhysicsEffects{

void PfxRigidBodyWorld::printPairs(const char *msg,PfxBroadphasePair *pairs,int n)
{
	SCE_PFX_PRINTF("--------- %u : %s ----------\n",m_frame,msg);
	for(int p=0;p<n;p++) {
		SCE_PFX_PRINTF("pair%d %d,%d key %u\n",p,pfxGetObjectIdA(pairs[p]),pfxGetObjectIdB(pairs[p]),pfxGetKey(pairs[p]));
		if(p > 0 && pfxGetKey(pairs[p-1]) == pfxGetKey(pairs[p])) {
			SCE_PFX_ALWAYS_ASSERT(false);
		}
	}
}

PfxUInt32 PfxRigidBodyWorld::getRigidBodyWorldBytes(const PfxRigidBodyWorldParam &param)
{
	PfxUInt32 dataBytes = 0;
	PfxUInt32 workBytes = 0;

	//---------------------------------------------------------
	// Data

	dataBytes += 128 + SCE_PFX_ALLOC_BYTES_ALIGN128(sizeof(PfxRigidState) * param.maxRigidBodies);
	dataBytes += 128 + SCE_PFX_ALLOC_BYTES_ALIGN128(sizeof(PfxRigidBody) * param.maxRigidBodies);
	dataBytes += 128 + SCE_PFX_ALLOC_BYTES_ALIGN128(sizeof(PfxCollidable) * param.maxRigidBodies);
	dataBytes += 128 + SCE_PFX_ALLOC_BYTES_ALIGN128(sizeof(PfxSolverBody) * param.maxRigidBodies);

	dataBytes += SCE_PFX_ALLOC_BYTES_ALIGN16(sizeof(PfxUInt32) * param.maxRigidBodies) * 3; // used in PfxPoolArray
	dataBytes += SCE_PFX_ALLOC_BYTES_ALIGN16(sizeof(PfxUInt32) * ((param.maxRigidBodies+31)>>5)) * 3; // used in PfxPoolArray

	dataBytes += 128 + SCE_PFX_ALLOC_BYTES_ALIGN128(sizeof(PfxShape) * param.maxShapes);
	dataBytes += SCE_PFX_ALLOC_BYTES_ALIGN16(sizeof(PfxUInt32) * param.maxShapes); // used in PfxPoolArray
	dataBytes += SCE_PFX_ALLOC_BYTES_ALIGN16(sizeof(PfxUInt32) * ((param.maxShapes+31)>>5)); // used in PfxPoolArray
	dataBytes += 128 + SCE_PFX_ALLOC_BYTES_ALIGN128(sizeof(PfxJoint) * param.maxJoints);
	dataBytes += 128 + SCE_PFX_ALLOC_BYTES_ALIGN128(sizeof(PfxConstraintPair) * param.maxJoints);
	dataBytes += SCE_PFX_ALLOC_BYTES_ALIGN16(sizeof(PfxUInt32) * param.maxJoints) * 2; // used in PfxPoolArray
	dataBytes += SCE_PFX_ALLOC_BYTES_ALIGN16(sizeof(PfxUInt32) * ((param.maxJoints+31)>>5)) * 2; // used in PfxPoolArray
	dataBytes += 128 + SCE_PFX_ALLOC_BYTES_ALIGN128(sizeof(PfxContactManifold) * param.maxContacts);
	dataBytes += SCE_PFX_ALLOC_BYTES_ALIGN16(sizeof(PfxUInt32) * param.maxContacts); // used in PfxPoolArray
	dataBytes += SCE_PFX_ALLOC_BYTES_ALIGN16(sizeof(PfxUInt32) * ((param.maxContacts+31)>>5)); // used in PfxPoolArray
	dataBytes += 128 + SCE_PFX_ALLOC_BYTES_ALIGN128(sizeof(PfxBroadphaseProxy) * param.maxRigidBodies) * 6;
	dataBytes += SCE_PFX_ALLOC_BYTES_ALIGN128(sizeof(PfxBroadphasePair) * param.maxContacts) * 2;
	dataBytes += SCE_PFX_ALLOC_BYTES_ALIGN128(pfxGetIslandBytesOfGenerateIsland(param.maxRigidBodies));
	dataBytes += SCE_PFX_ALLOC_BYTES_ALIGN16(sizeof(PfxUInt32) * param.maxNonContactPairs);

	//---------------------------------------------------------
	// Work

	PfxUInt32 tmp;

	// Broadphase
	tmp = 128 + SCE_PFX_ALLOC_BYTES_ALIGN128(sizeof(PfxBroadphaseProxy) * param.maxRigidBodies) * 6;
	workBytes = SCE_PFX_MAX(workBytes,tmp);
	tmp = 128 + SCE_PFX_ALLOC_BYTES_ALIGN128(sizeof(PfxBroadphasePair) * param.maxContacts);
	workBytes = SCE_PFX_MAX(workBytes,tmp);
	tmp = 16 + SCE_PFX_ALLOC_BYTES_ALIGN16(sizeof(PfxUInt32) * param.maxNonContactPairs);
	workBytes = SCE_PFX_MAX(workBytes,tmp);
	tmp = pfxGetWorkBytesOfUpdateBroadphaseProxies(param.maxContacts);
	workBytes = SCE_PFX_MAX(workBytes,tmp);
	tmp = pfxGetWorkBytesOfFindPairs(param.maxContacts,1);
	workBytes = SCE_PFX_MAX(workBytes,tmp);
	tmp = pfxGetWorkBytesOfDecomposePairs(param.maxContacts,param.maxContacts,1) + 
		sizeof(PfxBroadphasePair) * param.maxContacts * 3;
	workBytes = SCE_PFX_MAX(workBytes,tmp);
	
	// Constraint solver
	tmp = pfxGetWorkBytesOfSolveConstraints(param.maxRigidBodies,param.maxContacts,param.maxJoints);
	workBytes = SCE_PFX_MAX(workBytes,tmp);
	
	PfxUInt32 totalBytes = dataBytes + workBytes;
	
	return totalBytes;
}

///////////////////////////////////////////////////////////////////////////////
// World Constractor / Destructor

PfxRigidBodyWorld::PfxRigidBodyWorld(const PfxRigidBodyWorldParam &param) : 
	m_pool((PfxUInt8*)param.poolBuff,param.poolBytes),
	m_worldCenter(param.worldCenter),m_worldExtent(param.worldExtent),m_gravity(param.gravity),
	m_states(&m_pool,param.maxRigidBodies),
	m_bodies(&m_pool,param.maxRigidBodies),
	m_collidables(&m_pool,param.maxRigidBodies),
	m_shapes(&m_pool,param.maxShapes),
	m_jointPairs(&m_pool,param.maxJoints),
	m_joints(&m_pool,param.maxJoints),
	m_contacts(&m_pool,param.maxContacts),
	m_maxNonContactPairs(param.maxNonContactPairs),
	m_timeStep(param.timeStep),
	m_separateBias(param.separateBias),
	m_iteration(param.iteration),
	m_sleepCount(param.sleepCount),
	m_sleepVelocity(param.sleepVelocity),
	m_simulationFlag(param.simulationFlag)
{
	SCE_PFX_ALWAYS_ASSERT_MSG(param.poolBytes >= getRigidBodyWorldBytes(param),"Can't create the world because of pool memory shortage.");

	m_solverBodies = (PfxSolverBody*)m_pool.allocate(sizeof(PfxSolverBody)*param.maxRigidBodies,128);
	m_proxies[0] = (PfxBroadphaseProxy*)m_pool.allocate(sizeof(PfxBroadphaseProxy)*param.maxRigidBodies,128);
	m_proxies[1] = (PfxBroadphaseProxy*)m_pool.allocate(sizeof(PfxBroadphaseProxy)*param.maxRigidBodies,128);
	m_proxies[2] = (PfxBroadphaseProxy*)m_pool.allocate(sizeof(PfxBroadphaseProxy)*param.maxRigidBodies,128);
	m_proxies[3] = (PfxBroadphaseProxy*)m_pool.allocate(sizeof(PfxBroadphaseProxy)*param.maxRigidBodies,128);
	m_proxies[4] = (PfxBroadphaseProxy*)m_pool.allocate(sizeof(PfxBroadphaseProxy)*param.maxRigidBodies,128);
	m_proxies[5] = (PfxBroadphaseProxy*)m_pool.allocate(sizeof(PfxBroadphaseProxy)*param.maxRigidBodies,128);
	m_pairsBuff[0] = (PfxBroadphasePair*)m_pool.allocate(sizeof(PfxBroadphasePair)*param.maxContacts,128);
	m_pairsBuff[1] = (PfxBroadphasePair*)m_pool.allocate(sizeof(PfxBroadphasePair)*param.maxContacts,128);
	m_nonContactPairs = (PfxUInt32*)m_pool.allocate(sizeof(PfxUInt32)*param.maxNonContactPairs);
	m_islandBytes = pfxGetIslandBytesOfGenerateIsland(param.maxRigidBodies);
	m_islandBuff = m_pool.allocate(m_islandBytes,128);
	m_island = NULL;
}

PfxRigidBodyWorld::~PfxRigidBodyWorld()
{
}

///////////////////////////////////////////////////////////////////////////////
// Initialize / Finalize

void PfxRigidBodyWorld::initialize()
{
	reset();
}

void PfxRigidBodyWorld::finalize()
{
}

///////////////////////////////////////////////////////////////////////////////
// Reset

void PfxRigidBodyWorld::reset()
{
	m_frame = 0;
	m_pairSwap = 1;
	m_numPairs[0] = m_numPairs[1] = 0;
	m_states.clear();
	m_bodies.clear();
	m_collidables.clear();
	m_shapes.clear();
	m_jointPairs.clear();
	m_joints.clear();
	m_contacts.clear();
	m_numNonContactPairs = 0;
	m_numProxiesInWorld = 0;
	if(m_island) pfxResetIsland(m_island);
}

///////////////////////////////////////////////////////////////////////////////
// Simulation Method

void PfxRigidBodyWorld::broadphase()
{
	m_pairSwap = 1-m_pairSwap;

	unsigned int &previousNumPairs = m_numPairs[1-m_pairSwap];
	unsigned int &currentNumPairs = m_numPairs[m_pairSwap];
	PfxBroadphasePair *previousPairs = m_pairsBuff[1-m_pairSwap];
	PfxBroadphasePair *currentPairs = m_pairsBuff[m_pairSwap];

	//J 剛体が最も分散している軸を見つける
	//E Find the axis along which all rigid m_bodies are most widely positioned
	int axis = 0;
	{
		PfxVector3 s(0.0f),s2(0.0f);
		unsigned int i=0;
		for(;i<m_states.length();i++) {
			if(!m_states.isRemoved(i)) {
				PfxVector3 c = m_states[i].getPosition();
				s += c;
				s2 += mulPerElem(c,c);
			}
		}
		PfxVector3 v = s2 - mulPerElem(s,s) / (float)i;
		if(v[1] > v[0]) axis = 1;
		if(v[2] > v[axis]) axis = 2;
	}

	//J ブロードフェーズプロキシの更新
	//E Create broadpahse proxies
	{
		PfxUInt32 workBytes = pfxGetWorkBytesOfUpdateBroadphaseProxies(m_states.length(),1);

		PfxUpdateBroadphaseProxiesParam param;
		param.workBuff = m_pool.allocate(workBytes,128);
		param.workBytes = workBytes;
		param.numRigidBodies = m_states.length();
		param.offsetRigidStates = m_states.ptr();
		param.offsetCollidables = m_collidables.ptr();
		param.proxiesX = m_proxies[0];
		param.proxiesY = m_proxies[1];
		param.proxiesZ = m_proxies[2];
		param.proxiesXb = m_proxies[3];
		param.proxiesYb = m_proxies[4];
		param.proxiesZb = m_proxies[5];
		param.worldCenter = m_worldCenter;
		param.worldExtent = m_worldExtent;
		param.outOfWorldBehavior = SCE_PFX_OUT_OF_WORLD_BEHAVIOR_FIX_MOTION | SCE_PFX_OUT_OF_WORLD_BEHAVIOR_REMOVE_PROXY;
		
		PfxUpdateBroadphaseProxiesResult result;
		
		int ret = pfxUpdateBroadphaseProxies(param,result);
		if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxUpdateBroadphaseProxies failed %d\n",ret);
		
		m_pool.deallocate(param.workBuff);

		m_numProxiesInWorld =  m_states.length() - result.numOutOfWorldProxies;
	}

	m_areaCenter = m_worldCenter;
	m_areaExtent = m_worldExtent;

	//J 交差ペア探索
	//E Find overlapped pairs
	{
		PfxUInt32 workBytes = pfxGetWorkBytesOfFindPairs(m_contacts.capacity());
		void *workBuff = m_pool.allocate(workBytes,128);

		PfxFindPairsParam param;
		param.workBuff = workBuff;
		param.workBytes = workBytes;
		param.pairBuff = currentPairs;
		param.pairBytes = sizeof(PfxBroadphasePair)*m_contacts.capacity();
		param.proxies = m_proxies[axis];
		param.numProxies = m_numProxiesInWorld;
		param.maxPairs = m_contacts.capacity();
		param.axis = axis;

		PfxFindPairsResult result;

		int ret = pfxFindPairs(param,result);
		if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxFindPairs failed %d\n",ret);

		currentPairs = result.pairs;
		currentNumPairs = result.numPairs;

		m_pool.deallocate(workBuff);
	}
	
	//printPairs("find pairs",currentPairs,currentNumPairs);
	
	//J ノンコンタクトペアとして指定されているペアを排除する
	//E Remove pairs specified as a non-contact pair
	{
		PfxUInt32 i = 0,j = 0;
		PfxUInt32 numRemovedPairs = 0;
		while(i < m_numNonContactPairs && j < currentNumPairs) {
			PfxUInt32 key1 = m_nonContactPairs[i];
			PfxUInt32 key2 = pfxGetKey(currentPairs[j]);

			if(key1 < key2) {
				i++;
			}
			else if(key1 > key2) {
				j++;
			}
			else {// key1 == key2
				pfxSetKey(currentPairs[j],SCE_PFX_SENTINEL_KEY);
				numRemovedPairs++;
				i++;
				j++;
			}
		}

		if(numRemovedPairs > 0) {
			void *workBuff = m_pool.allocate(sizeof(PfxBroadphasePair)*currentNumPairs,128);
			pfxSort(currentPairs,(PfxBroadphasePair*)workBuff,currentNumPairs);
			m_pool.deallocate(workBuff);
			currentNumPairs -= numRemovedPairs;
		}
	}

	//printPairs("remove non contact pairs",currentPairs,currentNumPairs);

	//J 交差ペア合成
	//E Decompose overlapped pairs into 3 arrays
	{
		PfxUInt32 workBytes;
		workBytes = pfxGetWorkBytesOfDecomposePairs(previousNumPairs,currentNumPairs,1);
		void *workBuff = m_pool.allocate(workBytes,128);

		PfxBroadphasePair *pairBuffer = (PfxBroadphasePair*)m_pool.allocate(sizeof(PfxBroadphasePair)*m_contacts.capacity()*3,128);

		PfxDecomposePairsParam param;
		param.workBuff = workBuff;
		param.workBytes = workBytes;
		param.pairBuff = pairBuffer;
		param.pairBytes = sizeof(PfxBroadphasePair)*m_contacts.capacity()*3;
		param.previousPairs = previousPairs;
		param.numPreviousPairs = previousNumPairs;
		param.currentPairs = currentPairs;
		param.numCurrentPairs = currentNumPairs;

		PfxDecomposePairsResult result;

		int ret = pfxDecomposePairs(param,result);
		if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxDecomposePairs failed %d\n",ret);

		PfxBroadphasePair *outNewPairs = result.outNewPairs;
		PfxBroadphasePair *outKeepPairs = result.outKeepPairs;
		PfxBroadphasePair *outRemovePairs = result.outRemovePairs;
		PfxUInt32 numOutNewPairs = result.numOutNewPairs;
		PfxUInt32 numOutKeepPairs = result.numOutKeepPairs;
		PfxUInt32 numOutRemovePairs = result.numOutRemovePairs;

		//J 廃棄コンタクトをプールに戻す
		//E Put removed contacts into the contact pool
		for(PfxUInt32 i=0;i<numOutRemovePairs;i++) {
			SCE_PFX_ALWAYS_ASSERT_MSG(m_contacts.remove(pfxGetContactId(outRemovePairs[i])),"can't remove a contact");

			//J 寝てる剛体を起こす
			//E Wake up sleeping rigid bodies
			PfxRigidState &stateA = m_states[pfxGetObjectIdA(outRemovePairs[i])];
			if(stateA.isAsleep()) {
				stateA.wakeup();
			}
			PfxRigidState &stateB = m_states[pfxGetObjectIdB(outRemovePairs[i])];
			if(stateB.isAsleep()) {
				stateB.wakeup();
			}
		}

		//J 新規ペアのコンタクトを初期化
		//E Add new contacts and initialize
		for(PfxUInt32 i=0;i<numOutNewPairs;i++) {
			PfxContactManifold contact;
			contact.reset(pfxGetObjectIdA(outNewPairs[i]),pfxGetObjectIdB(outNewPairs[i]));

			PfxUInt32 contactId = m_contacts.push(contact);
			pfxSetContactId(outNewPairs[i],contactId);

			//J 寝てる剛体を起こす
			//E Wake up sleeping rigid bodies
			PfxRigidState &stateA = m_states[pfxGetObjectIdA(outNewPairs[i])];
			PfxRigidState &stateB = m_states[pfxGetObjectIdB(outNewPairs[i])];
			if(stateA.isAsleep()) {
				stateA.wakeup();
			}
			if(stateB.isAsleep()) {
				stateB.wakeup();
			}
		}

		//J 合成
		//E Merge
		currentNumPairs = 0;
		for(PfxUInt32 i=0;i<numOutKeepPairs;i++) {
			currentPairs[currentNumPairs++] = outKeepPairs[i];
		}
		for(PfxUInt32 i=0;i<numOutNewPairs;i++) {
			currentPairs[currentNumPairs++] = outNewPairs[i];
		}
		
		m_pool.deallocate(pairBuffer);
		m_pool.deallocate(workBuff);
	}

	//printPairs("decompose pairs",currentPairs,currentNumPairs);

	{
		PfxUInt32 workBytes = sizeof(PfxBroadphasePair) * currentNumPairs;
		PfxBroadphasePair *workBuff = (PfxBroadphasePair*)m_pool.allocate(workBytes,128);

		pfxSort(currentPairs,workBuff,currentNumPairs);

		m_pool.deallocate(workBuff);
	}

	//printPairs("sort pairs",currentPairs,currentNumPairs);
}

void PfxRigidBodyWorld::collision()
{
	unsigned int currentNumPairs = m_numPairs[m_pairSwap];
	PfxBroadphasePair *currentPairs = m_pairsBuff[m_pairSwap];

	//J リフレッシュ
	//E Refresh contacts
	{
		PfxRefreshContactsParam param;
		param.contactPairs = currentPairs;
		param.numContactPairs = currentNumPairs;
		param.offsetContactManifolds = m_contacts.ptr();
		param.offsetRigidStates = m_states.ptr();
		param.numRigidBodies = m_states.length();
		
		int ret = pfxRefreshContacts(param);
		if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxRefreshContacts failed %d\n",ret);
	}

	//printPairs("refresh",currentPairs,currentNumPairs);

	//J 衝突検出
	//E Detect collisions
	{
		PfxDetectCollisionParam param;
		param.contactPairs = currentPairs;
		param.numContactPairs = currentNumPairs;
		param.offsetContactManifolds = m_contacts.ptr();
		param.offsetRigidStates = m_states.ptr();
		param.offsetCollidables = m_collidables.ptr();
		param.numRigidBodies = m_states.length();

		int ret = pfxDetectCollision(param);
		if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxDetectCollision failed %d\n",ret);
	}

	//J アイランド生成
	//E Create simulation islands
	{
		PfxGenerateIslandParam param;
		param.islandBuff = m_islandBuff;
		param.islandBytes = m_islandBytes;
		param.pairs = currentPairs;
		param.numPairs = currentNumPairs;
		param.numObjects = m_states.length();

		PfxGenerateIslandResult result;

		int ret = pfxGenerateIsland(param,result);
		if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxGenerateIsland failed %d\n",ret);
		m_island = result.island;

		//J ジョイント分のペアを追加
		//E Add joint pairs to islands
		ret = pfxAppendPairs(m_island,m_jointPairs.ptr(),m_jointPairs.length());
		if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxAppendPairs failed %d\n",ret);
	}
}

void PfxRigidBodyWorld::constraintSolver()
{
	PfxPerfCounter pc;

	unsigned int &currentNumPairs = m_numPairs[m_pairSwap];
	PfxBroadphasePair *currentPairs = m_pairsBuff[m_pairSwap];

	pc.countBegin("setup solver bodies");
	{
		PfxSetupSolverBodiesParam param;
		param.states = m_states.ptr();
		param.bodies = m_bodies.ptr();
		param.solverBodies = m_solverBodies;
		param.numRigidBodies = m_states.length();
		
		int ret = pfxSetupSolverBodies(param);
		if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxSetupSolverBodies failed %d\n",ret);
	}
	pc.countEnd();

	pc.countBegin("setup contact constraints");
	{
		PfxSetupContactConstraintsParam param;
		param.contactPairs = currentPairs;
		param.numContactPairs = currentNumPairs;
		param.offsetContactManifolds = m_contacts.ptr();
		param.offsetRigidStates = m_states.ptr();
		param.offsetRigidBodies = m_bodies.ptr();
		param.offsetSolverBodies = m_solverBodies;
		param.numRigidBodies = m_states.length();
		param.timeStep = m_timeStep;
		param.separateBias = m_separateBias;
		
		int ret = pfxSetupContactConstraints(param);
		if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxSetupJointConstraints failed %d\n",ret);
	}
	pc.countEnd();

	pc.countBegin("setup joint constraints");
	{
		PfxSetupJointConstraintsParam param;
		param.jointPairs = m_jointPairs.ptr();
		param.numJointPairs = m_joints.length();
		param.offsetJoints = m_joints.ptr();
		param.offsetRigidStates = m_states.ptr();
		param.offsetRigidBodies = m_bodies.ptr();
		param.offsetSolverBodies = m_solverBodies;
		param.numRigidBodies = m_states.length();
		param.timeStep = m_timeStep;

		for(unsigned int i=0;i<m_joints.length();i++) {
			PfxJoint &joint = m_joints[i];
			PfxRigidState &stateA = m_states[joint.m_rigidBodyIdA];
			PfxRigidState &stateB = m_states[joint.m_rigidBodyIdB];
			pfxUpdateJointPairs(m_jointPairs[i],i,joint,stateA,stateB);
		}

		int ret = pfxSetupJointConstraints(param);
		if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxSetupJointConstraints failed %d\n",ret);
	}
	pc.countEnd();

	pc.countBegin("solve constraints");
	{
		PfxUInt32 workBytes = pfxGetWorkBytesOfSolveConstraints(m_states.length(),currentNumPairs,m_joints.length());
		void *workBuff = m_pool.allocate(workBytes,128);

		PfxSolveConstraintsParam param;
		param.workBuff = workBuff;
		param.workBytes = workBytes;
		param.contactPairs = currentPairs;
		param.numContactPairs = currentNumPairs;
		param.offsetContactManifolds = m_contacts.ptr();
		param.jointPairs = m_jointPairs.ptr();
		param.numJointPairs = m_joints.length();
		param.offsetJoints = m_joints.ptr();
		param.offsetRigidStates = m_states.ptr();
		param.offsetSolverBodies = m_solverBodies;
		param.numRigidBodies = m_states.length();
		param.iteration = m_iteration;

		int ret = pfxSolveConstraints(param);
		if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxSolveConstraints failed %d\n",ret);

		m_pool.deallocate(workBuff);
	}
	pc.countEnd();

	//pc.printCount();
}

void PfxRigidBodyWorld::sleepOrWakeup()
{
	PfxFloat sleepVelSqr = m_sleepVelocity * m_sleepVelocity;

	for(PfxUInt32 i=0;i<(PfxUInt32)m_states.length();i++) {
		if(m_states.isRemoved(i)) continue;

		PfxRigidState &state = m_states[i];
		if(SCE_PFX_MOTION_MASK_CAN_SLEEP(state.getMotionType())) {
			PfxFloat linVelSqr = lengthSqr(state.getLinearVelocity());
			PfxFloat angVelSqr = lengthSqr(state.getAngularVelocity());

			if(state.isAwake()) {
				if( linVelSqr < sleepVelSqr && angVelSqr < sleepVelSqr ) {
					state.incrementSleepCount();
				}
				else {
					state.resetSleepCount();
				}
			}
		}
	}

	if(m_island) {
		for(PfxUInt32 i=0;i<pfxGetNumIslands(m_island);i++) {
			int numActive = 0;
			int numSleep = 0;
			int numCanSleep = 0;
			
			{
				PfxIslandUnit *islandUnit = pfxGetFirstUnitInIsland(m_island,(PfxUInt32)i);
				for(;islandUnit!=NULL;islandUnit=pfxGetNextUnitInIsland(islandUnit)) {
					if(!(SCE_PFX_MOTION_MASK_CAN_SLEEP(m_states[pfxGetUnitId(islandUnit)].getMotionType()))) continue;
					PfxRigidState &state = m_states[pfxGetUnitId(islandUnit)];
					if(state.isAsleep()) {
						numSleep++;
					}
					else {
						numActive++;
						if(state.getSleepCount() > m_sleepCount) {
							numCanSleep++;
						}
					}
				}
			}
			
			// Deactivate Island
			if(numCanSleep > 0 && numCanSleep == numActive + numSleep) {
				PfxIslandUnit *islandUnit = pfxGetFirstUnitInIsland(m_island,(PfxUInt32)i);
				for(;islandUnit!=NULL;islandUnit=pfxGetNextUnitInIsland(islandUnit)) {
					if(!(SCE_PFX_MOTION_MASK_CAN_SLEEP(m_states[pfxGetUnitId(islandUnit)].getMotionType()))) continue;
					m_states[pfxGetUnitId(islandUnit)].sleep();
				}
			}

			// Activate Island
			else if(numSleep > 0 && numActive > 0) {
				PfxIslandUnit *islandUnit = pfxGetFirstUnitInIsland(m_island,(PfxUInt32)i);
				for(;islandUnit!=NULL;islandUnit=pfxGetNextUnitInIsland(islandUnit)) {
					if(!(SCE_PFX_MOTION_MASK_CAN_SLEEP(m_states[pfxGetUnitId(islandUnit)].getMotionType()))) continue;
					m_states[pfxGetUnitId(islandUnit)].wakeup();
				}
			}
		}
	}
}

void PfxRigidBodyWorld::integrate()
{
	PfxUpdateRigidStatesParam param;
	param.states = m_states.ptr();
	param.bodies = m_bodies.ptr();
	param.numRigidBodies = m_states.length();
	param.timeStep = m_timeStep;
	
	int ret = pfxUpdateRigidStates(param);
	if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxUpdateRigidStates failed %d\n",ret);
}

void PfxRigidBodyWorld::simulate()
{
	PfxPerfCounter pc;

	for(unsigned int i=0;i<m_states.length();i++) {
		if(!m_states.isRemoved(i)) {
			pfxApplyExternalForce(
				m_states[i],m_bodies[i],
				m_bodies[i].getMass() * m_gravity,
				PfxVector3(0.0f),
				m_timeStep);
		}
	}

	SCE_PFX_PUSH_MARKER_HUD("broadphase",0x80ff0000);
	pc.countBegin("broadphase");
	broadphase();
	pc.countEnd();
	SCE_PFX_POP_MARKER();

	SCE_PFX_PUSH_MARKER_HUD("collision",0x8000ff00);
	pc.countBegin("collision");
	collision();
	pc.countEnd();
	SCE_PFX_POP_MARKER();

	SCE_PFX_PUSH_MARKER_HUD("solver",0x800000ff);
	pc.countBegin("solver");
	constraintSolver();
	pc.countEnd();
	SCE_PFX_POP_MARKER();

	SCE_PFX_PUSH_MARKER_HUD("sleepOrWakeup",0x80ff00ff);
	pc.countBegin("sleepOrWakeup");
	if(m_simulationFlag & SCE_PFX_ENABLE_SLEEP) {
		sleepOrWakeup();
	}
	pc.countEnd();
	SCE_PFX_POP_MARKER();

	SCE_PFX_PUSH_MARKER_HUD("integrate",0x80ffff00);
	pc.countBegin("integrate");
	integrate();
	pc.countEnd();
	SCE_PFX_POP_MARKER();

	m_frame++;

	#ifdef SCE_PFX_USE_PERFCOUNTER
	if(m_frame%100 == 0) {
		float broadphaseTime = pc.getCountTime(0);
		float collisionTime  = pc.getCountTime(2);
		float solverTime     = pc.getCountTime(4);
		float sleepTime      = pc.getCountTime(6);
		float integrateTime  = pc.getCountTime(8);
		SCE_PFX_PRINTF("frame %3d broadphase %.2f collision %.2f solver %.2f sleep %.2f integrate %.2f | total %.2f\n",m_frame,
			broadphaseTime,collisionTime,solverTime,sleepTime,integrateTime,
			broadphaseTime+collisionTime+solverTime+sleepTime+integrateTime);
	}
	#endif
}

PfxBool PfxRigidBodyWorld::checkNonContactPair(PfxUInt16 rigidBodyIdA,PfxUInt16 rigidBodyIdB)
{
	if(m_numNonContactPairs == 0) return false;

	PfxUInt32 key = pfxCreateUniqueKey(rigidBodyIdA,rigidBodyIdB);

	int left = 0;
	int right = (int)m_numNonContactPairs-1;
	int mid;

	while(left < right) {
		mid = (left + right) / 2;
		if(m_nonContactPairs[mid] == key) {
			return true;
		}
		else if(m_nonContactPairs[mid] < key) {
			left = mid + 1;
		}
		else { // m_nonContactPairs[mid] > key
			right = mid - 1;
		}
	}

	return false;
}

void PfxRigidBodyWorld::appendNonContactPair(PfxUInt16 rigidBodyIdA,PfxUInt16 rigidBodyIdB)
{
	SCE_PFX_ALWAYS_ASSERT(m_numNonContactPairs < m_maxNonContactPairs);

	PfxUInt32 key = pfxCreateUniqueKey(rigidBodyIdA,rigidBodyIdB);

	if(m_numNonContactPairs == 0) {
		m_nonContactPairs[m_numNonContactPairs++] = key;
		return;
	}

	int left = 0;
	int right = (int)m_numNonContactPairs-1;
	int mid;

	while(left <= right) {
		mid = (left + right) / 2;
		if(m_nonContactPairs[mid] == key) {
			return;
		}
		else if(m_nonContactPairs[mid] < key) {
			left = mid + 1;
		}
		else { // m_nonContactPairs[mid] > key
			right = mid - 1;
		}
	}

	m_numNonContactPairs++;

	for(int i=left;i<(int)m_numNonContactPairs;i++) {
		PfxUInt32 tmp = m_nonContactPairs[i];
		m_nonContactPairs[i] = key;
		key = tmp;
	}
}

void PfxRigidBodyWorld::removeNonContactPair(PfxUInt16 rigidBodyIdA,PfxUInt16 rigidBodyIdB)
{
	if(m_numNonContactPairs == 0) return;

	PfxUInt32 key = pfxCreateUniqueKey(rigidBodyIdA,rigidBodyIdB);

	int left = 0;
	int right = (int)m_numNonContactPairs-1;
	int mid;

	while(left <= right) {
		mid = (left + right) / 2;
		if(m_nonContactPairs[mid] == key) {
			for(int i=mid;i<(int)m_numNonContactPairs-1;i++) {
				m_nonContactPairs[i] = m_nonContactPairs[i+1];
			}
			m_numNonContactPairs--;
			
			return;
		}
		else if(m_nonContactPairs[mid] < key) {
			left = mid + 1;
		}
		else { // m_nonContactPairs[mid] > key
			right = mid - 1;
		}
	}

	return;
}

void PfxRigidBodyWorld::setCastArea(const PfxVector3 &areaCenter,const PfxVector3 &areaExtent)
{
	PfxUInt32 workBytes = pfxGetWorkBytesOfUpdateBroadphaseProxies(m_states.length(),1);

	m_areaCenter = areaCenter;
	m_areaExtent = areaExtent;

	PfxUpdateBroadphaseProxiesParam param;
	param.workBuff = m_pool.allocate(workBytes,128);
	param.workBytes = workBytes;
	param.numRigidBodies = m_states.length();
	param.offsetRigidStates = m_states.ptr();
	param.offsetCollidables = m_collidables.ptr();
	param.proxiesX = m_proxies[0];
	param.proxiesY = m_proxies[1];
	param.proxiesZ = m_proxies[2];
	param.proxiesXb = m_proxies[3];
	param.proxiesYb = m_proxies[4];
	param.proxiesZb = m_proxies[5];
	param.worldCenter = m_areaCenter;
	param.worldExtent = m_areaExtent;
	param.outOfWorldBehavior = SCE_PFX_OUT_OF_WORLD_BEHAVIOR_REMOVE_PROXY;
	
	PfxUpdateBroadphaseProxiesResult result;
	
	int ret = pfxUpdateBroadphaseProxies(param,result);
	if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxUpdateBroadphaseProxies failed %d\n",ret);
	
	m_pool.deallocate(param.workBuff);

	m_numProxiesInWorld = m_states.length() - result.numOutOfWorldProxies;
}

void PfxRigidBodyWorld::castSingleRay(const PfxRayInput &rayIn,PfxRayOutput &rayOut)
{
	PfxRayCastParam param;
	param.offsetRigidStates = m_states.ptr();
	param.offsetCollidables = m_collidables.ptr();
	param.proxiesX = m_proxies[0];
	param.proxiesY = m_proxies[1];
	param.proxiesZ = m_proxies[2];
	param.proxiesXb = m_proxies[3];
	param.proxiesYb = m_proxies[4];
	param.proxiesZb = m_proxies[5];
	param.numProxies = m_numProxiesInWorld;
	param.rangeCenter = m_areaCenter;
	param.rangeExtent = m_areaExtent;
	
	pfxCastSingleRay(rayIn,rayOut,param);
}

void PfxRigidBodyWorld::castRays(PfxRayInput *rayInputs,PfxRayOutput *rayOutputs,int numRays)
{
	PfxPerfCounter pc;

	PfxRayCastParam param;
	param.offsetRigidStates = m_states.ptr();
	param.offsetCollidables = m_collidables.ptr();
	param.proxiesX = m_proxies[0];
	param.proxiesY = m_proxies[1];
	param.proxiesZ = m_proxies[2];
	param.proxiesXb = m_proxies[3];
	param.proxiesYb = m_proxies[4];
	param.proxiesZb = m_proxies[5];
	param.numProxies = m_numProxiesInWorld;
	param.rangeCenter = m_areaCenter;
	param.rangeExtent = m_areaExtent;

	pc.countBegin("raycast");
	pfxCastRays(rayInputs,rayOutputs,numRays,param);
	pc.countEnd();

	#ifdef SCE_PFX_USE_PERFCOUNTER
	SCE_PFX_PRINTF("raycast %fmsec\n",pc.getCountTime(0));
	#endif
}

PfxInt32 PfxRigidBodyWorld::findAabbOverlap(
	const PfxAabbInput &aabbInput,
	PfxUInt16 *intersectBuff,PfxUInt32 &numIntesection,PfxUInt32 maxIntesection)
{
	PfxPerfCounter pc;
	pc.countBegin("AABB cast");
	{
		PfxVector3 chkAxisVec = absPerElem(aabbInput.extent);
		int axis = 0;
		if(chkAxisVec[1] < chkAxisVec[0]) axis = 1;
		if(chkAxisVec[2] < chkAxisVec[axis]) axis = 2;
		
		PfxVecInt3 aabbMin,aabbMax;
		pfxConvertCoordWorldToLocal(m_areaCenter,m_areaExtent,
			aabbInput.center - aabbInput.extent,aabbInput.center + aabbInput.extent,aabbMin,aabbMax);
		
		PfxAabb16 aabb;
		pfxSetXMin(aabb,aabbMin.getX());
		pfxSetXMax(aabb,aabbMax.getX());
		pfxSetYMin(aabb,aabbMin.getY());
		pfxSetYMax(aabb,aabbMax.getY());
		pfxSetZMin(aabb,aabbMin.getZ());
		pfxSetZMax(aabb,aabbMax.getZ());

		numIntesection = 0;

		for(PfxUInt32 i=0;i<m_numProxiesInWorld;i++) {
			PfxBroadphaseProxy &proxy = m_proxies[axis][i];

			if(pfxGetXYZMax(aabb,axis) < pfxGetXYZMin(proxy,axis)) {
				break;
			}

			if(pfxGetXYZMax(proxy,axis) < pfxGetXYZMin(aabb,axis)) {
				continue;
			}

			PfxUInt16 rigidbodyId = pfxGetObjectId(proxy);
			PfxUInt32 contactFilterSelf = pfxGetSelf(proxy);
			PfxUInt32 contactFilterTarget = pfxGetTarget(proxy);

			if((aabbInput.contactFilterSelf&contactFilterTarget) && 
				(aabbInput.contactFilterTarget&contactFilterSelf) && 
				pfxTestAabb(aabb,proxy)) {
				if(numIntesection >= maxIntesection) return SCE_PFX_ERR_OUT_OF_BUFFER;
				intersectBuff[numIntesection++] = rigidbodyId;
			}
		}
	}
	pc.countEnd();

	#ifdef SCE_PFX_USE_PERFCOUNTER
	SCE_PFX_PRINTF("AABB overlap %fmsec\n",pc.getCountTime(0));
	#endif

	return SCE_PFX_OK;
}

} // namespace PhysicsEffects
} // namespace sce
