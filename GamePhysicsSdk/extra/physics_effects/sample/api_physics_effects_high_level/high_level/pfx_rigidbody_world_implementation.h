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

#ifndef	_SCE_PFX_RIGIDBODY_WORLD_IMPLEMENTATION_H
#define _SCE_PFX_RIGIDBODY_WORLD_IMPLEMENTATION_H

inline
PfxUInt32 PfxRigidBodyWorld::addRigidBody(const PfxRigidState &state,const PfxRigidBody &body,const PfxCollidable &collidable)
{
	PfxUInt32 id = m_states.push(state);
	m_bodies.push(body);
	m_collidables.push(collidable);
	m_states[id].setRigidBodyId(id);
	return id;
}

inline
PfxUInt32 PfxRigidBodyWorld::addJoint(const PfxJoint &joint)
{
	PfxUInt32 id = m_joints.push(joint);
	m_jointPairs.push(PfxConstraintPair());
	pfxUpdateJointPairs(m_jointPairs[id],id,m_joints[id],
		m_states[m_joints[id].m_rigidBodyIdA],m_states[m_joints[id].m_rigidBodyIdB]);
	return id;
}

inline
void PfxRigidBodyWorld::updateJoint(PfxUInt16 jointId)
{
	pfxUpdateJointPairs(m_jointPairs[jointId],jointId,m_joints[jointId],
		m_states[m_joints[jointId].m_rigidBodyIdA],m_states[m_joints[jointId].m_rigidBodyIdB]);
}

inline void PfxRigidBodyWorld::removeRigidBody(PfxUInt16 rigidBodyId)
{
	//J	半径１の球体の固定剛体として、ワールド境界の外に配置して次回再利用に備える。
	//E Change removed rigid body into fixed sphere (radius=1.0f), and place outside of the world 
	//E to prepare for the next use.
	PfxVector3 outOfWorld = m_worldCenter + m_worldExtent;
	m_states[rigidBodyId].setMotionType(kPfxMotionTypeFixed);
	m_states[rigidBodyId].setPosition(outOfWorld + PfxVector3(1.0f));
	releaseCollidable(m_collidables[rigidBodyId]);

	m_states.remove(rigidBodyId);
	m_bodies.remove(rigidBodyId);
	m_collidables.remove(rigidBodyId);
}

inline void PfxRigidBodyWorld::removeJoint(PfxUInt16 jointId)
{
	m_joints[jointId].m_active = 0;
	pfxSetActive(m_jointPairs[jointId],false);
	m_joints.remove(jointId);
	m_jointPairs.remove(jointId);
}

inline bool PfxRigidBodyWorld::isRemovedRigidBody(PfxUInt16 rigidBodyId) const
{
	return m_states.isRemoved(rigidBodyId);
}

inline bool PfxRigidBodyWorld::isRemovedJoint(PfxUInt16 jointId) const
{
	return m_joints.isRemoved(jointId);
}

inline
void PfxRigidBodyWorld::setupCollidable(PfxCollidable &collidable,PfxUInt32 numShapes)
{
	PfxUInt16 ids[SCE_PFX_NUMPRIMS+1];
	for(PfxUInt32 i=0;i<numShapes;i++) {
		ids[i] = m_shapes.push(PfxShape());
	}
	collidable.reset(m_shapes.ptr(),ids,numShapes);
}

inline
void PfxRigidBodyWorld::releaseCollidable(PfxCollidable &collidable)
{
	for(PfxUInt32 i=0;i<collidable.getNumShapes();i++) {
		if(i>0) {
			m_shapes.remove(collidable.getShapeId(i));
		}
	}

	PfxShape shape;
	shape.reset();
	shape.setSphere(PfxSphere(1.0f));
	collidable.reset();
	collidable.addShape(shape);
	collidable.finish();
}
#endif//_SCE_PFX_RIGIDBODY_WORLD_IMPLEMENTATION_H
