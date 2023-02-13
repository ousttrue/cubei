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

#include "EpxCollisionDetection.h"
#include "../collision/EpxConvexConvexContact.h"

namespace EasyPhysics {

void epxDetectCollision(
	const EpxState *states,
	const EpxCollidable *collidables,
	EpxUInt32 numRigidBodies,
	const EpxPair *pairs,
	EpxUInt32 numPairs)
{
	assert(states);
	assert(collidables);
	assert(pairs);
	
	for(EpxUInt32 i=0;i<numPairs;i++) {
		const EpxPair &pair = pairs[i];
		
		assert(pair.contact);

		const EpxState &stateA = states[pair.rigidBodyA];
		const EpxState &stateB = states[pair.rigidBodyB];
		const EpxCollidable &collA = collidables[pair.rigidBodyA];
		const EpxCollidable &collB = collidables[pair.rigidBodyB];
		
		EpxTransform3 transformA(stateA.m_orientation,stateA.m_position);
		EpxTransform3 transformB(stateB.m_orientation,stateB.m_position);
		
		for(EpxUInt32 j=0;j<collA.m_numShapes;j++) {
			const EpxShape &shapeA = collA.m_shapes[j];
			EpxTransform3 offsetTransformA(shapeA.m_offsetOrientation,shapeA.m_offsetPosition);
			EpxTransform3 worldTransformA = transformA * offsetTransformA;
			
			for(EpxUInt32 k=0;k<collB.m_numShapes;k++) {
				const EpxShape &shapeB = collB.m_shapes[k];
				EpxTransform3 offsetTransformB(shapeB.m_offsetOrientation,shapeB.m_offsetPosition);
				EpxTransform3 worldTransformB = transformB * offsetTransformB;
				
				EpxVector3 contactPointA;
				EpxVector3 contactPointB;
				EpxVector3 normal;
				EpxFloat penetrationDepth;
				
				if(epxConvexConvexContact(
					shapeA.m_geometry,worldTransformA,
					shapeB.m_geometry,worldTransformB,
					normal,penetrationDepth,
					contactPointA,contactPointB) && penetrationDepth < 0.0f) {
					
					// 衝突点を剛体の座標系へ変換し、コンタクトへ追加する。
					pair.contact->addContact(
						penetrationDepth,normal,
						offsetTransformA.getTranslation() + offsetTransformA.getUpper3x3() * contactPointA,
						offsetTransformB.getTranslation() + offsetTransformB.getUpper3x3() * contactPointB);
				}
			}
		}
	}
}

} // namespace EasyPhysics
