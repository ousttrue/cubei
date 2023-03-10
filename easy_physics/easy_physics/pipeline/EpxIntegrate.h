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

#ifndef EPX_INTEGRATE_H
#define EPX_INTEGRATE_H

#include "../EpxBase.h"
#include "../elements/EpxState.h"
#include "../elements/EpxRigidBody.h"

#define EPX_MAX_LINEAR_VELOCITY		340.0f
#define EPX_MAX_ANGULAR_VELOCITY	(EPX_PI * 60.0f)

namespace EasyPhysics {

/// 剛体に外力を与える
/// @param state 剛体の状態
/// @param body 剛体の属性
/// @param externalForce 与えるフォース
/// @param externalTorque 与えるトルク
/// @param timeStep タイムステップ
void epxApplyExternalForce(
	EpxState &state,
	const EpxRigidBody &body,
	const EpxVector3 &externalForce,
	const EpxVector3 &externalTorque,
	EpxFloat timeStep);

/// インテグレート
/// @param states 剛体の状態の配列
/// @param numRigidBodies 剛体の数
/// @param timeStep タイムステップ
void epxIntegrate(
	EpxState *statse,
	EpxUInt32 numRigidBodies,
	EpxFloat timeStep);

} // namespace EasyPhysics

#endif // EPX_INTEGRATE_H
