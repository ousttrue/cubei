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

        2. Altered source versions must be plainly marked as such, and must not
   be misrepresented as being the original software.

        3. This notice may not be removed or altered from any source
   distribution.
*/

#ifndef EPX_BROADPHASE_H
#define EPX_BROADPHASE_H

#include "../EpxBase.h"
#include "../elements/EpxCollidable.h"
#include "../elements/EpxPair.h"
#include "../elements/EpxState.h"
#include "EpxAllocator.h"
#include <span>

#define EPX_MAX_LINEAR_VELOCITY 340.0f
#define EPX_MAX_ANGULAR_VELOCITY (EPX_PI * 60.0f)

namespace EasyPhysics {

/// ブロードフェーズコールバック<br>
/// epxBroadPhase()の引数として渡すと、AABB交差判定前に呼ばれる。
/// @param rigidBodyIdA 剛体Aのインデックス
/// @param rigidBodyIdB 剛体Bのインデックス
/// @param userData ユーザーデータ
/// return true:判定を続行 , false:判定をキャンセル
typedef EpxBool (*epxBroadPhaseCallback)(EpxUInt32 rigidBodyIdA,
                                         EpxUInt32 rigidBodyIdB,
                                         void *userData);

/// ブロードフェーズ
/// @param states 剛体の状態の配列
/// @param collidables 剛体の形状の配列
/// @param numRigidBodies 剛体の数
/// @param oldPairs 前フレームのペア配列
/// @param numOldPairs 前フレームのペア数
/// @param[out] newPairs 検出されたペア配列
/// @param[out] numNewPairs 検出されたペア数
/// @param maxPairs 検出ペアの最大数
/// @param allocator アロケータ
/// @param userData コールバック呼び出し時に渡されるユーザーデータ
/// @param callback コールバック
void epxBroadPhase(std::span<const EpxState> states,
                   std::span<const EpxCollidable> collidables,
                   std::span<const EpxPair> oldPairs,
                   EpxPair *newPairs, EpxUInt32 &numNewPairs,
                   const EpxUInt32 maxPairs, EpxAllocator *allocator,
                   void *userData, epxBroadPhaseCallback callback = NULL);

} // namespace EasyPhysics

#endif // EPX_BROADPHASE_H
