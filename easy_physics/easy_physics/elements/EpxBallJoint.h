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

#ifndef EPX_BALL_JOINT_H
#define EPX_BALL_JOINT_H

#include "../EpxBase.h"
#include "../elements/EpxConstraint.h"

namespace EasyPhysics {

/// ボールジョイント
struct EpxBallJoint {
  EpxFloat bias;        ///< 拘束の強さの調整値
  EpxUInt32 rigidBodyA; ///< 剛体Aへのインデックス
  EpxUInt32 rigidBodyB; ///< 剛体Bへのインデックス
  EpxVector3 anchorA; ///< 剛体Aのローカル座標系における接続点
  EpxVector3 anchorB; ///< 剛体Bのローカル座標系における接続点
  EpxConstraint constraint; ///< 拘束

  /// 初期化
  void reset() {
    bias = 0.1f;
    constraint.accumImpulse = 0.0f;
  }
};

} // namespace EasyPhysics

#endif // EPX_BALL_JOINT_H
