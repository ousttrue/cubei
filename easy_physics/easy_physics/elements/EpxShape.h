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

#ifndef EPX_SHAPE_H
#define EPX_SHAPE_H

#include "../EpxBase.h"
#include "EpxConvexMesh.h"

namespace EasyPhysics {

/// 形状
struct EpxShape {
  EpxConvexMesh m_geometry = {};                     ///< 凸メッシュ
  EpxVector3 m_offsetPosition = {0, 0, 0};           ///< オフセット位置
  EpxQuat m_offsetOrientation = EpxQuat::identity(); ///< オフセット姿勢
  void *userData = nullptr;                          ///< ユーザーデータ

  EpxMatrix4 WorldShapeMatrix(const EpxTransform3 &rigidBodyTransform) const {
    EasyPhysics::EpxTransform3 shapeTransform(m_offsetOrientation,
                                              m_offsetPosition);
    EasyPhysics::EpxTransform3 worldTransform =
        rigidBodyTransform * shapeTransform;
    return EpxMatrix4(worldTransform) * EpxMatrix4::scale(m_geometry.m_scale * 2);
  }
};

} // namespace EasyPhysics

#endif // EPX_SHAPE_H
