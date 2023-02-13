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

#include "EpxBroadphase.h"
#include "EpxSort.h"

#define EPX_AABB_EXPAND 0.01f

namespace EasyPhysics {

static inline EpxBool epxIntersectAABB(const EpxVector3 &centerA,
                                       const EpxVector3 &halfA,
                                       const EpxVector3 &centerB,
                                       const EpxVector3 &halfB) {
  if (fabs(centerA[0] - centerB[0]) > halfA[0] + halfB[0])
    return false;
  if (fabs(centerA[1] - centerB[1]) > halfA[1] + halfB[1])
    return false;
  if (fabs(centerA[2] - centerB[2]) > halfA[2] + halfB[2])
    return false;
  return true;
}

uint32_t epxBroadPhase(std::span<const EpxState> states,
                       std::span<const EpxCollidable> collidables,
                       std::span<const EpxPair> oldPairs, EpxPair *newPairs,
                       const EpxUInt32 maxPairs, EpxAllocator *allocator,
                       void *userData, epxBroadPhaseCallback callback) {
  assert(states.size() == collidables.size());
  assert(newPairs);
  assert(allocator);

  uint32_t numNewPairs = 0;

  // AABB交差ペアを見つける（総当たり）
  // 処理の内容を明確にするため、ここでは空間分割テクニックを使っていませんが、
  // 理論編で解説されているSweet and prune等の手法を使えば高速化できます。
  for (EpxUInt32 i = 0; i < states.size(); i++) {
    for (EpxUInt32 j = i + 1; j < states.size(); j++) {
      const EpxState &stateA = states[i];
      const EpxCollidable &collidableA = collidables[i];
      const EpxState &stateB = states[j];
      const EpxCollidable &collidableB = collidables[j];

      if (callback && !callback(i, j, userData)) {
        continue;
      }

      EpxMatrix3 orientationA(stateA.m_orientation);
      EpxVector3 centerA =
          stateA.m_position + orientationA * collidableA.m_center;
      EpxVector3 halfA = absPerElem(orientationA) *
                         (collidableA.m_half +
                          EpxVector3(EPX_AABB_EXPAND)); // AABBサイズを若干拡張

      EpxMatrix3 orientationB(stateB.m_orientation);
      EpxVector3 centerB =
          stateB.m_position + orientationB * collidableB.m_center;
      EpxVector3 halfB = absPerElem(orientationB) *
                         (collidableB.m_half +
                          EpxVector3(EPX_AABB_EXPAND)); // AABBサイズを若干拡張

      if (epxIntersectAABB(centerA, halfA, centerB, halfB) &&
          numNewPairs < maxPairs) {
        EpxPair &newPair = newPairs[numNewPairs++];

        newPair.rigidBodyA = i < j ? i : j;
        newPair.rigidBodyB = i < j ? j : i;
        newPair.contact = NULL;
      }
    }
  }

  // ソート
  {
    EpxPair *sortBuff =
        (EpxPair *)allocator->allocate(sizeof(EpxPair) * numNewPairs);
    epxSort<EpxPair>(std::span<EpxPair>(newPairs, numNewPairs),
                     {sortBuff, numNewPairs});
    allocator->deallocate(sortBuff);
  }

  // 新しく検出したペアと過去のペアを比較
  EpxPair *outNewPairs =
      (EpxPair *)allocator->allocate(sizeof(EpxPair) * numNewPairs);
  EpxPair *outKeepPairs =
      (EpxPair *)allocator->allocate(sizeof(EpxPair) * oldPairs.size());
  assert(outNewPairs);
  assert(outKeepPairs);

  EpxUInt32 nNew = 0;
  EpxUInt32 nKeep = 0;

  EpxUInt32 oldId = 0, newId = 0;

  while (oldId < oldPairs.size() && newId < numNewPairs) {
    if (newPairs[newId].key > oldPairs[oldId].key) {
      // remove
      allocator->deallocate(oldPairs[oldId].contact);
      oldId++;
    } else if (newPairs[newId].key == oldPairs[oldId].key) {
      // keep
      assert(nKeep <= oldPairs.size());
      outKeepPairs[nKeep] = oldPairs[oldId];
      nKeep++;
      oldId++;
      newId++;
    } else {
      // new
      assert(nNew <= numNewPairs);
      outNewPairs[nNew] = newPairs[newId];
      nNew++;
      newId++;
    }
  };

  if (newId < numNewPairs) {
    // all new
    for (; newId < numNewPairs; newId++, nNew++) {
      assert(nNew <= numNewPairs);
      outNewPairs[nNew] = newPairs[newId];
    }
  } else if (oldId < oldPairs.size()) {
    // all remove
    for (; oldId < oldPairs.size(); oldId++) {
      allocator->deallocate(oldPairs[oldId].contact);
    }
  }

  for (EpxUInt32 i = 0; i < nNew; i++) {
    outNewPairs[i].contact =
        (EpxContact *)allocator->allocate(sizeof(EpxContact));
    outNewPairs[i].contact->reset();
  }

  for (EpxUInt32 i = 0; i < nKeep; i++) {
    outKeepPairs[i].contact->refresh(
        states[outKeepPairs[i].rigidBodyA].m_position,
        states[outKeepPairs[i].rigidBodyA].m_orientation,
        states[outKeepPairs[i].rigidBodyB].m_position,
        states[outKeepPairs[i].rigidBodyB].m_orientation);
  }

  numNewPairs = 0;
  for (EpxUInt32 i = 0; i < nKeep; i++) {
    outKeepPairs[i].type = EpxPairTypeKeep;
    newPairs[numNewPairs++] = outKeepPairs[i];
  }
  for (EpxUInt32 i = 0; i < nNew; i++) {
    outNewPairs[i].type = EpxPairTypeNew;
    newPairs[numNewPairs++] = outNewPairs[i];
  }

  allocator->deallocate(outKeepPairs);
  allocator->deallocate(outNewPairs);

  // ソート
  {
    EpxPair *sortBuff =
        (EpxPair *)allocator->allocate(sizeof(EpxPair) * numNewPairs);
    epxSort<EpxPair>(std::span<EpxPair>(newPairs, numNewPairs),
                     {sortBuff, numNewPairs});
    allocator->deallocate(sortBuff);
  }

  return numNewPairs;
}

} // namespace EasyPhysics
