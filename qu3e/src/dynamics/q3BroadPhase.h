//--------------------------------------------------------------------------------------------------
/**
@file	q3BroadPhase.h

@author	Randy Gaul
@date	10/10/2014

        Copyright (c) 2014 Randy Gaul http://www.randygaul.net

        This software is provided 'as-is', without any express or implied
        warranty. In no event will the authors be held liable for any damages
        arising from the use of this software.

        Permission is granted to anyone to use this software for any purpose,
        including commercial applications, and to alter it and redistribute it
        freely, subject to the following restrictions:
          1. The origin of this software must not be misrepresented; you must
not claim that you wrote the original software. If you use this software in a
product, an acknowledgment in the product documentation would be appreciated but
is not required.
          2. Altered source versions must be plainly marked as such, and must
not be misrepresented as being the original software.
          3. This notice may not be removed or altered from any source
distribution.
*/
//--------------------------------------------------------------------------------------------------

#ifndef Q3BROADPHASE_H
#define Q3BROADPHASE_H


#include "q3DynamicAABBTree.h"
#include <vector>

//--------------------------------------------------------------------------------------------------
// q3BroadPhase
//--------------------------------------------------------------------------------------------------
class q3ContactManager;
struct q3Box;
struct q3Transform;
struct q3AABB;

struct q3ContactPair {
  int A;
  int B;
};

class q3BroadPhase {
public:
  q3BroadPhase(q3ContactManager *manager);
  ~q3BroadPhase();

  void InsertBox(q3Box *shape, const q3AABB &aabb);
  void RemoveBox(const q3Box *shape);

  // Generates the contact list. All previous contacts are returned to the
  // allocator before generation occurs.
  void UpdatePairs(void);

  void Update(int id, const q3AABB &aabb);

  bool TestOverlap(int A, int B) const;
  void SynchronizeProxies(class q3Body *body);

  q3DynamicAABBTree m_tree;

private:
  q3ContactManager *m_manager;

  std::vector<q3ContactPair> m_pairBuffer;
  std::vector<int> m_moveBuffer;

  int m_currentIndex;

  void BufferMove(int id);
  bool TreeCallBack(int index);

  friend class q3DynamicAABBTree;
};

inline bool q3BroadPhase::TreeCallBack(int index) {
  // Cannot collide with self
  if (index == m_currentIndex)
    return true;

  // if (m_pairCount == m_pairCapacity) {
  //   q3ContactPair *oldBuffer = m_pairBuffer;
  //   m_pairCapacity *= 2;
  //   m_pairBuffer =
  //       (q3ContactPair *)q3Alloc(m_pairCapacity * sizeof(q3ContactPair));
  //   memcpy(m_pairBuffer, oldBuffer, m_pairCount * sizeof(q3ContactPair));
  //   q3Free(oldBuffer);
  // }

  int iA = q3Min(index, m_currentIndex);
  int iB = q3Max(index, m_currentIndex);

  m_pairBuffer.push_back({
      .A = iA,
      .B = iB,
  });

  return true;
}

#endif // Q3BROADPHASE_H