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
class q3Body;
struct q3Box;
struct q3Transform;
struct q3AABB;

struct q3ContactPair {
  int A;
  int B;
};

class q3BroadPhase {
  std::vector<q3ContactPair> m_pairBuffer;
  std::vector<int> m_moveBuffer;

  int m_currentIndex = -1;

  using Payload = std::tuple<q3Body *, q3Box *>;
  q3DynamicAABBTree<Payload> m_tree;

public:
  q3BroadPhase();
  ~q3BroadPhase();

  void InsertBox(q3Body *body, q3Box *shape, const q3AABB &aabb);
  void RemoveBox(const q3Box *shape);
  void RemoveBody(q3Body *body);

  // Generates the contact list. All previous contacts are returned to the
  // allocator before generation occurs.
  void
  UpdatePairs(const std::function<void(q3Body *bodyA, q3Box *A, q3Body *bodyB,
                                       q3Box *B)> &addContact);

  void Update(int id, const q3AABB &aabb);

  bool TestOverlap(int A, int B) const;
  void SynchronizeProxies(q3Body *body);

  // Query the world to find any shapes that can potentially intersect
  // the provided AABB. This works by querying the broadphase with an
  // AAABB -- only *potential* intersections are reported. Perhaps the
  // user might use lmDistance as fine-grained collision detection.
  void QueryAABB(const std::function<bool(q3Body *body, q3Box *box)> &cb,
                 const q3AABB &aabb) const;
  // Query the world to find any shapes intersecting a world space point.
  void QueryPoint(const std::function<bool(q3Body *body, q3Box *box)> &cb,
                  const q3Vec3 &point) const;
  // Query the world to find any shapes intersecting a ray.
  void RayCast(const std::function<bool(q3Body *body, q3Box *box)> &cb,
               q3RaycastData &rayCast) const;

  void Render(q3Render *renderer) const { m_tree.Render(renderer); }

private:
  void BufferMove(int id);
  bool TreeCallBack(int index);
};

#endif // Q3BROADPHASE_H
