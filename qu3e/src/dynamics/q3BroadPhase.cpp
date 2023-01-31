
/**
@file	q3BroadPhase.cpp

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

#include "q3BroadPhase.h"
#include "../scene/q3Body.h"
#include "../scene/q3Box.h"
#include <Remotery.h>
#include <algorithm>

q3BroadPhase::q3BroadPhase() {}

q3BroadPhase::~q3BroadPhase() {}

void q3BroadPhase::InsertBox(q3Body *body, q3Box *box, const q3AABB &aabb) {
  int id = m_tree.Insert(aabb, {body, box});
  box->SetBroadPhaseIndex(id);
  BufferMove(id);
}

void q3BroadPhase::RemoveBox(const q3Box *box) {
  m_tree.Remove(box->BroadPhaseIndex());
}

void q3BroadPhase::RemoveBody(q3Body *body) {
  for (auto box : *body) {
    RemoveBox(box);
  }
}

inline bool ContactPairSort(const q3ContactPair &lhs,
                            const q3ContactPair &rhs) {
  if (lhs.A < rhs.A)
    return true;

  if (lhs.A == rhs.A)
    return lhs.B < rhs.B;

  return false;
}

void q3BroadPhase::UpdatePairs(
    const std::function<void(q3Body *bodyA, q3Box *A, q3Body *bodyB, q3Box *B)>
        &addContact) {
  rmt_ScopedCPUSample(q3BroadPhaseUpdatePairs, 0);

  m_pairBuffer.clear();

  // Query the tree with all moving boxs
  for (int i = 0; i < m_moveBuffer.size(); ++i) {
    m_currentIndex = m_moveBuffer[i];
    q3AABB aabb = m_tree.GetFatAABB(m_currentIndex);

    // @TODO: Use a static and non-static tree and query one against the other.
    //        This will potentially prevent (gotta think about this more) time
    //        wasted with queries of static bodies against static bodies, and
    //        kinematic to kinematic.
    m_tree.QueryAABB(
        std::bind(&q3BroadPhase::TreeCallBack, this, std::placeholders::_1),
        aabb);
  }

  // Reset the move buffer
  m_moveBuffer.clear();

  // Sort pairs to expose duplicates
  std::sort(m_pairBuffer.begin(), m_pairBuffer.end(), ContactPairSort);

  // Queue manifolds for solving
  {
    int i = 0;
    while (i < m_pairBuffer.size()) {
      // Add contact to manager
      q3ContactPair *pair = &m_pairBuffer[i];
      auto [bodyA, A] = m_tree.GetUserData(pair->A);
      auto [bodyB, B] = m_tree.GetUserData(pair->B);
      addContact(bodyA, A, bodyB, B);

      ++i;

      // Skip duplicate pairs by iterating i until we find a unique pair
      while (i < m_pairBuffer.size()) {
        q3ContactPair *potentialDup = &m_pairBuffer[i];

        if (pair->A != potentialDup->A || pair->B != potentialDup->B)
          break;

        ++i;
      }
    }
  }

  m_tree.Validate();
}

void q3BroadPhase::Update(int id, const q3AABB &aabb) {
  if (m_tree.Update(id, aabb))
    BufferMove(id);
}

bool q3BroadPhase::TestOverlap(int A, int B) const {
  return m_tree.GetFatAABB(A).IsOverlapped(m_tree.GetFatAABB(B));
}

void q3BroadPhase::SynchronizeProxies(q3Body *body) {
  auto m_tx = body->UpdatePosition();

  for (auto box : *body) {
    Update(box->BroadPhaseIndex(), box->ComputeAABB(m_tx));
  }
}

void q3BroadPhase::BufferMove(int id) { m_moveBuffer.push_back(id); }

bool q3BroadPhase::TreeCallBack(int index) {
  // Cannot collide with self
  if (index == m_currentIndex)
    return true;

  int iA = std::min(index, m_currentIndex);
  int iB = std::max(index, m_currentIndex);

  m_pairBuffer.push_back({
      .A = iA,
      .B = iB,
  });

  return true;
}

void q3BroadPhase::QueryAABB(
    const std::function<bool(q3Body *body, q3Box *box)> &cb,
    const q3AABB &aabb) const {
  m_tree.QueryAABB(
      [broadPhase = this, cb, m_aabb = aabb](int id) {
        auto [body, box] = broadPhase->m_tree.GetUserData(id);
        if (m_aabb.IsOverlapped(box->ComputeAABB(body->Transform()))) {
          return cb(body, box);
        }
        return true;
      },
      aabb);
}

void q3BroadPhase::QueryPoint(
    const std::function<bool(q3Body *body, q3Box *box)> &cb,
    const q3Vec3 &point) const {
  const float k_fattener = float(0.5);
  q3Vec3 v{k_fattener, k_fattener, k_fattener};
  q3AABB aabb;
  aabb.min = point - v;
  aabb.max = point + v;
  m_tree.QueryAABB(
      [broadPhase = this, m_point = point, cb](int id) {
        auto [body, box] = broadPhase->m_tree.GetUserData(id);
        if (box->TestPoint(body->Transform(), m_point)) {
          cb(body, box);
        }
        return true;
      },
      aabb);
}

void q3BroadPhase::RayCast(
    const std::function<bool(q3Body *body, q3Box *box)> &cb,
    q3RaycastData &rayCast) const {
  m_tree.QueryRay(
      [m_rayCast = &rayCast, broadPhase = this, cb](int id) {
        auto [body, box] = broadPhase->m_tree.GetUserData(id);
        if (box->Raycast(body->Transform(), m_rayCast)) {
          return cb(body, box);
        }
        return true;
      },
      rayCast);
}
