//--------------------------------------------------------------------------------------------------
/**
@file	q3DynamicAABBTree.h

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

#pragma once
#include "../math/q3Geometry.h"
#include "../debug/q3Render.h"
#include "../math/q3Math.h"
#include <vector>

//--------------------------------------------------------------------------------------------------
// q3DynamicAABBTree
//--------------------------------------------------------------------------------------------------
// Resources:
// http://box2d.org/2014/08/balancing-dynamic-trees/
// http://www.randygaul.net/2013/08/06/dynamic-aabb-tree/
inline void FattenAABB(q3AABB &aabb) {
  const float k_fattener = float(0.5);
  q3Vec3 v{k_fattener, k_fattener, k_fattener};

  aabb.min -= v;
  aabb.max += v;
}

class q3DynamicAABBTree {
  struct Node {
    bool IsLeaf(void) const {
      // The right leaf does not use the same memory as the userdata,
      // and will always be Null (no children)
      return right == Null;
    }

    // Fat AABB for leafs, bounding AABB for branches
    q3AABB aabb;

    union {
      int parent;
      int next; // free list
    };

    // Child indices
    struct {
      int left;
      int right;
    };

    // Since only leaf nodes hold userdata, we can use the
    // same memory used for left/right indices to store
    // the userdata void pointer
    void *userData;

    // leaf = 0, free nodes = -1
    int height;

    static const int Null = -1;
  };

  int m_root;
  std::vector<Node> m_nodes;
  int m_count; // Number of active nodes
  int m_freeList;

public:
  q3DynamicAABBTree() {
    m_root = Node::Null;
    m_nodes.resize(1024);
    m_count = 0;
    AddToFreeList(0);
  }
  ~q3DynamicAABBTree() {}

  // Provide tight-AABB
  int Insert(const q3AABB &aabb, void *userData) {
    int id = AllocateNode();

    // Fatten AABB and set height/userdata
    m_nodes[id].aabb = aabb;
    FattenAABB(m_nodes[id].aabb);
    m_nodes[id].userData = userData;
    m_nodes[id].height = 0;

    InsertLeaf(id);

    return id;
  }

  void Remove(int id) {
    assert(id >= 0 && id < m_nodes.size());
    assert(m_nodes[id].IsLeaf());
    RemoveLeaf(id);
    DeallocateNode(id);
  }

  bool Update(int id, const q3AABB &aabb) {
    assert(id >= 0 && id < m_nodes.size());
    assert(m_nodes[id].IsLeaf());

    if (m_nodes[id].aabb.Contains(aabb))
      return false;

    RemoveLeaf(id);

    m_nodes[id].aabb = aabb;
    FattenAABB(m_nodes[id].aabb);

    InsertLeaf(id);

    return true;
  }

  void *GetUserData(int id) const {
    assert(id >= 0 && id < m_nodes.size());
    return m_nodes[id].userData;
  }

  const q3AABB &GetFatAABB(int id) const {
    assert(id >= 0 && id < m_nodes.size());
    return m_nodes[id].aabb;
  }

  void Render(q3Render *render) const {
    if (m_root != Node::Null) {
      render->SetPenColor(0.5f, 0.5f, 1.0f);
      RenderNode(render, m_root);
    }
  }

  template <typename T> void Query(T *cb, const q3AABB &aabb) const;
  template <typename T> void Query(T *cb, q3RaycastData &rayCast) const;

  // For testing
  void Validate() const {
    // Verify free list
    int freeNodes = 0;
    int index = m_freeList;

    while (index != Node::Null) {
      assert(index >= 0 && index < m_nodes.size());
      index = m_nodes[index].next;
      ++freeNodes;
    }

    assert(m_count + freeNodes == m_nodes.size());

    // Validate tree structure
    if (m_root != Node::Null) {
      assert(m_nodes[m_root].parent == Node::Null);

#ifdef _DEBUG
      ValidateStructure(m_root);
#endif
    }
  }

private:
  int AllocateNode() {
    if (m_freeList == Node::Null) {
      m_nodes.resize(m_nodes.size()*2);
      AddToFreeList(m_count);
    }

    int freeNode = m_freeList;
    m_freeList = m_nodes[m_freeList].next;
    m_nodes[freeNode].height = 0;
    m_nodes[freeNode].left = Node::Null;
    m_nodes[freeNode].right = Node::Null;
    m_nodes[freeNode].parent = Node::Null;
    m_nodes[freeNode].userData = NULL;
    ++m_count;
    return freeNode;
  }

  inline void DeallocateNode(int index);

  int Balance(int iA) {
    Node *A = &m_nodes[iA];

    if (A->IsLeaf() || A->height == 1)
      return iA;

    /*      A
          /   \
         B     C
        / \   / \
       D   E F   G
    */

    int iB = A->left;
    int iC = A->right;
    Node *B = &m_nodes[iB];
    Node *C = &m_nodes[iC];

    int balance = C->height - B->height;

    // C is higher, promote C
    if (balance > 1) {
      int iF = C->left;
      int iG = C->right;
      Node *F = &m_nodes[iF];
      Node *G = &m_nodes[iG];

      // grandParent point to C
      if (A->parent != Node::Null) {
        if (m_nodes[A->parent].left == iA)
          m_nodes[A->parent].left = iC;

        else
          m_nodes[A->parent].right = iC;
      } else
        m_root = iC;

      // Swap A and C
      C->left = iA;
      C->parent = A->parent;
      A->parent = iC;

      // Finish rotation
      if (F->height > G->height) {
        C->right = iF;
        A->right = iG;
        G->parent = iA;
        A->aabb = q3Combine(B->aabb, G->aabb);
        C->aabb = q3Combine(A->aabb, F->aabb);

        A->height = 1 + q3Max(B->height, G->height);
        C->height = 1 + q3Max(A->height, F->height);
      }

      else {
        C->right = iG;
        A->right = iF;
        F->parent = iA;
        A->aabb = q3Combine(B->aabb, F->aabb);
        C->aabb = q3Combine(A->aabb, G->aabb);

        A->height = 1 + q3Max(B->height, F->height);
        C->height = 1 + q3Max(A->height, G->height);
      }

      return iC;
    }

    // B is higher, promote B
    else if (balance < -1) {
      int iD = B->left;
      int iE = B->right;
      Node *D = &m_nodes[iD];
      Node *E = &m_nodes[iE];

      // grandParent point to B
      if (A->parent != Node::Null) {
        if (m_nodes[A->parent].left == iA)
          m_nodes[A->parent].left = iB;
        else
          m_nodes[A->parent].right = iB;
      }

      else
        m_root = iB;

      // Swap A and B
      B->right = iA;
      B->parent = A->parent;
      A->parent = iB;

      // Finish rotation
      if (D->height > E->height) {
        B->left = iD;
        A->left = iE;
        E->parent = iA;
        A->aabb = q3Combine(C->aabb, E->aabb);
        B->aabb = q3Combine(A->aabb, D->aabb);

        A->height = 1 + q3Max(C->height, E->height);
        B->height = 1 + q3Max(A->height, D->height);
      }

      else {
        B->left = iE;
        A->left = iD;
        D->parent = iA;
        A->aabb = q3Combine(C->aabb, D->aabb);
        B->aabb = q3Combine(A->aabb, E->aabb);

        A->height = 1 + q3Max(C->height, D->height);
        B->height = 1 + q3Max(A->height, E->height);
      }

      return iB;
    }

    return iA;
  }

  void InsertLeaf(int id) {
    if (m_root == Node::Null) {
      m_root = id;
      m_nodes[m_root].parent = Node::Null;
      return;
    }

    // Search for sibling
    int searchIndex = m_root;
    q3AABB leafAABB = m_nodes[id].aabb;
    while (!m_nodes[searchIndex].IsLeaf()) {
      // Cost for insertion at index (branch node), involves creation
      // of new branch to contain this index and the new leaf
      q3AABB combined = q3Combine(leafAABB, m_nodes[searchIndex].aabb);
      float combinedArea = combined.SurfaceArea();
      float branchCost = float(2.0) * combinedArea;

      // Inherited cost (surface area growth from heirarchy update after
      // descent)
      float inheritedCost =
          float(2.0) * (combinedArea - m_nodes[searchIndex].aabb.SurfaceArea());

      int left = m_nodes[searchIndex].left;
      int right = m_nodes[searchIndex].right;

      // Calculate costs for left/right descents. If traversal is to a leaf,
      // then the cost of the combind AABB represents a new branch node.
      // Otherwise the cost is only the inflation of the pre-existing branch.
      float leftDescentCost;
      if (m_nodes[left].IsLeaf())
        leftDescentCost =
            q3Combine(leafAABB, m_nodes[left].aabb).SurfaceArea() +
            inheritedCost;
      else {
        float inflated = q3Combine(leafAABB, m_nodes[left].aabb).SurfaceArea();
        float branchArea = m_nodes[left].aabb.SurfaceArea();
        leftDescentCost = inflated - branchArea + inheritedCost;
      }

      // Cost for right descent
      float rightDescentCost;
      if (m_nodes[right].IsLeaf())
        rightDescentCost =
            q3Combine(leafAABB, m_nodes[right].aabb).SurfaceArea() +
            inheritedCost;
      else {
        float inflated = q3Combine(leafAABB, m_nodes[right].aabb).SurfaceArea();
        float branchArea = m_nodes[right].aabb.SurfaceArea();
        rightDescentCost = inflated - branchArea + inheritedCost;
      }

      // Determine traversal direction, or early out on a branch index
      if (branchCost < leftDescentCost && branchCost < rightDescentCost)
        break;

      if (leftDescentCost < rightDescentCost)
        searchIndex = left;

      else
        searchIndex = right;
    }

    int sibling = searchIndex;

    // Create new parent
    int oldParent = m_nodes[sibling].parent;
    int newParent = AllocateNode();
    m_nodes[newParent].parent = oldParent;
    m_nodes[newParent].userData = NULL;
    m_nodes[newParent].aabb = q3Combine(leafAABB, m_nodes[sibling].aabb);
    m_nodes[newParent].height = m_nodes[sibling].height + 1;

    // Sibling was root
    if (oldParent == Node::Null) {
      m_nodes[newParent].left = sibling;
      m_nodes[newParent].right = id;
      m_nodes[sibling].parent = newParent;
      m_nodes[id].parent = newParent;
      m_root = newParent;
    }

    else {
      if (m_nodes[oldParent].left == sibling)
        m_nodes[oldParent].left = newParent;

      else
        m_nodes[oldParent].right = newParent;

      m_nodes[newParent].left = sibling;
      m_nodes[newParent].right = id;
      m_nodes[sibling].parent = newParent;
      m_nodes[id].parent = newParent;
    }

    SyncHeirarchy(m_nodes[id].parent);
  }

  void RemoveLeaf(int id) {
    if (id == m_root) {
      m_root = Node::Null;
      return;
    }

    // Setup parent, grandParent and sibling
    int parent = m_nodes[id].parent;
    int grandParent = m_nodes[parent].parent;
    int sibling;

    if (m_nodes[parent].left == id)
      sibling = m_nodes[parent].right;

    else
      sibling = m_nodes[parent].left;

    // Remove parent and replace with sibling
    if (grandParent != Node::Null) {
      // Connect grandParent to sibling
      if (m_nodes[grandParent].left == parent)
        m_nodes[grandParent].left = sibling;

      else
        m_nodes[grandParent].right = sibling;

      // Connect sibling to grandParent
      m_nodes[sibling].parent = grandParent;
    }

    // Parent was root
    else {
      m_root = sibling;
      m_nodes[sibling].parent = Node::Null;
    }

    DeallocateNode(parent);
    SyncHeirarchy(grandParent);
  }

  void ValidateStructure(int index) const {
    auto n = &m_nodes[index];

    int il = n->left;
    int ir = n->right;

    if (n->IsLeaf()) {
      assert(ir == Node::Null);
      assert(n->height == 0);
      return;
    }

    assert(il >= 0 && il < m_nodes.size());
    assert(ir >= 0 && ir < m_nodes.size());
    auto l = &m_nodes[il];
    auto r = &m_nodes[ir];

    assert(l->parent == index);
    assert(r->parent == index);

    ValidateStructure(il);
    ValidateStructure(ir);
  }

  void RenderNode(q3Render *render, int index) const {
    assert(index >= 0 && index < m_nodes.size());

    auto n = &m_nodes[index];
    const q3AABB &b = n->aabb;

    render->SetPenPosition(b.min.x, b.max.y, b.min.z);

    render->Line(b.min.x, b.max.y, b.max.z);
    render->Line(b.max.x, b.max.y, b.max.z);
    render->Line(b.max.x, b.max.y, b.min.z);
    render->Line(b.min.x, b.max.y, b.min.z);

    render->SetPenPosition(b.min.x, b.min.y, b.min.z);

    render->Line(b.min.x, b.min.y, b.max.z);
    render->Line(b.max.x, b.min.y, b.max.z);
    render->Line(b.max.x, b.min.y, b.min.z);
    render->Line(b.min.x, b.min.y, b.min.z);

    render->SetPenPosition(b.min.x, b.min.y, b.min.z);
    render->Line(b.min.x, b.max.y, b.min.z);
    render->SetPenPosition(b.max.x, b.min.y, b.min.z);
    render->Line(b.max.x, b.max.y, b.min.z);
    render->SetPenPosition(b.max.x, b.min.y, b.max.z);
    render->Line(b.max.x, b.max.y, b.max.z);
    render->SetPenPosition(b.min.x, b.min.y, b.max.z);
    render->Line(b.min.x, b.max.y, b.max.z);

    if (!n->IsLeaf()) {
      RenderNode(render, n->left);
      RenderNode(render, n->right);
    }
  }

  // Correct AABB hierarchy heights and AABBs starting at supplied
  // index traversing up the heirarchy
  void SyncHeirarchy(int index) {
    while (index != Node::Null) {
      index = Balance(index);
      int left = m_nodes[index].left;
      int right = m_nodes[index].right;
      m_nodes[index].height =
          1 + q3Max(m_nodes[left].height, m_nodes[right].height);
      m_nodes[index].aabb = q3Combine(m_nodes[left].aabb, m_nodes[right].aabb);
      index = m_nodes[index].parent;
    }
  }

  // Insert nodes at a given index until m_nodes.size() into the free list
  void AddToFreeList(int index) {
    for (int i = index; i < m_nodes.size() - 1; ++i) {
      m_nodes[i].next = i + 1;
      m_nodes[i].height = Node::Null;
    }
    m_nodes[m_nodes.size() - 1].next = Node::Null;
    m_nodes[m_nodes.size() - 1].height = Node::Null;
    m_freeList = index;
  }
};

//--------------------------------------------------------------------------------------------------
inline void q3DynamicAABBTree::DeallocateNode(int index) {
  assert(index >= 0 && index < m_nodes.size());

  m_nodes[index].next = m_freeList;
  m_nodes[index].height = Node::Null;
  m_freeList = index;

  --m_count;
}

//--------------------------------------------------------------------------------------------------
template <typename T>
inline void q3DynamicAABBTree::Query(T *cb, const q3AABB &aabb) const {
  const int k_stackCapacity = 256;
  int stack[k_stackCapacity];
  int sp = 1;

  *stack = m_root;

  while (sp) {
    // k_stackCapacity too small
    assert(sp < k_stackCapacity);

    int id = stack[--sp];

    const Node *n = &m_nodes[id];
    if (q3AABBtoAABB(aabb, n->aabb)) {
      if (n->IsLeaf()) {
        if (!cb->TreeCallBack(id))
          return;
      } else {
        stack[sp++] = n->left;
        stack[sp++] = n->right;
      }
    }
  }
}

//--------------------------------------------------------------------------------------------------
template <typename T>
void q3DynamicAABBTree::Query(T *cb, q3RaycastData &rayCast) const {
  const float k_epsilon = float(1.0e-6);
  const int k_stackCapacity = 256;
  int stack[k_stackCapacity];
  int sp = 1;

  *stack = m_root;

  q3Vec3 p0 = rayCast.start;
  q3Vec3 p1 = p0 + rayCast.dir * rayCast.t;

  while (sp) {
    // k_stackCapacity too small
    assert(sp < k_stackCapacity);

    int id = stack[--sp];

    if (id == Node::Null)
      continue;

    const Node *n = &m_nodes[id];

    q3Vec3 e = n->aabb.max - n->aabb.min;
    q3Vec3 d = p1 - p0;
    q3Vec3 m = p0 + p1 - n->aabb.min - n->aabb.max;

    float adx = q3Abs(d.x);

    if (q3Abs(m.x) > e.x + adx)
      continue;

    float ady = q3Abs(d.y);

    if (q3Abs(m.y) > e.y + ady)
      continue;

    float adz = q3Abs(d.z);

    if (q3Abs(m.z) > e.z + adz)
      continue;

    adx += k_epsilon;
    ady += k_epsilon;
    adz += k_epsilon;

    if (q3Abs(m.y * d.z - m.z * d.y) > e.y * adz + e.z * ady)
      continue;

    if (q3Abs(m.z * d.x - m.x * d.z) > e.x * adz + e.z * adx)
      continue;

    if (q3Abs(m.x * d.y - m.y * d.x) > e.x * ady + e.y * adx)
      continue;

    if (n->IsLeaf()) {
      if (!cb->TreeCallBack(id))
        return;
    }

    else {
      stack[sp++] = n->left;
      stack[sp++] = n->right;
    }
  }
}
