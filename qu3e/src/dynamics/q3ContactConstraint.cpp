#include "q3ContactConstraint.h"

// Restitution mixing. The idea is to use the maximum bounciness, so bouncy
// objects will never not bounce during collisions.
static float q3MixRestitution(const q3Box *A, const q3Box *B) {
  return std::max(A->Restitution(), B->Restitution());
}

// Friction mixing. The idea is to allow a very low friction value to
// drive down the mixing result. Example: anything slides on ice.
static float q3MixFriction(const q3Box *A, const q3Box *B) {
  return std::sqrt(A->Friction() * B->Friction());
}

q3ContactConstraint::q3ContactConstraint(q3Box *A, q3Body *bodyA, q3Box *B,
                                         q3Body *bodyB)
    : A(A), B(B), bodyA(bodyA), bodyB(bodyB) {
  friction = q3MixFriction(A, B);
  restitution = q3MixRestitution(A, B);

  manifold.SetPair(bodyA, A, bodyB, B);
  manifold.contactCount = 0;
  for (int i = 0; i < 8; ++i)
    manifold.contacts[i].warmStarted = 0;
}

bool q3ContactConstraint::Test(
    const std::function<bool(q3Box *, q3Box *)> &testOverlap) {
  // auto constraint = *it;
  this->RemoveFlag(q3ContactConstraintFlags::eIsland);

  if (!bodyA->IsAwake() && !bodyB->IsAwake()) {
    return true;
  }

  if (!bodyA->CanCollide(bodyB)) {
    return false;
  }

  // Check if contact should persist
  if (!testOverlap(A, B)) {
    return false;
  }

  // Solves contact manifolds
  q3Manifold *manifold = &this->manifold;
  q3Manifold oldManifold = this->manifold;
  q3Vec3 ot0 = oldManifold.tangentVectors[0];
  q3Vec3 ot1 = oldManifold.tangentVectors[1];
  manifold->contactCount = 0;

  q3BoxtoBox(manifold, bodyA, A, bodyB, B);

  if (manifold->contactCount > 0) {
    if (this->HasFlag(q3ContactConstraintFlags::eColliding)) {
      this->AddFlag(q3ContactConstraintFlags::eWasColliding);
    } else {
      this->AddFlag(q3ContactConstraintFlags::eColliding);
    }
  } else {
    if (this->HasFlag(q3ContactConstraintFlags::eColliding)) {
      this->RemoveFlag(q3ContactConstraintFlags::eColliding);
      this->AddFlag(q3ContactConstraintFlags::eWasColliding);
    } else {
      this->RemoveFlag(q3ContactConstraintFlags::eWasColliding);
    }
  }
  // }

  q3ComputeBasis(manifold->normal, manifold->tangentVectors,
                 manifold->tangentVectors + 1);

  for (int i = 0; i < manifold->contactCount; ++i) {
    q3Contact *c = manifold->contacts + i;
    c->tangentImpulse[0] = c->tangentImpulse[1] = c->normalImpulse = float(0.0);
    uint8_t oldWarmStart = c->warmStarted;
    c->warmStarted = uint8_t(0);

    for (int j = 0; j < oldManifold.contactCount; ++j) {
      q3Contact *oc = oldManifold.contacts + j;
      if (c->fp.key == oc->fp.key) {
        c->normalImpulse = oc->normalImpulse;

        // Attempt to re-project old friction solutions
        q3Vec3 friction =
            ot0 * oc->tangentImpulse[0] + ot1 * oc->tangentImpulse[1];
        c->tangentImpulse[0] = q3Dot(friction, manifold->tangentVectors[0]);
        c->tangentImpulse[1] = q3Dot(friction, manifold->tangentVectors[1]);
        c->warmStarted = std::max(oldWarmStart, uint8_t(oldWarmStart + 1));
        break;
      }
    }
  }

  return true;
}
