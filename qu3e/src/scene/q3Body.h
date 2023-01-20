//--------------------------------------------------------------------------------------------------
/**
@file	q3Body.h

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
#include "../math/q3Math.h"
#include "../math/q3Transform.h"
#include <functional>
#include <list>
#include <stdio.h>

class q3Scene;
struct q3BoxDef;
struct q3ContactEdge;
struct q3Box;

enum q3BodyType {
  eStaticBody,
  eDynamicBody,
  eKinematicBody,
};

enum class q3BodyFlags {
  eNone = 0,
  eAwake = 0x001,
  eActive = 0x002,
  eAllowSleep = 0x004,
  eIsland = 0x010,
  eStatic = 0x020,
  eDynamic = 0x040,
  eKinematic = 0x080,
  eLockAxisX = 0x100,
  eLockAxisY = 0x200,
  eLockAxisZ = 0x400,
};

//--------------------------------------------------------------------------------------------------
// q3BodyDef
//--------------------------------------------------------------------------------------------------
struct q3BodyDef {
  q3Vec3 axis = {};            // Initial world transformation.
  float angle = {};            // Initial world transformation. Radians.
  q3Vec3 position = {};        // Initial world transformation.
  q3Vec3 linearVelocity = {};  // Initial linear velocity in world space.
  q3Vec3 angularVelocity = {}; // Initial angular velocity in world space.
                               // Usually a gravity scale of 1 is the best
  float gravityScale = 1.0f;   // Convenient scale values for gravity x, y and z
                               // directions.
  int layers = 0x000000001; // Bitmask of collision layers. Bodies matching at
                            // least one layer can collide.
  void *userData = nullptr; // Use to store application specific data.

  float linearDamping = 0;
  float angularDamping = 0.1f;

  // Static, dynamic or kinematic. Dynamic bodies with zero mass are defaulted
  // to a mass of 1. Static bodies never move or integrate, and are very CPU
  // efficient. Static bodies have infinite mass. Kinematic bodies have
  // infinite mass, but *do* integrate and move around. Kinematic bodies do not
  // resolve any collisions.
  q3BodyType bodyType = eStaticBody;

  bool allowSleep = true; // Sleeping lets a body assume a non-moving state.
                          // Greatly reduces CPU usage.
  bool awake = true;      // Initial sleep state. True means awake.
  bool active = true; // A body can start out inactive and just sits in memory.
  bool lockAxisX = false; // Locked rotation on the x axis.
  bool lockAxisY = false; // Locked rotation on the y axis.
  bool lockAxisZ = false; // Locked rotation on the z axis.
};

struct q3VelocityState {
  q3Vec3 w;
  q3Vec3 v;
};

struct q3BodyState {
  q3Vec3 m_worldCenter;
  q3Mat3 m_invInertiaWorld;
  float m_invMass;
  int m_islandIndex;
};

//--------------------------------------------------------------------------------------------------
// q3Body
//--------------------------------------------------------------------------------------------------
class q3Body {
  q3BodyState m_state;
  q3Mat3 m_invInertiaModel;
  float m_mass;
  q3Vec3 m_linearVelocity;
  q3Vec3 m_angularVelocity;
  q3Vec3 m_force = {};
  q3Vec3 m_torque = {};
  q3Transform m_tx;
  q3Quaternion m_q;
  q3Vec3 m_localCenter;
  float m_sleepTime = {};
  float m_gravityScale;
  int m_layers;
  q3BodyFlags m_flags = {};
  std::list<q3Box *> m_boxes;
  void *m_userData;
  float m_linearDamping;
  float m_angularDamping;
  std::function<void(const q3Box *)> m_onRemoveBox;
  std::function<void(struct q3ContactConstraint *)> m_onRemoveConstraint;

public:
  std::function<void()> m_transformUpdated;
  q3ContactEdge *m_contactList = nullptr;
  q3Body(const q3BodyDef &def, q3Scene *scene);
  q3BodyState State() const { return m_state; }
  void SetIslandIndex(size_t index) { m_state.m_islandIndex = index; }
  q3VelocityState VelocityState() const {
    return {
        .w = m_angularVelocity,
        .v = m_linearVelocity,
    };
  }
  q3Transform UpdatePosition() {
    m_tx.position = m_state.m_worldCenter - q3Mul(m_tx.rotation, m_localCenter);
    return m_tx;
  }
  std::list<q3Box *>::const_iterator begin() const { return m_boxes.begin(); }
  std::list<q3Box *>::const_iterator end() const { return m_boxes.end(); }
  void SetVelocityState(const struct q3Env &env, const q3VelocityState &v);
  void Sleep(const q3Env &env, float *minSleepTime);
  void ClearForce() {
    m_force = {};
    m_torque = {};
  }
  void AddBox(q3Box *box) { m_boxes.push_back(box); }
  q3Transform Transform() const { return m_tx; }
  void CalculateMassData();
  q3BodyFlags GetFlags() const { return m_flags; }
  bool HasFlag(q3BodyFlags flag) const {
    return ((int)m_flags & (int)flag) != 0;
  }
  void AddFlag(q3BodyFlags flag) {
    m_flags = (q3BodyFlags)((int)m_flags | (int)flag);
  }
  void RemoveFlag(q3BodyFlags flag) {
    m_flags = (q3BodyFlags)((int)m_flags & ~(int)flag);
  }
  // Removes this box from the body and broadphase. Forces the body
  // to recompute its mass if the body is dynamic. Frees the memory
  // pointed to by the box pointer.
  void RemoveBox(const q3Box *box);

  // Removes all boxes from this body and the broadphase.
  void RemoveAllBoxes();

  void ApplyLinearForce(const q3Vec3 &force) {
    m_force += force * m_mass;
    SetToAwake();
  }
  void ApplyForceAtWorldPoint(const q3Vec3 &force, const q3Vec3 &point) {
    m_force += force * m_mass;
    m_torque += q3Cross(point - m_state.m_worldCenter, force);
    SetToAwake();
  }
  void ApplyLinearImpulse(const q3Vec3 &impulse) {
    m_linearVelocity += impulse * m_state.m_invMass;
    SetToAwake();
  }
  void ApplyLinearImpulseAtWorldPoint(const q3Vec3 &impulse,
                                      const q3Vec3 &point) {
    m_linearVelocity += impulse * m_state.m_invMass;
    m_angularVelocity += m_state.m_invInertiaWorld *
                         q3Cross(point - m_state.m_worldCenter, impulse);
    SetToAwake();
  }
  void SetToAwake() {
    if (!HasFlag(q3BodyFlags::eAwake)) {
      AddFlag(q3BodyFlags::eAwake);
      m_sleepTime = float(0.0);
    }
  }
  void SetToSleep() {
    RemoveFlag(q3BodyFlags::eAwake);
    m_sleepTime = float(0.0);
    m_linearVelocity = {};
    m_angularVelocity = {};
    m_force = {};
    m_torque = {};
  }
  void ApplyTorque(const q3Vec3 &torque) { m_torque += torque; }
  bool IsAwake() const { return HasFlag(q3BodyFlags::eAwake) ? true : false; }
  float GetGravityScale() const { return m_gravityScale; }
  void SetGravityScale(float scale) { m_gravityScale = scale; }
  const q3Vec3 GetLocalPoint(const q3Vec3 &p) const { return q3MulT(m_tx, p); }
  const q3Vec3 GetLocalVector(const q3Vec3 &v) const {
    return q3MulT(m_tx.rotation, v);
  }
  const q3Vec3 GetWorldPoint(const q3Vec3 &p) const { return q3Mul(m_tx, p); }
  float GetMass() const { return m_mass; }
  float GetInvMass() const { return m_state.m_invMass; }
  const q3Vec3 GetWorldVector(const q3Vec3 &v) const {
    return q3Mul(m_tx.rotation, v);
  }
  const q3Vec3 GetLinearVelocity() const { return m_linearVelocity; }
  const q3Vec3 GetVelocityAtWorldPoint(const q3Vec3 &p) const {
    q3Vec3 directionToPoint = p - m_state.m_worldCenter;
    q3Vec3 relativeAngularVel = q3Cross(m_angularVelocity, directionToPoint);

    return m_linearVelocity + relativeAngularVel;
  }
  void SetLinearVelocity(const q3Vec3 &v) {
    // Velocity of static bodies cannot be adjusted
    if (HasFlag(q3BodyFlags::eStatic))
      assert(false);

    if (q3Dot(v, v) > float(0.0)) {
      SetToAwake();
    }

    m_linearVelocity = v;
  }
  bool CanCollide(const q3Body *other) const {
    if (this == other)
      return false;

    // Every collision must have at least one dynamic body involved
    if (!(HasFlag(q3BodyFlags::eDynamic)) &&
        !(other->HasFlag(q3BodyFlags::eDynamic)))
      return false;

    if (!(m_layers & other->m_layers))
      return false;

    return true;
  }
  const q3Vec3 GetAngularVelocity() const { return m_angularVelocity; }
  void SetAngularVelocity(const q3Vec3 v) {
    // Velocity of static bodies cannot be adjusted
    if (HasFlag(q3BodyFlags::eStatic))
      assert(false);
    if (q3Dot(v, v) > float(0.0)) {
      SetToAwake();
    }
    m_angularVelocity = v;
  }
  const q3Transform GetTransform() const { return m_tx; }
  void SetLayers(int layers) { m_layers = layers; }
  int GetLayers() const { return m_layers; }
  const q3Quaternion GetQuaternion() const { return m_q; }
  void *GetUserData() const { return m_userData; }
  void SetLinearDamping(float damping) { m_linearDamping = damping; }
  float GetLinearDamping(float damping) const { return m_linearDamping; }
  void SetAngularDamping(float damping) { m_angularDamping = damping; }
  float GetAngularDamping(float damping) const { return m_angularDamping; }

  // Manipulating the transformation of a body manually will result in
  // non-physical behavior. Contacts are updated upon the next call to
  // q3Scene::Step( ). Parameters are in world space. All body types
  // can be updated.
  void SetTransform(const q3Vec3 &position) {
    m_state.m_worldCenter = position;
    m_transformUpdated();
  }
  void SetTransform(const q3Vec3 &position, const q3Vec3 &axis, float angle) {
    m_state.m_worldCenter = position;
    m_q.Set(axis, angle);
    m_tx.rotation = m_q.ToMat3();
    m_transformUpdated();
  }

  void Solve(const struct q3Env &env);

  // Used for debug rendering lines, triangles and basic lighting
  void Render(class q3Render *render) const;

  // Dump this rigid body and its shapes into a log file. The log can be
  // used as C++ code to re-create an initial scene setup.
  void Dump(FILE *file, int index) const;
};
