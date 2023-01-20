//--------------------------------------------------------------------------------------------------
/**
@file	q3Body.cpp

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

#include "q3Body.h"
#include "../broadphase/q3BroadPhase.h"
#include "../dynamics/q3Contact.h"
#include "../math/q3Math.h"
#include "q3Box.h"
#include "q3Scene.h"

//--------------------------------------------------------------------------------------------------
// q3Body
//--------------------------------------------------------------------------------------------------
q3Body::q3Body(const q3BodyDef &def, q3Scene *scene) {
  m_linearVelocity = def.linearVelocity;
  m_angularVelocity = def.angularVelocity;
  m_force = {};
  m_torque = {};
  m_q.Set(q3Normalize(def.axis), def.angle);
  m_tx.rotation = m_q.ToMat3();
  m_tx.position = def.position;
  m_sleepTime = float(0.0);
  m_gravityScale = def.gravityScale;
  m_layers = def.layers;
  m_userData = def.userData;
  m_scene = scene;
  m_flags = {};
  m_linearDamping = def.linearDamping;
  m_angularDamping = def.angularDamping;

  if (def.bodyType == eDynamicBody)
    AddFlag(q3BodyFlags::eDynamic);

  else {
    if (def.bodyType == eStaticBody) {
      AddFlag(q3BodyFlags::eStatic);
      m_linearVelocity = {};
      m_angularVelocity = {};
      m_force = {};
      m_torque = {};
    }

    else if (def.bodyType == eKinematicBody)
      AddFlag(q3BodyFlags::eKinematic);
  }

  if (def.allowSleep)
    AddFlag(q3BodyFlags::eAllowSleep);

  if (def.awake)
    AddFlag(q3BodyFlags::eAwake);

  if (def.active)
    AddFlag(q3BodyFlags::eActive);

  if (def.lockAxisX)
    AddFlag(q3BodyFlags::eLockAxisX);

  if (def.lockAxisY)
    AddFlag(q3BodyFlags::eLockAxisY);

  if (def.lockAxisZ)
    AddFlag(q3BodyFlags::eLockAxisZ);

  m_contactList = NULL;
}

//--------------------------------------------------------------------------------------------------
void q3Body::RemoveBox(const q3Box *box) {
  assert(box);
  assert(box->body == this);

  auto node = std::find(m_boxes.begin(), m_boxes.end(), box);
  bool found = node != m_boxes.end();

  // This shape was not connected to this body.
  assert(found);
  m_boxes.erase(node);

  // Remove all contacts associated with this shape
  q3ContactEdge *edge = m_contactList;
  while (edge) {
    q3ContactConstraint *contact = edge->constraint;
    edge = edge->next;

    q3Box *A = contact->A;
    q3Box *B = contact->B;

    if (box == A || box == B)
      m_scene->m_contactManager.RemoveContact(contact);
  }

  m_scene->m_contactManager.m_broadphase.RemoveBox(box);

  CalculateMassData();
  delete box;
}

//--------------------------------------------------------------------------------------------------
void q3Body::RemoveAllBoxes() {
  for (auto box : m_boxes) {
    m_scene->m_contactManager.m_broadphase.RemoveBox(box);
    delete box;
  }
  m_boxes.clear();
  m_scene->m_contactManager.RemoveContactsFromBody(this);
}

//--------------------------------------------------------------------------------------------------
void q3Body::ApplyLinearForce(const q3Vec3 &force) {
  m_force += force * m_mass;

  SetToAwake();
}

//--------------------------------------------------------------------------------------------------
void q3Body::ApplyForceAtWorldPoint(const q3Vec3 &force, const q3Vec3 &point) {
  m_force += force * m_mass;
  m_torque += q3Cross(point - m_state.m_worldCenter, force);

  SetToAwake();
}

//--------------------------------------------------------------------------------------------------
void q3Body::ApplyLinearImpulse(const q3Vec3 &impulse) {
  m_linearVelocity += impulse * m_state.m_invMass;

  SetToAwake();
}

//--------------------------------------------------------------------------------------------------
void q3Body::ApplyLinearImpulseAtWorldPoint(const q3Vec3 &impulse,
                                            const q3Vec3 &point) {
  m_linearVelocity += impulse * m_state.m_invMass;
  m_angularVelocity += m_state.m_invInertiaWorld *
                       q3Cross(point - m_state.m_worldCenter, impulse);

  SetToAwake();
}

//--------------------------------------------------------------------------------------------------
void q3Body::ApplyTorque(const q3Vec3 &torque) { m_torque += torque; }

//--------------------------------------------------------------------------------------------------
void q3Body::SetToAwake() {
  if (!HasFlag(q3BodyFlags::eAwake)) {
    AddFlag(q3BodyFlags::eAwake);
    m_sleepTime = float(0.0);
  }
}

//--------------------------------------------------------------------------------------------------
void q3Body::SetToSleep() {
  RemoveFlag(q3BodyFlags::eAwake);
  m_sleepTime = float(0.0);
  m_linearVelocity = {};
  m_angularVelocity = {};
  m_force = {};
  m_torque = {};
}

//--------------------------------------------------------------------------------------------------
bool q3Body::IsAwake() const {
  return HasFlag(q3BodyFlags::eAwake) ? true : false;
}

//--------------------------------------------------------------------------------------------------
float q3Body::GetMass() const { return m_mass; }

//--------------------------------------------------------------------------------------------------
float q3Body::GetInvMass() const { return m_state.m_invMass; }

//--------------------------------------------------------------------------------------------------
float q3Body::GetGravityScale() const { return m_gravityScale; }

//--------------------------------------------------------------------------------------------------
void q3Body::SetGravityScale(float scale) { m_gravityScale = scale; }

//--------------------------------------------------------------------------------------------------
const q3Vec3 q3Body::GetLocalPoint(const q3Vec3 &p) const {
  return q3MulT(m_tx, p);
}

//--------------------------------------------------------------------------------------------------
const q3Vec3 q3Body::GetLocalVector(const q3Vec3 &v) const {
  return q3MulT(m_tx.rotation, v);
}

//--------------------------------------------------------------------------------------------------
const q3Vec3 q3Body::GetWorldPoint(const q3Vec3 &p) const {
  return q3Mul(m_tx, p);
}

//--------------------------------------------------------------------------------------------------
const q3Vec3 q3Body::GetWorldVector(const q3Vec3 &v) const {
  return q3Mul(m_tx.rotation, v);
}

//--------------------------------------------------------------------------------------------------
const q3Vec3 q3Body::GetLinearVelocity() const { return m_linearVelocity; }

//--------------------------------------------------------------------------------------------------
const q3Vec3 q3Body::GetVelocityAtWorldPoint(const q3Vec3 &p) const {
  q3Vec3 directionToPoint = p - m_state.m_worldCenter;
  q3Vec3 relativeAngularVel = q3Cross(m_angularVelocity, directionToPoint);

  return m_linearVelocity + relativeAngularVel;
}

//--------------------------------------------------------------------------------------------------
void q3Body::SetLinearVelocity(const q3Vec3 &v) {
  // Velocity of static bodies cannot be adjusted
  if (HasFlag(q3BodyFlags::eStatic))
    assert(false);

  if (q3Dot(v, v) > float(0.0)) {
    SetToAwake();
  }

  m_linearVelocity = v;
}

//--------------------------------------------------------------------------------------------------
const q3Vec3 q3Body::GetAngularVelocity() const { return m_angularVelocity; }

//--------------------------------------------------------------------------------------------------
void q3Body::SetAngularVelocity(const q3Vec3 v) {
  // Velocity of static bodies cannot be adjusted
  if (HasFlag(q3BodyFlags::eStatic))
    assert(false);

  if (q3Dot(v, v) > float(0.0)) {
    SetToAwake();
  }

  m_angularVelocity = v;
}

//--------------------------------------------------------------------------------------------------
bool q3Body::CanCollide(const q3Body *other) const {
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

//--------------------------------------------------------------------------------------------------
const q3Transform q3Body::GetTransform() const { return m_tx; }

//--------------------------------------------------------------------------------------------------
void q3Body::SetTransform(const q3Vec3 &position) {
  m_state.m_worldCenter = position;

  SynchronizeProxies();
}

//--------------------------------------------------------------------------------------------------
void q3Body::SetTransform(const q3Vec3 &position, const q3Vec3 &axis,
                          float angle) {
  m_state.m_worldCenter = position;
  m_q.Set(axis, angle);
  m_tx.rotation = m_q.ToMat3();

  SynchronizeProxies();
}

//--------------------------------------------------------------------------------------------------
void q3Body::SetLayers(int layers) { m_layers = layers; }

//--------------------------------------------------------------------------------------------------
int q3Body::GetLayers() const { return m_layers; }

//--------------------------------------------------------------------------------------------------
const q3Quaternion q3Body::GetQuaternion() const { return m_q; }

//--------------------------------------------------------------------------------------------------
void *q3Body::GetUserData() const { return m_userData; }

//--------------------------------------------------------------------------------------------------
void q3Body::SetLinearDamping(float damping) { m_linearDamping = damping; }

//--------------------------------------------------------------------------------------------------
float q3Body::GetLinearDamping(float damping) const { return m_linearDamping; }

//--------------------------------------------------------------------------------------------------
void q3Body::SetAngularDamping(float damping) { m_angularDamping = damping; }

//--------------------------------------------------------------------------------------------------
float q3Body::GetAngularDamping(float damping) const {
  return m_angularDamping;
}

void q3Body::Solve(const q3Env &env) {
  if (HasFlag(q3BodyFlags::eDynamic)) {
    ApplyLinearForce(env.m_gravity * m_gravityScale);

    // Calculate world space intertia tensor
    q3Mat3 r = m_tx.rotation;
    m_state.m_invInertiaWorld = r * m_invInertiaModel * q3Transpose(r);

    // Integrate velocity
    m_linearVelocity += (m_force * m_state.m_invMass) * env.m_dt;
    m_angularVelocity += (m_state.m_invInertiaWorld * m_torque) * env.m_dt;

    // From Box2D!
    // Apply damping.
    // ODE: dv/dt + c * v = 0
    // Solution: v(t) = v0 * exp(-c * t)
    // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) *
    // exp(-c * dt) = v * exp(-c * dt) v2 = exp(-c * dt) * v1 Pade
    // approximation: v2 = v1 * 1 / (1 + c * dt)
    m_linearVelocity *= float(1.0) / (float(1.0) + env.m_dt * m_linearDamping);
    m_angularVelocity *=
        float(1.0) / (float(1.0) + env.m_dt * m_angularDamping);
  }
}

//--------------------------------------------------------------------------------------------------
void q3Body::Render(q3Render *render) const {
  bool awake = IsAwake();
  for (auto box : m_boxes) {
    box->Render(m_tx, awake, render);
  }
}

//--------------------------------------------------------------------------------------------------
void q3Body::Dump(FILE *file, int index) const {
  fprintf(file, "{\n");
  fprintf(file, "\tq3BodyDef bd;\n");

  switch ((q3BodyFlags)((int)m_flags & ((int)q3BodyFlags::eStatic |
                                        (int)q3BodyFlags::eDynamic |
                                        (int)q3BodyFlags::eKinematic))) {
  case q3BodyFlags::eStatic:
    fprintf(file, "\tbd.bodyType = q3BodyType( %d );\n", eStaticBody);
    break;

  case q3BodyFlags::eDynamic:
    fprintf(file, "\tbd.bodyType = q3BodyType( %d );\n", eDynamicBody);
    break;

  case q3BodyFlags::eKinematic:
    fprintf(file, "\tbd.bodyType = q3BodyType( %d );\n", eKinematicBody);
    break;

  default:
    assert(false);
    break;
  }

  fprintf(file,
          "\tbd.position.Set( float( %.15lf ), float( %.15lf ), float( %.15lf "
          ") );\n",
          m_tx.position.x, m_tx.position.y, m_tx.position.z);
  q3Vec3 axis;
  float angle;
  m_q.ToAxisAngle(&axis, &angle);
  fprintf(
      file,
      "\tbd.axis.Set( float( %.15lf ), float( %.15lf ), float( %.15lf ) );\n",
      axis.x, axis.y, axis.z);
  fprintf(file, "\tbd.angle = float( %.15lf );\n", angle);
  fprintf(file,
          "\tbd.linearVelocity.Set( float( %.15lf ), float( %.15lf ), float( "
          "%.15lf ) );\n",
          m_linearVelocity.x, m_linearVelocity.y, m_linearVelocity.z);
  fprintf(file,
          "\tbd.angularVelocity.Set( float( %.15lf ), float( %.15lf ), float( "
          "%.15lf ) );\n",
          m_angularVelocity.x, m_angularVelocity.y, m_angularVelocity.z);
  fprintf(file, "\tbd.gravityScale = float( %.15lf );\n", m_gravityScale);
  fprintf(file, "\tbd.layers = %d;\n", m_layers);
  fprintf(file, "\tbd.allowSleep = bool( %d );\n",
          HasFlag(q3BodyFlags::eAllowSleep));
  fprintf(file, "\tbd.awake = bool( %d );\n", HasFlag(q3BodyFlags::eAwake));
  fprintf(file, "\tbd.awake = bool( %d );\n", HasFlag(q3BodyFlags::eAwake));
  fprintf(file, "\tbd.lockAxisX = bool( %d );\n",
          HasFlag(q3BodyFlags::eLockAxisX));
  fprintf(file, "\tbd.lockAxisY = bool( %d );\n",
          HasFlag(q3BodyFlags::eLockAxisY));
  fprintf(file, "\tbd.lockAxisZ = bool( %d );\n",
          HasFlag(q3BodyFlags::eLockAxisZ));
  fprintf(file, "\tbodies[ %d ] = scene.CreateBody( bd );\n\n", index);

  for (auto box : m_boxes) {
    box->Dump(file, index);
  }

  fprintf(file, "}\n\n");
}

void q3Body::SetVelocityState(const q3Env &env, const q3VelocityState &v) {
  if (HasFlag(q3BodyFlags::eStatic))
    return;
  m_linearVelocity = v.v;
  m_angularVelocity = v.w;
  // Integrate position
  m_state.m_worldCenter += m_linearVelocity * env.m_dt;
  m_q.Integrate(m_angularVelocity, env.m_dt);
  m_q = q3Normalize(m_q);
  m_tx.rotation = m_q.ToMat3();
}

void q3Body::Sleep(const q3Env &env, float *minSleepTime) {
  if (HasFlag(q3BodyFlags::eStatic))
    return;

  const float sqrLinVel = q3Dot(m_linearVelocity, m_linearVelocity);
  const float cbAngVel = q3Dot(m_angularVelocity, m_angularVelocity);
  const float linTol = Q3_SLEEP_LINEAR;
  const float angTol = Q3_SLEEP_ANGULAR;

  if (sqrLinVel > linTol || cbAngVel > angTol) {
    *minSleepTime = float(0.0);
    m_sleepTime = float(0.0);
  }

  else {
    m_sleepTime += env.m_dt;
    *minSleepTime = q3Min(*minSleepTime, m_sleepTime);
  }
}

//--------------------------------------------------------------------------------------------------
void q3Body::CalculateMassData() {
  q3Mat3 inertia = q3Diagonal(float(0.0));
  m_invInertiaModel = q3Diagonal(float(0.0));
  m_state.m_invInertiaWorld = q3Diagonal(float(0.0));
  m_state.m_invMass = float(0.0);
  m_mass = float(0.0);
  float mass = float(0.0);

  if (HasFlag(q3BodyFlags::eStatic) || HasFlag(q3BodyFlags::eKinematic)) {
    m_localCenter = {};
    m_state.m_worldCenter = m_tx.position;
    return;
  }

  q3Vec3 lc = {};
  for (auto box : m_boxes) {
    if (box->density == float(0.0))
      continue;

    q3MassData md;
    box->ComputeMass(&md);
    mass += md.mass;
    inertia += md.inertia;
    lc += md.center * md.mass;
  }

  if (mass > float(0.0)) {
    m_mass = mass;
    m_state.m_invMass = float(1.0) / mass;
    lc *= m_state.m_invMass;
    q3Mat3 identity = {};
    inertia -= (identity * q3Dot(lc, lc) - q3OuterProduct(lc, lc)) * mass;
    m_invInertiaModel = q3Inverse(inertia);

    if (HasFlag(q3BodyFlags::eLockAxisX))
      m_invInertiaModel.ex = {};

    if (HasFlag(q3BodyFlags::eLockAxisY))
      m_invInertiaModel.ey = {};

    if (HasFlag(q3BodyFlags::eLockAxisZ))
      m_invInertiaModel.ez = {};
  } else {
    // Force all dynamic bodies to have some mass
    m_state.m_invMass = float(1.0);
    m_invInertiaModel = q3Diagonal(float(0.0));
    m_state.m_invInertiaWorld = q3Diagonal(float(0.0));
  }

  m_localCenter = lc;
  m_state.m_worldCenter = q3Mul(m_tx, lc);
}

//--------------------------------------------------------------------------------------------------
void q3Body::SynchronizeProxies() {
  q3BroadPhase *broadphase = &m_scene->m_contactManager.m_broadphase;

  m_tx.position = m_state.m_worldCenter - q3Mul(m_tx.rotation, m_localCenter);

  q3AABB aabb;
  q3Transform tx = m_tx;

  for (auto box : m_boxes) {
    box->ComputeAABB(tx, &aabb);
    broadphase->Update(box->broadPhaseIndex, aabb);
  }
}