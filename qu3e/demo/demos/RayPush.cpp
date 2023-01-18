#include "RayPush.h"

void RayPush::Init(q3Scene *scene) {
  acc = std::chrono::nanoseconds(0);

  // Create the floor
  q3BodyDef bodyDef;
  q3Body *body = scene->CreateBody(bodyDef);

  q3BoxDef boxDef;
  boxDef.SetRestitution(0);
  q3Transform tx;
  q3Identity(tx);
  boxDef.Set(tx, q3Vec3(50.0f, 1.0f, 50.0f));
  body->AddBox(boxDef);
}

void RayPush::Update(q3Scene *scene, std::chrono::nanoseconds dt) {
  acc += dt;

  if (acc > std::chrono::seconds(1)) {
    acc = std::chrono::nanoseconds(0);

    q3BodyDef bodyDef;
    bodyDef.position.Set(0.0f, 3.0f, 0.0f);
    bodyDef.axis.Set(q3RandomFloat(-1.0f, 1.0f), q3RandomFloat(-1.0f, 1.0f),
                     q3RandomFloat(-1.0f, 1.0f));
    bodyDef.angle = q3PI * q3RandomFloat(-1.0f, 1.0f);
    bodyDef.bodyType = eDynamicBody;
    bodyDef.angularVelocity.Set(q3RandomFloat(1.0f, 3.0f),
                                q3RandomFloat(1.0f, 3.0f),
                                q3RandomFloat(1.0f, 3.0f));
    bodyDef.angularVelocity *= q3Sign(q3RandomFloat(-1.0f, 1.0f));
    bodyDef.linearVelocity.Set(q3RandomFloat(1.0f, 3.0f),
                               q3RandomFloat(1.0f, 3.0f),
                               q3RandomFloat(1.0f, 3.0f));
    bodyDef.linearVelocity *= q3Sign(q3RandomFloat(-1.0f, 1.0f));
    q3Body *body = scene->CreateBody(bodyDef);

    q3Transform tx;
    q3Identity(tx);
    q3BoxDef boxDef;
    boxDef.Set(tx, q3Vec3(1.0f, 1.0f, 1.0f));
    body->AddBox(boxDef);
  }

  rayCast.Init(q3Vec3(3.0f, 5.0f, 3.0f), q3Vec3(-1.0f, -1.0f, -1.0f));
  scene->RayCast(&rayCast, rayCast.data);

  if (rayCast.impactBody) {
    rayCast.impactBody->SetToAwake();
    rayCast.impactBody->ApplyForceAtWorldPoint(rayCast.data.dir * 20.0f,
                                               rayCast.data.GetImpactPoint());
  }
}

void RayPush::Render(q3Render *render) {
  render->SetScale(1.0f, 1.0f, 1.0f);
  render->SetPenColor(0.2f, 0.5f, 1.0f);
  render->SetPenPosition(rayCast.data.start.x, rayCast.data.start.y,
                         rayCast.data.start.z);
  q3Vec3 impact = rayCast.data.GetImpactPoint();
  render->Line(impact.x, impact.y, impact.z);

  render->SetPenPosition(impact.x, impact.y, impact.z);
  render->SetPenColor(1.0f, 0.5f, 0.5f);
  render->SetScale(10.0f, 10.0f, 10.0f);
  render->Point();

  render->SetPenColor(1.0f, 0.5f, 0.2f);
  render->SetScale(1.0f, 1.0f, 1.0f);
  impact += rayCast.nfinal * 2.0f;
  render->Line(impact.x, impact.y, impact.z);
}
