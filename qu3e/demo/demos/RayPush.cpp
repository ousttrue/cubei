#include "RayPush.h"

void RayPush::Init(q3Scene *scene) {
  acc = std::chrono::nanoseconds(0);

  // Create the floor
  {
    auto body = scene->CreateBody({}, &scene->m_contactManager);
    scene->AddBox(body, {
                            .m_tx = {},
                            .m_e = q3Vec3{50.0f, 1.0f, 50.0f} * 0.5f,
                            .m_restitution = 0,
                        });
  }
}

void RayPush::Update(q3Scene *scene, std::chrono::nanoseconds dt) {
  acc += dt;

  if (acc > std::chrono::seconds(1)) {
    acc = std::chrono::nanoseconds(0);

    q3BodyDef bodyDef{
        .axis =
            {
                q3RandomFloat(-1.0f, 1.0f),
                q3RandomFloat(-1.0f, 1.0f),
                q3RandomFloat(-1.0f, 1.0f),
            },
        .angle = q3PI * q3RandomFloat(-1.0f, 1.0f),
        .position = {0.0f, 3.0f, 0.0f},
        .linearVelocity =
            q3Vec3{q3RandomFloat(1.0f, 3.0f), q3RandomFloat(1.0f, 3.0f),
                   q3RandomFloat(1.0f, 3.0f)} *
            q3Sign(q3RandomFloat(-1.0f, 1.0f)),
        .angularVelocity =
            q3Vec3{q3RandomFloat(1.0f, 3.0f), q3RandomFloat(1.0f, 3.0f),
                   q3RandomFloat(1.0f, 3.0f)} *
            q3Sign(q3RandomFloat(-1.0f, 1.0f)),
        .bodyType = eDynamicBody,
    };
    auto body = scene->CreateBody(bodyDef, &scene->m_contactManager);

    scene->AddBox(body, {
                            .m_tx = {},
                            .m_e = q3Vec3{1.0f, 1.0f, 1.0f} * 0.5f,
                        });
  }

  rayCast.Init({3.0f, 5.0f, 3.0f}, {-1.0f, -1.0f, -1.0f});
  scene->m_contactManager.RayCast(&rayCast, rayCast.data);

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
