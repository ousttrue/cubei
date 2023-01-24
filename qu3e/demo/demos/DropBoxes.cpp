#include "DropBoxes.h"

void DropBoxes::Init(q3Scene *scene) {
  acc = {};

  // Create the floor
  // bodyDef.axis.Set( q3RandomFloat( -1.0f, 1.0f ), q3RandomFloat(
  // -1.0f, 1.0f ), q3RandomFloat( -1.0f, 1.0f ) ); bodyDef.angle = q3PI *
  // q3RandomFloat( -1.0f, 1.0f );
  {
    auto body = scene->CreateBody({});
    scene->AddBox(body, {
                            .m_tx = {},
                            .m_e = q3Vec3{50.0f, 1.0f, 50.0f} * 0.5f,
                            .m_restitution = 0,
                        });
  }

  // Create boxes
  for (int i = 0; i < 10; ++i) {
    // bodyDef.axis.Set( 0.0f, 1.0f, 0.0f );
    // bodyDef.angle = q3PI * q3RandomFloat( -1.0f, 1.0f );
    // bodyDef.angularVelocity.Set( 3.0f, 3.0f, 3.0f );
    // bodyDef.linearVelocity.Set( 2.0f, 0.0f, 0.0f );
    auto body = scene->CreateBody({
        .position = {0.0f, 1.2f * (i + 1), -0.0f},
        .bodyType = eDynamicBody,
    });
    scene->AddBox(body, {
                            .m_tx = {},
                            .m_e = q3Vec3{1.0f, 1.0f, 1.0f} * 0.5f,
                            .m_restitution = 0,
                        });
  }
}

void DropBoxes::Update(q3Scene *scene, std::chrono::nanoseconds dt,
                       q3BroadPhase *, q3ContactManager *) {
  acc += dt;

  if (acc > std::chrono::seconds(1)) {
    acc = {};

    q3BodyDef bodyDef{
        .axis = {q3RandomFloat(-1.0f, 1.0f), q3RandomFloat(-1.0f, 1.0f),
                 q3RandomFloat(-1.0f, 1.0f)},
        .angle = q3PI * q3RandomFloat(-1.0f, 1.0f),
        .position = {0.0f, 3.0f, 0.0f},
        .linearVelocity =
            q3Vec3{
                q3RandomFloat(1.0f, 3.0f),
                q3RandomFloat(1.0f, 3.0f),
                q3RandomFloat(1.0f, 3.0f),
            } *
            q3Sign(q3RandomFloat(-1.0f, 1.0f)),
        .angularVelocity =
            q3Vec3{q3RandomFloat(1.0f, 3.0f), q3RandomFloat(1.0f, 3.0f),
                   q3RandomFloat(1.0f, 3.0f)} *
            q3Sign(q3RandomFloat(-1.0f, 1.0f)),
        .bodyType = eDynamicBody,
    };
    auto body = scene->CreateBody(bodyDef);

    scene->AddBox(body, {
                            .m_tx = {},
                            .m_e = q3Vec3{1.0f, 1.0f, 1.0f} * 0.5f,
                        });
  }
}
