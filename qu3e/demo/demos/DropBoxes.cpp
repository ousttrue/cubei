#include "DropBoxes.h"

void DropBoxes::Init(q3Scene *scene) {
  acc = {};

  // Create the floor
  q3BodyDef bodyDef = {};
  // bodyDef.axis.Set( q3RandomFloat( -1.0f, 1.0f ), q3RandomFloat(
  // -1.0f, 1.0f ), q3RandomFloat( -1.0f, 1.0f ) ); bodyDef.angle = q3PI *
  // q3RandomFloat( -1.0f, 1.0f );
  q3Body *body = scene->CreateBody(bodyDef);

  q3BoxDef boxDef = {
      .m_restitution = 0,
  };
  q3Transform tx;
  q3Identity(tx);
  boxDef.Set(tx, q3Vec3(50.0f, 1.0f, 50.0f));
  body->AddBox(boxDef);

  // Create boxes
  for (int i = 0; i < 10; ++i) {
    bodyDef.position.Set(0.0f, 1.2f * (i + 1), -0.0f);
    // bodyDef.axis.Set( 0.0f, 1.0f, 0.0f );
    // bodyDef.angle = q3PI * q3RandomFloat( -1.0f, 1.0f );
    // bodyDef.angularVelocity.Set( 3.0f, 3.0f, 3.0f );
    // bodyDef.linearVelocity.Set( 2.0f, 0.0f, 0.0f );
    bodyDef.bodyType = eDynamicBody;
    body = scene->CreateBody(bodyDef);
    boxDef.Set(tx, q3Vec3(1.0f, 1.0f, 1.0f));
    body->AddBox(boxDef);
  }
}

void DropBoxes::Update(q3Scene *scene, std::chrono::nanoseconds dt) {
  acc += dt;

  if (acc > std::chrono::seconds(1)) {
    acc = {};

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
    q3BoxDef boxDef = {};
    boxDef.Set(tx, q3Vec3(1.0f, 1.0f, 1.0f));
    body->AddBox(boxDef);
  }
}
