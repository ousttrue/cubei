#include "Test.h"

void Test::Init(q3Scene *scene) {
  q3BodyDef bodyDef;
  q3Body *body = scene->CreateBody(bodyDef);
  q3BoxDef boxDef{
      .m_restitution = 0,
  };
  q3Transform tx = {};
  boxDef.Set(tx, {50.0f, 1.0f, 50.0f});
  body->AddBox(boxDef);

  bodyDef.bodyType = eDynamicBody;
  bodyDef.position.Set(0, 5.0f, 0);
  body = scene->CreateBody(bodyDef);
  for (int i = 0; i < 20; ++i) {
    tx.position.Set(q3RandomFloat(1.0f, 10.0f), q3RandomFloat(1.0f, 10.0f),
                    q3RandomFloat(1.0f, 10.0f));
    boxDef.Set(tx, {1.0f, 1.0f, 1.0f});
    body->AddBox(boxDef);
  }
}
