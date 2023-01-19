#include "Test.h"

void Test::Init(q3Scene *scene) {
  {
    auto body = scene->CreateBody({});
    body->AddBox({
        .m_tx = {},
        .m_e = {50.0f, 1.0f, 50.0f},
        .m_restitution = 0,
    });
  }

  {
    auto body = scene->CreateBody({
        .position = {0, 5.0f, 0},
        .bodyType = eDynamicBody,
    });
    for (int i = 0; i < 20; ++i) {
      q3BoxDef boxDef{
          .m_tx =
              {
                  .position =
                      {
                          q3RandomFloat(1.0f, 10.0f),
                          q3RandomFloat(1.0f, 10.0f),
                          q3RandomFloat(1.0f, 10.0f),
                      },
              },
          .m_e = {1.0f, 1.0f, 1.0f},
          .m_restitution = 0,
      };
      body->AddBox(boxDef);
    }
  }
}