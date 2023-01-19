#include "BoxStack.h"

void BoxStack::Init(q3Scene *scene) {
  // Create the floor
  // bodyDef.axis.Set( q3RandomFloat( -1.0f, 1.0f ), q3RandomFloat(
  // -1.0f, 1.0f ), q3RandomFloat( -1.0f, 1.0f ) ); bodyDef.angle = q3PI *
  // q3RandomFloat( -1.0f, 1.0f );
  {
    auto body = scene->CreateBody({});
    q3BoxDef boxDef{
        .m_tx = {},
        .m_e = q3Vec3{50.0f, 1.0f, 50.0f} * 0.5f,
        .m_restitution = 0,
    };
    body->AddBox(boxDef);
  }

  // Create boxes
  // for ( int i = 0; i < 10; ++i )
  //{
  //	bodyDef.position.Set( 0.0f, 1.2f * (i + 1), -0.0f );
  //	//bodyDef.axis.Set( 0.0f, 1.0f, 0.0f );
  //	//bodyDef.angle = q3PI * q3RandomFloat( -1.0f, 1.0f );
  //	//bodyDef.angularVelocity.Set( 3.0f, 3.0f, 3.0f );
  //	//bodyDef.linearVelocity.Set( 2.0f, 0.0f, 0.0f );
  //	bodyDef.bodyType = eDynamicBody;
  //	body = scene->CreateBody( bodyDef );
  //	boxDef.Set( tx, q3Vec3( 1.0f, 1.0f, 1.0f ) );
  //	body->AddBox( boxDef );
  //}

  for (int i = 0; i < 8; ++i) {
    for (int j = 0; j < 8; ++j) {
      for (int k = 0; k < 10; ++k) {
        auto body = scene->CreateBody({
            .position =
                {
                    -16.0f + 1.0f * j,
                    1.0f * i + 5.0f,
                    -16.0f + 1.0f * k,
                },
            .bodyType = eDynamicBody,
        });
        body->AddBox({
            .m_tx = {},
            .m_e = q3Vec3{1.0f, 1.0f, 1.0f} * 0.5f,
            .m_restitution = 0,
        });
      }
    }
  }
}
