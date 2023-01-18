#include "BoxStack.h"

void BoxStack::Init(q3Scene *scene) {
  // Create the floor
  q3BodyDef bodyDef = {};
  // bodyDef.axis.Set( q3RandomFloat( -1.0f, 1.0f ), q3RandomFloat(
  // -1.0f, 1.0f ), q3RandomFloat( -1.0f, 1.0f ) ); bodyDef.angle = q3PI *
  // q3RandomFloat( -1.0f, 1.0f );
  q3Body *body = scene->CreateBody(bodyDef);

  q3BoxDef boxDef{
      .m_restitution = 0,
  };
  q3Transform tx;
  q3Identity(tx);
  boxDef.Set(tx, q3Vec3(50.0f, 1.0f, 50.0f));
  body->AddBox(boxDef);

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

  bodyDef.bodyType = eDynamicBody;
  boxDef.Set(tx, q3Vec3(1.0f, 1.0f, 1.0f));

  for (int i = 0; i < 8; ++i) {
    for (int j = 0; j < 8; ++j) {
      for (int k = 0; k < 10; ++k) {
        bodyDef.position.Set(-16.0f + 1.0f * j, 1.0f * i + 5.0f,
                             -16.0f + 1.0f * k);
        body = scene->CreateBody(bodyDef);
        body->AddBox(boxDef);
      }
    }
  }
}
