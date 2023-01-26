#pragma once
#include "../scene/q3Body.h"
#include "../scene/q3Box.h"
#include "q3Contact.h"

struct q3Manifold {
  void SetPair(q3Body *bodyA, q3Box *a, q3Body *bodyB, q3Box *b) {
    A = {bodyA, a};
    B = {bodyB, b};
    sensor = a->Sensor() || b->Sensor();
  }

  std::tuple<q3Body *, q3Box *> A;
  std::tuple<q3Body *, q3Box *> B;
  q3Contact contacts[8];
  int contactCount;

  q3Vec3 normal;            // From A to B
  q3Vec3 tangentVectors[2]; // Tangent vectors

  q3Manifold *next;
  q3Manifold *prev;

  bool sensor;
};

void q3BoxtoBox(q3Manifold *m, q3Body *a_body, q3Box *a, q3Body *b_body,
                q3Box *b);
