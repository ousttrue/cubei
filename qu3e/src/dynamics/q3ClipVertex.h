#pragma once
#include "../math/q3Vec3.h"
#include "q3FeaturePair.h"

struct q3ClipVertex {
  q3ClipVertex() { f.key = ~0; }

  q3Vec3 v;
  q3FeaturePair f;
};
