#pragma once
#include <stdint.h>

// in stands for "incoming"
// out stands for "outgoing"
// I stands for "incident"
// R stands for "reference"
// See D. Gregorius GDC 2015 on creating contacts for more details
// Each feature pair is used to cache solutions from one physics tick to
// another. This is called warmstarting, and lets boxes stack and stay stable.
// Feature pairs identify points of contact over multiple physics ticks. Each
// feature pair is the junction of an incoming feature and an outgoing feature,
// usually a result of clipping routines. The exact info stored in the feature
// pair can be arbitrary as long as the result is a unique ID for a given
// intersecting configuration.
union q3FeaturePair {
  struct {
    uint8_t inR;
    uint8_t outR;
    uint8_t inI;
    uint8_t outI;
  };

  int key;
};
