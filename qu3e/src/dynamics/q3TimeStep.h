#pragma once
#include "../scene/q3Env.h"

// Run the simulation forward in time by dt (fixed timestep). Variable
// timestep is not supported.
void q3TimeStep(const q3Env &env, class q3Scene *scene,
                class q3BroadPhase *broadPhase,
                class q3ContactManager *contactManager);
