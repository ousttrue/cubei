//--------------------------------------------------------------------------------------------------
/**
@file	q3Contact.h

@author	Randy Gaul
@date	10/10/2014

        Copyright (c) 2014 Randy Gaul http://www.randygaul.net

        This software is provided 'as-is', without any express or implied
        warranty. In no event will the authors be held liable for any damages
        arising from the use of this software.

        Permission is granted to anyone to use this software for any purpose,
        including commercial applications, and to alter it and redistribute it
        freely, subject to the following restrictions:
          1. The origin of this software must not be misrepresented; you must
not claim that you wrote the original software. If you use this software in a
product, an acknowledgment in the product documentation would be appreciated but
is not required.
          2. Altered source versions must be plainly marked as such, and must
not be misrepresented as being the original software.
          3. This notice may not be removed or altered from any source
distribution.
*/
//--------------------------------------------------------------------------------------------------

#pragma once
#include "../math/q3Vec3.h"
#include "q3FeaturePair.h"

struct q3Contact {
  q3FeaturePair fp;        // Features on A and B for this contact
  q3Vec3 position;         // World coordinate of contact
  float penetration;       // Depth of penetration from collision
  float normalImpulse;     // Accumulated normal impulse
  float tangentImpulse[2]; // Accumulated friction impulse
  float bias;              // Restitution + baumgarte
  float normalMass;        // Normal constraint mass
  float tangentMass[2];    // Tangent constraint mass
  uint8_t warmStarted;     // Used for debug rendering
};
