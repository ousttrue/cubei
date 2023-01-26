//--------------------------------------------------------------------------------------------------
/**
@file	q3.h

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

#ifndef Q3_H
#define Q3_H

#include "dynamics/q3BroadPhase.h"
#include "dynamics/q3Contact.h"
#include "dynamics/q3ContactConstraint.h"
#include "dynamics/q3ContactManager.h"
#include "dynamics/q3TimeStep.h"
#include "math/q3Mat3.h"
#include "math/q3Math.h"
#include "math/q3Quaternion.h"
#include "math/q3Transform.h"
#include "math/q3Vec3.h"
#include "q3Render.h"
#include "scene/q3Body.h"
#include "scene/q3Box.h"
#include "scene/q3Env.h"
#include "scene/q3Scene.h"

inline void q3RenderScene(q3Render *renderer, const class q3Scene *scene) {
  for (auto body : *scene) {
    body->Render(renderer);
  }
}

#endif // Q3_H
