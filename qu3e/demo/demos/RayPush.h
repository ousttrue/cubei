//--------------------------------------------------------------------------------------------------
/**
@file	DropBoxes.h

@author	Randy Gaul
@date	11/25/2014

        Copyright (c) 2014 Randy Gaul http://www.randygaul.net

        This software is provided 'as-is', without any express or implied
        warranty. In no event will the authors be held liable for any damages
        arising from the use of this software.

        Permission is granted to anyone to use this software for any purpose,
        including commercial applications, and to alter it and redistribute it
        freely, subject to the following restrictions:
                1. The origin of this software must not be misrepresented; you
must not claim that you wrote the original software. If you use this software in
a product, an acknowledgment in the product documentation would be appreciated
but is not required.
                2. Altered source versions must be plainly marked as such, and
must not be misrepresented as being the original software.
                3. This notice may not be removed or altered from any source
distribution.
*/
//--------------------------------------------------------------------------------------------------
#include "Demo.h"
#include <chrono>
#include <q3.h>
class Raycast {
public:
  q3RaycastData data;
  float tfinal;
  q3Vec3 nfinal;
  q3Body *impactBody;

  bool ReportShape(q3Body *body, q3Box *shape) {
    if (data.toi < tfinal) {
      tfinal = data.toi;
      nfinal = data.normal;
      impactBody = body;
    }

    data.toi = tfinal;
    return true;
  }

  void Init(const q3Vec3 &spot, const q3Vec3 &dir) {
    data.start = spot;
    data.dir = q3Normalize(dir);
    data.t = float(10000.0);
    tfinal = FLT_MAX;
    data.toi = data.t;
    impactBody = NULL;
  }
};

struct RayPush : public Demo {
  Raycast rayCast;
  std::chrono::nanoseconds acc;
  void Init(q3Scene *scene) override;
  void Update(q3Scene *scene, std::chrono::nanoseconds dt,
              class q3BroadPhase *broadphase,
              class q3ContactManager *contactManager) override;
  void Shutdown(q3Scene *scene) override { scene->RemoveAllBodies(); }
  void Render(q3Render *render) override;
};
