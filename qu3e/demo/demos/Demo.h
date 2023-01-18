//--------------------------------------------------------------------------------------------------
/**
@file	Demo.h

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

#pragma once

// Base class for running demos to show off q3

struct Demo {
  virtual ~Demo() {}

  virtual void Init(class q3Scene *scene){};
  virtual void Update(class q3Scene *scene, float dt){};
  virtual void Shutdown(class q3Scene *scene){};

  virtual void Render(class q3Render *debugDrawer) { (void)debugDrawer; }
  virtual void KeyDown(unsigned char key) { (void)key; }
  virtual void KeyUp(unsigned char key) { (void)key; }
  virtual void LeftClick(int x, int y) {
    (void)x;
    (void)y;
  }
};

void DemosMouseLeftDown(int x, int y);
void DemosKeyDown(unsigned char key);
void DemosKeyUp(unsigned char key);
void DemosTogglePause();
void DemosSingleStep();
void DemosUpdate();
void DemosRender(int w, int h);
void DemosGui();