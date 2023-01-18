#include "GL2Renderer.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#endif
#include <gl/GL.h>
#include <gl/GLU.h>

GL2Renderer::GL2Renderer() {
  // Setup all the open-gl states we want to use (ones that don't change in
  // the lifetime of the application) Note: These can be changed anywhere, but
  // generally we don't change the back buffer color
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glEnable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glCullFace(GL_BACK);
  glFrontFace(GL_CCW);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);

  // Used FFP to setup lights
  float floats[4];
  for (int i = 0; i < 4; ++i)
    floats[i] = (float)light_.ambient[i];
  glLightfv(GL_LIGHT0, GL_AMBIENT, floats);
  for (int i = 0; i < 4; ++i)
    floats[i] = (float)light_.diffuse[i];
  glLightfv(GL_LIGHT0, GL_DIFFUSE, floats);
  for (int i = 0; i < 4; ++i)
    floats[i] = (float)light_.specular[i];
  glLightfv(GL_LIGHT0, GL_SPECULAR, floats);
  for (int i = 0; i < 4; ++i)
    floats[i] = (float)camera_.position[i];
  glLightfv(GL_LIGHT0, GL_POSITION, floats);
  glEnable(GL_LIGHT0);
  glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
}

GL2Renderer::~GL2Renderer() {}

void GL2Renderer::SetPenColor(float r, float g, float b, float a) {
  Q3_UNUSED(a);

  glColor3f((float)r, (float)g, (float)b);
}

void GL2Renderer::SetPenPosition(float x, float y, float z) {
  x_ = x, y_ = y, z_ = z;
}

void GL2Renderer::SetScale(float sx, float sy, float sz) {
  glPointSize((float)sx);
  sx_ = sx, sy_ = sy, sz_ = sz;
}

void GL2Renderer::Line(float x, float y, float z) {
  glBegin(GL_LINES);
  glVertex3f((float)x_, (float)y_, (float)z_);
  glVertex3f((float)x, (float)y, (float)z);
  SetPenPosition(x, y, z);
  glEnd();
}

void GL2Renderer::Triangle(float x1, float y1, float z1, float x2, float y2,
                        float z2, float x3, float y3, float z3) {
  glEnable(GL_LIGHTING);
  glBegin(GL_TRIANGLES);
  glNormal3f((float)nx_, (float)ny_, (float)nz_);
  glColor4f(0.2f, 0.4f, 0.7f, 0.5f);
  glVertex3f((float)x1, (float)y1, (float)z1);
  glVertex3f((float)x2, (float)y2, (float)z2);
  glVertex3f((float)x3, (float)y3, (float)z3);
  glEnd();
  glDisable(GL_LIGHTING);
}

void GL2Renderer::SetTriNormal(float x, float y, float z) {
  nx_ = x;
  ny_ = y;
  nz_ = z;
}

void GL2Renderer::Point() {
  glBegin(GL_POINTS);
  glVertex3f((float)x_, (float)y_, (float)z_);
  glEnd();
};

void GL2Renderer::BeginFrame(int width, int height) {
  glViewport(0, 0, width, height);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  if (height <= 0) {
    height = 1;
  }
  float aspectRatio = (float)width / (float)height;
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45.0f, aspectRatio, 0.1f, 10000.0f);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(camera_.position[0], camera_.position[1], camera_.position[2],
            camera_.target[0], camera_.target[1], camera_.target[2], 0.0f, 1.0f,
            0.0f);
}
