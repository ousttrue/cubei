#include "Renderer.h"

Renderer::Renderer() {}

Renderer::~Renderer() {}

void Renderer::SetPenColor(float r, float g, float b, float a) {
  Q3_UNUSED(a);

  glColor3f((float)r, (float)g, (float)b);
}

void Renderer::SetPenPosition(float x, float y, float z) {
  x_ = x, y_ = y, z_ = z;
}

void Renderer::SetScale(float sx, float sy, float sz) {
  glPointSize((float)sx);
  sx_ = sx, sy_ = sy, sz_ = sz;
}

void Renderer::Line(float x, float y, float z) {
  glBegin(GL_LINES);
  glVertex3f((float)x_, (float)y_, (float)z_);
  glVertex3f((float)x, (float)y, (float)z);
  SetPenPosition(x, y, z);
  glEnd();
}

void Renderer::Triangle(float x1, float y1, float z1, float x2, float y2,
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

void Renderer::SetTriNormal(float x, float y, float z) {
  nx_ = x;
  ny_ = y;
  nz_ = z;
}

void Renderer::Point() {
  glBegin(GL_POINTS);
  glVertex3f((float)x_, (float)y_, (float)z_);
  glEnd();
};
