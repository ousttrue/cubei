#pragma once
#include <q3.h>

#include <Windows.h>
#include <gl/GL.h>

class Renderer : public q3Render {
public:
  void SetPenColor(float r, float g, float b, float a = 1.0f) override {
    Q3_UNUSED(a);

    glColor3f((float)r, (float)g, (float)b);
  }

  void SetPenPosition(float x, float y, float z) override {
    x_ = x, y_ = y, z_ = z;
  }

  void SetScale(float sx, float sy, float sz) override {
    glPointSize((float)sx);
    sx_ = sx, sy_ = sy, sz_ = sz;
  }

  void Line(float x, float y, float z) override {
    glBegin(GL_LINES);
    glVertex3f((float)x_, (float)y_, (float)z_);
    glVertex3f((float)x, (float)y, (float)z);
    SetPenPosition(x, y, z);
    glEnd();
  }

  void Triangle(float x1, float y1, float z1, float x2, float y2, float z2,
                float x3, float y3, float z3) override {
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

  void SetTriNormal(float x, float y, float z) override {
    nx_ = x;
    ny_ = y;
    nz_ = z;
  }

  void Point() override {
    glBegin(GL_POINTS);
    glVertex3f((float)x_, (float)y_, (float)z_);
    glEnd();
  };

private:
  float x_, y_, z_;
  float sx_, sy_, sz_;
  float nx_, ny_, nz_;
};
