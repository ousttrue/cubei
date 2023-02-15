#include <common/DrawData.h>
#include <common/Geometry.h>
#include <common/Gl1Renderer.h>
#include <gl/gl.h>
#include <stdexcept>
#include <vector>

using namespace EasyPhysics;

Gl1Renderer::Gl1Renderer() {
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClearDepth(1.0f);
}

Gl1Renderer::~Gl1Renderer() {}

void Gl1Renderer::Begin(int width, int height, const float projection[16],
                        const float view[16]) {
  glViewport(0, 0, width, height);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glFrontFace(GL_CCW);
  glDepthFunc(GL_LESS);
  glCullFace(GL_BACK);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glMultMatrixf(projection);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glMultMatrixf(view);
}

void Gl1Renderer::Render(const DrawData &data, class Geometry &scene) {
  for (auto [wMtx, shape] : data.shapes) {
    auto mesh = scene.meshes[(size_t)shape->userData];
    RenderMesh((const float *)&wMtx, EasyPhysics::EpxVector3(1, 1, 1),
               mesh.get());
  }
  DebugBegin();
  for (auto &point : data.points) {
    DebugPoint(point.position, point.color);
  }
  for (auto &line : data.lines) {
    DebugLine(line.begin, line.end, line.color);
  }
  DebugEnd();
}

void Gl1Renderer::DebugBegin() {
  glDepthMask(GL_FALSE);
  glDisable(GL_DEPTH_TEST);
}

void Gl1Renderer::DebugEnd() {
  glDepthMask(GL_TRUE);
  glEnable(GL_DEPTH_TEST);
}

void Gl1Renderer::RenderMesh(const float transform[16], const EpxVector3 &color,
                             const MeshBuff *buff) {
  // assert(meshId >= 0 && (EpxUInt32)meshId < s_meshBuff.size());
  // MeshBuff &buff = s_meshBuff[meshId];

  glPushMatrix();
  glMultMatrixf(transform);

  glEnableClientState(GL_VERTEX_ARRAY);

  glVertexPointer(3, GL_FLOAT, 0, buff->vtx.data());

  glColor4f(color[0], color[1], color[2], 1.0f);
  glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(1.0f, 1.0f);
  glDrawElements(GL_TRIANGLES, buff->idx.size(), GL_UNSIGNED_SHORT,
                 buff->idx.data());
  glDisable(GL_POLYGON_OFFSET_FILL);

  glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
  glDrawElements(GL_LINES, buff->wireIdx.size(), GL_UNSIGNED_SHORT,
                 buff->wireIdx.data());

  glDisableClientState(GL_VERTEX_ARRAY);

  glPopMatrix();
}

void Gl1Renderer::DebugPoint(const EpxVector3 &position,
                             const EpxVector3 &color) {
  glColor4f(color[0], color[1], color[2], 1.0f);

  glPointSize(5.0f);
  glEnableClientState(GL_VERTEX_ARRAY);
  glVertexPointer(3, GL_FLOAT, 16, (float *)&position);
  glDrawArrays(GL_POINTS, 0, 1);
  glDisableClientState(GL_VERTEX_ARRAY);
  glPointSize(1.0f);
}

void Gl1Renderer::DebugLine(const EpxVector3 &position1,
                            const EpxVector3 &position2,
                            const EpxVector3 &color) {
  glColor4f(color[0], color[1], color[2], 1.0f);

  const EpxVector3 points[2] = {
      position1,
      position2,
  };

  glEnableClientState(GL_VERTEX_ARRAY);
  glVertexPointer(3, GL_FLOAT, 16, (float *)points);
  glDrawArrays(GL_LINES, 0, 2);
  glDisableClientState(GL_VERTEX_ARRAY);
}

void Gl1Renderer::DebugAabb(const EpxVector3 &center, const EpxVector3 &extent,
                            const EpxVector3 &color) {
  const EpxVector3 points[8] = {
      center + mulPerElem(EpxVector3(-1, -1, -1), extent),
      center + mulPerElem(EpxVector3(-1, -1, 1), extent),
      center + mulPerElem(EpxVector3(1, -1, 1), extent),
      center + mulPerElem(EpxVector3(1, -1, -1), extent),
      center + mulPerElem(EpxVector3(-1, 1, -1), extent),
      center + mulPerElem(EpxVector3(-1, 1, 1), extent),
      center + mulPerElem(EpxVector3(1, 1, 1), extent),
      center + mulPerElem(EpxVector3(1, 1, -1), extent),
  };

  const unsigned short indices[] = {
      0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7,
  };

  glColor4f(color[0], color[1], color[2], 1.0f);
  glEnableClientState(GL_VERTEX_ARRAY);
  glVertexPointer(3, GL_FLOAT, 16, (float *)points);
  glDrawElements(GL_LINES, 24, GL_UNSIGNED_SHORT, indices);
  glDisableClientState(GL_VERTEX_ARRAY);
}

void Gl1Renderer::Debug2dBegin(int width, int height) {
  glPushMatrix();
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  EpxMatrix4 proj =
      EpxMatrix4::orthographic(-width * 0.5f, width * 0.5f, -height * 0.5f,
                               height * 0.5f, -10.0f, 10.0f);
  glMultMatrixf((GLfloat *)&proj);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  EpxMatrix4 modelview = EpxMatrix4::translation(EpxVector3(0, 0, -1));
  glMultMatrixf((GLfloat *)&modelview);

  glDisable(GL_DEPTH_TEST);
}

void Gl1Renderer::Debug2dEnd() {
  glEnable(GL_DEPTH_TEST);
  glPopMatrix();
}
