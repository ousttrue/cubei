#include <common/DrawData.h>
#include <common/Gl1Renderer.h>
#include <common/common.h>
#include <elements/geometry_data.h>
#include <gl/gl.h>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <vector>

using namespace EasyPhysics;

struct MeshBuff {

  std::vector<float> vtx;
  std::vector<unsigned short> idx;
  std::vector<unsigned short> wireIdx;
  static std::shared_ptr<MeshBuff> Create(const EpxFloat *vertices,
                                          EpxUInt32 numVertices,
                                          const EpxUInt16 *indices,
                                          EpxUInt32 numIndices) {
    std::shared_ptr<MeshBuff> ptr(new MeshBuff);
    ptr->vtx.assign(vertices, vertices + numVertices * 3);
    ptr->idx.assign(indices, indices + numIndices);
    for (int i = 0; i < numIndices; i += 3) {
      ptr->wireIdx.push_back(indices[i]);
      ptr->wireIdx.push_back(indices[i + 1]);
      ptr->wireIdx.push_back(indices[i + 1]);
      ptr->wireIdx.push_back(indices[i + 2]);
      ptr->wireIdx.push_back(indices[i + 2]);
      ptr->wireIdx.push_back(indices[i]);
    }
    return ptr;
  }
  static std::shared_ptr<MeshBuff> GetOrCreateMesh(EpxShapeType shapeType) {
    static std::unordered_map<EpxShapeType, std::shared_ptr<MeshBuff>>
        m_meshMap;
    auto found = m_meshMap.find(shapeType);
    if (found != m_meshMap.end()) {
      return found->second;
    }

    const EpxFloat *vertices = nullptr;
    EpxUInt32 numVertices = 0;
    const EpxUInt16 *indices = nullptr;
    EpxUInt32 numIndices = 0;
    switch (shapeType) {
    case EpxShapeType::Box:
      vertices = box_vertices;
      numVertices = box_numVertices;
      indices = box_indices;
      numIndices = box_numIndices;
      break;
    case EpxShapeType::Cylinder:
      vertices = cylinder_vertices;
      numVertices = cylinder_numVertices;
      indices = cylinder_indices;
      numIndices = cylinder_numIndices;
      break;
    case EpxShapeType::Sphere:
      vertices = sphere_vertices;
      numVertices = sphere_numVertices;
      indices = sphere_indices;
      numIndices = sphere_numIndices;
      break;
    case EpxShapeType::Tetrahedron:
      vertices = tetrahedron_vertices;
      numVertices = tetrahedron_numVertices;
      indices = tetrahedron_indices;
      numIndices = tetrahedron_numIndices;
      break;
    }
    auto mesh = MeshBuff::Create(vertices, numVertices, indices, numIndices);
    m_meshMap.insert({shapeType, mesh});
    return mesh;
  }
};

//
// Gl1Renderer
//
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

void Gl1Renderer::Render(const DrawDataSpan &data) {
  for (auto transform : data.boxes) {
    RenderShape(EpxShapeType::Box, transform, EasyPhysics::EpxVector3(1, 1, 1));
  }
  for (auto transform : data.spheres) {
    RenderShape(EpxShapeType::Sphere, transform,
                EasyPhysics::EpxVector3(1, 1, 1));
  }
  for (auto transform : data.cylinders) {
    RenderShape(EpxShapeType::Cylinder, transform,
                EasyPhysics::EpxVector3(1, 1, 1));
  }
  for (auto transform : data.tetrahedrons) {
    RenderShape(EpxShapeType::Tetrahedron, transform,
                EasyPhysics::EpxVector3(1, 1, 1));
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

void Gl1Renderer::RenderShape(EasyPhysics::EpxShapeType type,
                              const EasyPhysics::EpxMatrix4 &transform,
                              const EasyPhysics::EpxVector3 &color) {
  std::shared_ptr<MeshBuff> buf;
  switch (type) {
  case EasyPhysics::EpxShapeType::Box:
    buf = MeshBuff::GetOrCreateMesh(EpxShapeType::Box);
    break;
  case EasyPhysics::EpxShapeType::Sphere:
    buf = MeshBuff::GetOrCreateMesh(EpxShapeType::Sphere);
    break;
  case EasyPhysics::EpxShapeType::Cylinder:
    buf = MeshBuff::GetOrCreateMesh(EpxShapeType::Cylinder);
    break;
  case EasyPhysics::EpxShapeType::Tetrahedron:
    buf = MeshBuff::GetOrCreateMesh(EpxShapeType::Tetrahedron);
    break;
  }

  RenderMesh(transform, color, buf->vtx.data(), buf->idx.data(),
             buf->idx.size(), buf->wireIdx.data(), buf->wireIdx.size());
}

void Gl1Renderer::RenderMesh(const EpxMatrix4 &transform,
                             const EpxVector3 &color, const float *v,
                             const uint16_t *i, uint32_t indexCount,
                             const uint16_t *w, uint32_t wireCount) {
  // auto buff = GetOrCreateMesh(type);

  glPushMatrix();
  glMultMatrixf((const float *)&transform);

  glEnableClientState(GL_VERTEX_ARRAY);
  glVertexPointer(3, GL_FLOAT, 0, v);

  glColor4f(color[0], color[1], color[2], 1.0f);
  glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(1.0f, 1.0f);
  glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_SHORT, i);
  glDisable(GL_POLYGON_OFFSET_FILL);

  glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
  glDrawElements(GL_LINES, wireCount, GL_UNSIGNED_SHORT, w);

  glDisableClientState(GL_VERTEX_ARRAY);

  glPopMatrix();
}

void Gl1Renderer::DebugBegin() {
  glDepthMask(GL_FALSE);
  glDisable(GL_DEPTH_TEST);
}

void Gl1Renderer::DebugEnd() {
  glDepthMask(GL_TRUE);
  glEnable(GL_DEPTH_TEST);
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
