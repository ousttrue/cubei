/*
        Copyright (c) 2012 Hiroshi Matsuike

        This software is provided 'as-is', without any express or implied
        warranty. In no event will the authors be held liable for any damages
        arising from the use of this software.

        Permission is granted to anyone to use this software for any purpose,
        including commercial applications, and to alter it and redistribute it
        freely, subject to the following restrictions:

        1. The origin of this software must not be misrepresented; you must not
        claim that you wrote the original software. If you use this software
        in a product, an acknowledgment in the product documentation would be
        appreciated but is not required.

        2. Altered source versions must be plainly marked as such, and must not
   be misrepresented as being the original software.

        3. This notice may not be removed or altered from any source
   distribution.
*/

#include <common/Gl1Renderer.h>
#include <gl/gl.h>
#include <stdexcept>
#include <vector>

// using namespace std;
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

void Gl1Renderer::DebugBegin() {
  glDepthMask(GL_FALSE);
  glDisable(GL_DEPTH_TEST);
}

void Gl1Renderer::DebugEnd() {
  glDepthMask(GL_TRUE);
  glEnable(GL_DEPTH_TEST);
}

int MeshScene::InitMesh(const float *vtx, unsigned int vtxStrideBytes,
                        const float *nml, unsigned int nmlStrideBytes,
                        const unsigned short *tri, unsigned int triStrideBytes,
                        int numVtx, int numTri) {
  //	assert(numMesh<MAX_MESH);

  MeshBuff buff;
  buff.vtx = new float[3 * numVtx];
  buff.nml = new float[3 * numVtx];
  buff.idx = new unsigned short[numTri * 3];
  buff.wireIdx = new unsigned short[numTri * 6];
  buff.numIdx = numTri * 3;
  buff.numVtx = numVtx;

  for (int i = 0; i < numVtx; i++) {
    const float *v = (float *)((uintptr_t)vtx + vtxStrideBytes * i);
    const float *n = (float *)((uintptr_t)nml + nmlStrideBytes * i);
    buff.vtx[i * 3] = v[0];
    buff.vtx[i * 3 + 1] = v[1];
    buff.vtx[i * 3 + 2] = v[2];
    buff.nml[i * 3] = n[0];
    buff.nml[i * 3 + 1] = n[1];
    buff.nml[i * 3 + 2] = n[2];
  }

  for (int i = 0; i < numTri; i++) {
    const unsigned short *idx =
        (unsigned short *)((uintptr_t)tri + triStrideBytes * i);
    buff.idx[i * 3] = idx[0];
    buff.idx[i * 3 + 1] = idx[1];
    buff.idx[i * 3 + 2] = idx[2];
    buff.wireIdx[i * 6] = buff.idx[i * 3];
    buff.wireIdx[i * 6 + 1] = buff.idx[i * 3 + 1];
    buff.wireIdx[i * 6 + 2] = buff.idx[i * 3 + 1];
    buff.wireIdx[i * 6 + 3] = buff.idx[i * 3 + 2];
    buff.wireIdx[i * 6 + 4] = buff.idx[i * 3 + 2];
    buff.wireIdx[i * 6 + 5] = buff.idx[i * 3];
  }

  s_meshBuff.push_back(buff);
  return s_meshBuff.size() - 1;
}
void MeshScene::ReleaseMeshAll() {
  for (EpxUInt32 c = 0; c < s_meshBuff.size(); ++c) {
    delete[] s_meshBuff[c].vtx;
    delete[] s_meshBuff[c].nml;
    delete[] s_meshBuff[c].idx;
    delete[] s_meshBuff[c].wireIdx;
  }
  s_meshBuff.clear();
}

void Gl1Renderer::RenderMesh(const float transform[16], const EpxVector3 &color,
                             const MeshBuff &buff) {
  // assert(meshId >= 0 && (EpxUInt32)meshId < s_meshBuff.size());
  // MeshBuff &buff = s_meshBuff[meshId];

  glPushMatrix();
  glMultMatrixf(transform);

  glEnableClientState(GL_VERTEX_ARRAY);

  glVertexPointer(3, GL_FLOAT, 0, buff.vtx);

  glColor4f(color[0], color[1], color[2], 1.0f);
  glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(1.0f, 1.0f);
  glDrawElements(GL_TRIANGLES, buff.numIdx, GL_UNSIGNED_SHORT, buff.idx);
  glDisable(GL_POLYGON_OFFSET_FILL);

  glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
  glDrawElements(GL_LINES, buff.numIdx * 2, GL_UNSIGNED_SHORT, buff.wireIdx);

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

uint64_t MeshScene::CreateRenderMesh(EpxConvexMesh *convexMesh) {
  assert(convexMesh);

  EpxFloat *verts = new EpxFloat[convexMesh->m_numVertices * 3];
  EpxFloat *nmls = new EpxFloat[convexMesh->m_numVertices * 3];
  EpxUInt16 *idxs = new EpxUInt16[convexMesh->m_numFacets * 3];

  for (EpxUInt32 c = 0; c < convexMesh->m_numVertices; c++) {
    verts[c * 3 + 0] = convexMesh->m_vertices[c][0];
    verts[c * 3 + 1] = convexMesh->m_vertices[c][1];
    verts[c * 3 + 2] = convexMesh->m_vertices[c][2];
  }

  for (EpxUInt32 c = 0; c < convexMesh->m_numVertices; c++) {
    EpxVector3 normal(0.0f);
    int facetCount = 0;
    for (EpxUInt32 f = 0; f < convexMesh->m_numFacets; f++) {
      EpxFacet &facet = convexMesh->m_facets[f];
      if (facet.vertId[0] == c || facet.vertId[1] == c ||
          facet.vertId[2] == c) {
        const EpxVector3 &v0 = convexMesh->m_vertices[facet.vertId[0]];
        const EpxVector3 &v1 = convexMesh->m_vertices[facet.vertId[1]];
        const EpxVector3 &v2 = convexMesh->m_vertices[facet.vertId[2]];
        normal += cross(v1 - v0, v2 - v0);
        facetCount++;
      }
    }
    normal = normalize(normal / (EpxFloat)facetCount);

    nmls[c * 3 + 0] = normal[0];
    nmls[c * 3 + 1] = normal[1];
    nmls[c * 3 + 2] = normal[2];
  }

  for (EpxUInt32 c = 0; c < convexMesh->m_numFacets; c++) {
    idxs[c * 3 + 0] = convexMesh->m_facets[c].vertId[0];
    idxs[c * 3 + 1] = convexMesh->m_facets[c].vertId[1];
    idxs[c * 3 + 2] = convexMesh->m_facets[c].vertId[2];
  }

  int renderMeshId =
      InitMesh(verts, sizeof(EpxFloat) * 3, nmls, sizeof(EpxFloat) * 3, idxs,
               sizeof(EpxUInt16) * 3, convexMesh->m_numVertices,
               convexMesh->m_numFacets);

  delete[] idxs;
  delete[] nmls;
  delete[] verts;

  return renderMeshId;
}
