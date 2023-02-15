#include <common/Geometry.h>
using namespace EasyPhysics;

std::shared_ptr<MeshBuff> MeshBuff::Create(const EpxConvexMesh &convexMesh) {

  EpxFloat *verts = new EpxFloat[convexMesh.m_numVertices * 3];
  EpxFloat *nmls = new EpxFloat[convexMesh.m_numVertices * 3];
  EpxUInt16 *idxs = new EpxUInt16[convexMesh.m_numFacets * 3];

  for (EpxUInt32 c = 0; c < convexMesh.m_numVertices; c++) {
    verts[c * 3 + 0] = convexMesh.m_vertices[c][0];
    verts[c * 3 + 1] = convexMesh.m_vertices[c][1];
    verts[c * 3 + 2] = convexMesh.m_vertices[c][2];
  }

  for (EpxUInt32 c = 0; c < convexMesh.m_numVertices; c++) {
    EpxVector3 normal(0.0f);
    int facetCount = 0;
    for (EpxUInt32 f = 0; f < convexMesh.m_numFacets; f++) {
      auto &facet = convexMesh.m_facets[f];
      if (facet.vertId[0] == c || facet.vertId[1] == c ||
          facet.vertId[2] == c) {
        const EpxVector3 &v0 = convexMesh.m_vertices[facet.vertId[0]];
        const EpxVector3 &v1 = convexMesh.m_vertices[facet.vertId[1]];
        const EpxVector3 &v2 = convexMesh.m_vertices[facet.vertId[2]];
        normal += cross(v1 - v0, v2 - v0);
        facetCount++;
      }
    }
    normal = normalize(normal / (EpxFloat)facetCount);

    nmls[c * 3 + 0] = normal[0];
    nmls[c * 3 + 1] = normal[1];
    nmls[c * 3 + 2] = normal[2];
  }

  for (EpxUInt32 c = 0; c < convexMesh.m_numFacets; c++) {
    idxs[c * 3 + 0] = convexMesh.m_facets[c].vertId[0];
    idxs[c * 3 + 1] = convexMesh.m_facets[c].vertId[1];
    idxs[c * 3 + 2] = convexMesh.m_facets[c].vertId[2];
  }

  // int renderMeshId = InitMesh(verts, sizeof(EpxFloat) * 3, nmls,
  //                             sizeof(EpxFloat) * 3, idxs, sizeof(EpxUInt16) *
  //                             3, convexMesh.m_numVertices,
  //                             convexMesh.m_numFacets);

  std::shared_ptr<MeshBuff> ptr(
      new MeshBuff(convexMesh.m_numVertices, convexMesh.m_numFacets));
  for (int i = 0; i < convexMesh.m_numVertices; i++) {
    const float *v = (float *)((uintptr_t)verts + sizeof(EpxFloat) * 3 * i);
    ptr->vtx[i * 3] = v[0];
    ptr->vtx[i * 3 + 1] = v[1];
    ptr->vtx[i * 3 + 2] = v[2];
  }

  for (int i = 0; i < convexMesh.m_numFacets; i++) {
    const unsigned short *idx =
        (unsigned short *)((uintptr_t)idxs + sizeof(EpxUInt16) * 3 * i);
    ptr->idx[i * 3] = idx[0];
    ptr->idx[i * 3 + 1] = idx[1];
    ptr->idx[i * 3 + 2] = idx[2];
    ptr->wireIdx[i * 6] = ptr->idx[i * 3];
    ptr->wireIdx[i * 6 + 1] = ptr->idx[i * 3 + 1];
    ptr->wireIdx[i * 6 + 2] = ptr->idx[i * 3 + 1];
    ptr->wireIdx[i * 6 + 3] = ptr->idx[i * 3 + 2];
    ptr->wireIdx[i * 6 + 4] = ptr->idx[i * 3 + 2];
    ptr->wireIdx[i * 6 + 5] = ptr->idx[i * 3];
  }

  delete[] idxs;
  delete[] nmls;
  delete[] verts;

  return ptr;
}
