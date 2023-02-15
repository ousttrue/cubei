#include <common/Geometry.h>
using namespace EasyPhysics;

int Geometry::InitMesh(const float *vtx, unsigned int vtxStrideBytes,
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

void Geometry::ReleaseMeshAll() {
  for (EpxUInt32 c = 0; c < s_meshBuff.size(); ++c) {
    delete[] s_meshBuff[c].vtx;
    delete[] s_meshBuff[c].nml;
    delete[] s_meshBuff[c].idx;
    delete[] s_meshBuff[c].wireIdx;
  }
  s_meshBuff.clear();
}
