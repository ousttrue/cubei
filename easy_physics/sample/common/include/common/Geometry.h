#pragma once
#include "common.h"
#include <vector>

struct MeshBuff {
  float *vtx;
  float *nml;
  int numVtx;
  unsigned short *idx;
  unsigned short *wireIdx;
  int numIdx;

  MeshBuff() : vtx(0), nml(0), numVtx(0), idx(0), wireIdx(0), numIdx(0) {}
};

class Geometry {
  std::vector<MeshBuff> s_meshBuff;

public:
  Geometry(const Geometry &) = delete;
  Geometry &operator=(const Geometry &) = delete;
  Geometry() {}
  ~Geometry() {}

  uint64_t CreateRenderMesh(EasyPhysics::EpxConvexMesh *convexMesh);
  void ReleaseMeshAll();
  const MeshBuff &Get(uint64_t index) const { return s_meshBuff[index]; }

private:
  int InitMesh(const float *vtx, unsigned int vtxStrideBytes, const float *nml,
               unsigned int nmlStrideBytes, const unsigned short *tri,
               unsigned int triStrideBytes, int numVtx, int numTri);
};
