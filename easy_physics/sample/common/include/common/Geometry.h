#pragma once
#include "common.h"
#include <memory>
#include <vector>

struct MeshBuff {
  MeshBuff(size_t vertexCount, size_t triangleCount)
      : vtx(vertexCount * 3), idx(triangleCount * 3),
        wireIdx(triangleCount * 6) {}

public:
  std::vector<float> vtx;
  std::vector<unsigned short> idx;
  std::vector<unsigned short> wireIdx;
  static std::shared_ptr<MeshBuff>
  Create(const EasyPhysics::EpxConvexMesh &convexMesh);
};

class Geometry {
public:
  std::vector<std::shared_ptr<MeshBuff>> meshes;
};
