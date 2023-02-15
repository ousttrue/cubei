#pragma once
#include <common/common.h>
#include <vector>

struct Point {
  EasyPhysics::EpxVector3 position;
  EasyPhysics::EpxVector3 color;
};

struct Line {
  EasyPhysics::EpxVector3 begin;
  EasyPhysics::EpxVector3 end;
  EasyPhysics::EpxVector3 color;
};

struct DrawData {
  std::vector<
      std::tuple<EasyPhysics::EpxMatrix4, const EasyPhysics::EpxShape *>>
      shapes;
  std::vector<Point> points;
  std::vector<Line> lines;
};
