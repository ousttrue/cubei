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
  std::span<
      std::tuple<EasyPhysics::EpxTransform3, const EasyPhysics::EpxShape *>>
      shapes;
  std::span<Point> points;
  std::span<Line> lines;
};
