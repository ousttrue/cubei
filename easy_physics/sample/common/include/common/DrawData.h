#pragma once
#include <common/common.h>
#include <vector>

struct Shape {
  EasyPhysics::EpxShapeType type;
  EasyPhysics::EpxMatrix4 transform;
};

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
  std::span<Shape> shapes;
  std::span<Point> points;
  std::span<Line> lines;
};
