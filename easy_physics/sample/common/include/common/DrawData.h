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

struct DrawDataSpan {
  std::span<const EasyPhysics::EpxMatrix4> boxes;
  std::span<const EasyPhysics::EpxMatrix4> spheres;
  std::span<const EasyPhysics::EpxMatrix4> cylinders;
  std::span<const EasyPhysics::EpxMatrix4> tetrahedrons;
  std::span<const Point> points;
  std::span<const Line> lines;
};

struct DrawData {
  std::vector<EasyPhysics::EpxMatrix4> boxes;
  std::vector<EasyPhysics::EpxMatrix4> spheres;
  std::vector<EasyPhysics::EpxMatrix4> cylinders;
  std::vector<EasyPhysics::EpxMatrix4> tetrahedrons;
  std::vector<Point> points;
  std::vector<Line> lines;
  void Clear() {
    boxes.clear();
    spheres.clear();
    cylinders.clear();
    tetrahedrons.clear();
    points.clear();
    lines.clear();
  }
  void Push(EasyPhysics::EpxShapeType shapeType,
            const EasyPhysics::EpxMatrix4 &m) {
    switch (shapeType) {
    case EasyPhysics::EpxShapeType::Box:
      boxes.push_back(m);
      break;
    case EasyPhysics::EpxShapeType::Sphere:
      spheres.push_back(m);
      break;
    case EasyPhysics::EpxShapeType::Cylinder:
      cylinders.push_back(m);
      break;
    case EasyPhysics::EpxShapeType::Tetrahedron:
      tetrahedrons.push_back(m);
      break;
    }
  }
  void PushPoint(const Point &p) { points.push_back(p); }
  void PushLine(const Line &l) { lines.push_back(l); }
  DrawDataSpan Span() const {
    return {
        boxes, spheres, cylinders, tetrahedrons, points, lines,
    };
  }
};
