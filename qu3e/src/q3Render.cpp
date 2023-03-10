#include "q3Render.h"

void q3Render::LineAABB(const q3AABB &b) {
  SetPenPosition(b.min.x, b.max.y, b.min.z);
  Line(b.min.x, b.max.y, b.max.z);
  Line(b.max.x, b.max.y, b.max.z);
  Line(b.max.x, b.max.y, b.min.z);
  Line(b.min.x, b.max.y, b.min.z);

  SetPenPosition(b.min.x, b.min.y, b.min.z);
  Line(b.min.x, b.min.y, b.max.z);
  Line(b.max.x, b.min.y, b.max.z);
  Line(b.max.x, b.min.y, b.min.z);
  Line(b.min.x, b.min.y, b.min.z);

  SetPenPosition(b.min.x, b.min.y, b.min.z);
  Line(b.min.x, b.max.y, b.min.z);
  SetPenPosition(b.max.x, b.min.y, b.min.z);
  Line(b.max.x, b.max.y, b.min.z);
  SetPenPosition(b.max.x, b.min.y, b.max.z);
  Line(b.max.x, b.max.y, b.max.z);
  SetPenPosition(b.min.x, b.min.y, b.max.z);
  Line(b.min.x, b.max.y, b.max.z);
}

const int kBoxIndices[36] = {
    1 - 1, 7 - 1, 5 - 1, 1 - 1, 3 - 1, 7 - 1, 1 - 1, 4 - 1, 3 - 1,
    1 - 1, 2 - 1, 4 - 1, 3 - 1, 8 - 1, 7 - 1, 3 - 1, 4 - 1, 8 - 1,
    5 - 1, 7 - 1, 8 - 1, 5 - 1, 8 - 1, 6 - 1, 1 - 1, 5 - 1, 6 - 1,
    1 - 1, 6 - 1, 2 - 1, 2 - 1, 6 - 1, 8 - 1, 2 - 1, 8 - 1, 4 - 1};

void q3Render::Cube(const q3Transform &world, const q3Vec3 extent) {
  q3Vec3 vertices[8] = {
      {-extent.x, -extent.y, -extent.z}, {-extent.x, -extent.y, extent.z},
      {-extent.x, extent.y, -extent.z},  {-extent.x, extent.y, extent.z},
      {extent.x, -extent.y, -extent.z},  {extent.x, -extent.y, extent.z},
      {extent.x, extent.y, -extent.z},   {extent.x, extent.y, extent.z}};

  for (int i = 0; i < 36; i += 3) {
    q3Vec3 a = world * vertices[kBoxIndices[i]];
    q3Vec3 b = world * vertices[kBoxIndices[i + 1]];
    q3Vec3 c = world * vertices[kBoxIndices[i + 2]];

    q3Vec3 n = q3Cross(b - a, c - a).Normalized();

    // SetPenColor( 0.2f, 0.4f, 0.7f, 0.5f );
    // SetPenPosition( a.x, a.y, a.z );
    // Line( b.x, b.y, b.z );
    // Line( c.x, c.y, c.z );
    // Line( a.x, a.y, a.z );

    SetTriNormal(n.x, n.y, n.z);
    Triangle(a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z);
  }
}
