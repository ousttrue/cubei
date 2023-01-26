#pragma once
#include "q3Vec3.h"

struct q3AABB {
  q3Vec3 min;
  q3Vec3 max;

  bool Contains(const q3AABB &other) const {
    return min.x <= other.min.x && min.y <= other.min.y &&
           min.z <= other.min.z && max.x >= other.max.x &&
           max.y >= other.max.y && max.z >= other.max.z;
  }
  bool Contains(const q3Vec3 &point) const {
    return min.x <= point.x && min.y <= point.y && min.z <= point.z &&
           max.x >= point.x && max.y >= point.y && max.z >= point.z;
  }
  float SurfaceArea() const {
    float x = max.x - min.x;
    float y = max.y - min.y;
    float z = max.z - min.z;
    return float(2.0) * (x * y + x * z + y * z);
  }

  bool IsOverlapped(const q3AABB &b) const {
    if (max.x < b.min.x || min.x > b.max.x)
      return false;

    if (max.y < b.min.y || min.y > b.max.y)
      return false;

    if (max.z < b.min.z || min.z > b.max.z)
      return false;

    return true;
  }
  q3AABB Combine(const q3AABB &b) const {
    return {
        .min =
            {
                std::min(min.x, b.min.x),
                std::min(min.y, b.min.y),
                std::min(min.z, b.min.z),
            },
        .max =
            {
                std::max(max.x, b.max.x),
                std::max(max.y, b.max.y),
                std::max(max.z, b.max.z),
            },
    };
  }
};
