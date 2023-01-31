#pragma once
#include <memory>

struct q3ContactEdge {
  class q3Body *other;
  std::shared_ptr<struct q3ContactConstraint> constraint;
  q3ContactEdge *next;
  q3ContactEdge *prev;
};
