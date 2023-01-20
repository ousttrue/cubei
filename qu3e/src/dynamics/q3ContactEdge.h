#pragma once

struct q3ContactEdge {
  class q3Body *other;
  struct q3ContactConstraint *constraint;
  q3ContactEdge *next;
  q3ContactEdge *prev;
};
