#ifndef OpticFlow_h
#define OpticFlow_h

#include <cpp-toolkit/primitives_2d.h>

struct OpticFlow
{
  OpticFlow(Point2i anchor, Vector2i flow, double dt): anchor(anchor), flow(flow), dt(dt) {}

  Point2i anchor;
  Vector2i flow;

  double dt;
};

#endif
