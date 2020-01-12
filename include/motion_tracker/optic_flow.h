#ifndef OpticFlow_h
#define OpticFlow_h

#include <cpp-toolkit/primitives_2d.h>

struct OpticFlow
{
  OpticFlow(Point2i start, Point2i end, double dt): start(start), end(end), dt(dt) {}

  Point2i start;
  Point2i end;

  double dt;
};

#endif
