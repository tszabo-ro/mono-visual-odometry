#ifndef OpticFlow_h
#define OpticFlow_h

struct OpticFlow
{
  OpticFlow(Point2f anchor, Vector2f flow): anchor(anchor), flow(flow) {}

  Point2f anchor;
  Vector2f flow;
};

#endif
