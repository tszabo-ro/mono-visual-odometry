#ifndef OpticFlow_h
#define OpticFlow_h

struct OpticFlow
{
  OpticFlow(Point2i anchor, Vector2i flow): anchor(anchor), flow(flow) {}

  Point2i anchor;
  Vector2i flow;
};

#endif
