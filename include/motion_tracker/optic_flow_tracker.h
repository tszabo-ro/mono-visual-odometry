#ifndef OpticFlowTracker_h
#define OpticFlowTracker_h

#include <motion_tracker/camera_frame.h>

struct Projection
{

};

struct OpticFlow
{
  OpticFlow(Point2f anchor, Vector2f flow): anchor(anchor), flow(flow) {}

  Point2f anchor;
  Vector2f flow;
};

class OpticFlowTracker
{
public:
  OpticFlowTracker(const Frame& start_frame, Rect<unsigned int> roi, size_t num_points);
  ~OpticFlowTracker();

  [[nodiscard]] std::vector<OpticFlow> getFlowVectors(const Frame& frame, Projection proj = {});
  [[nodiscard]] std::vector<Point2f> getTrackedPoints() const;

private:
  const Rect<unsigned int> roi;
  const size_t num_tracked_points;

  struct Internal;
  std::unique_ptr<Internal> internal_;

};

#endif
