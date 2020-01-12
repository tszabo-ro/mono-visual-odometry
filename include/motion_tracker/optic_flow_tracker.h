#ifndef OpticFlowTracker_h
#define OpticFlowTracker_h

#include <motion_tracker/camera_frame.h>
#include <motion_tracker/optic_flow.h>
#include <future>

class OpticFlowTracker
{
public:
  OpticFlowTracker(const Frame& start_frame, Rect<unsigned int> roi, size_t num_points);
  ~OpticFlowTracker();

  [[nodiscard]] std::vector<OpticFlow> calculate(const Frame& frame);
  [[nodiscard]] std::packaged_task<std::vector<OpticFlow>()> packageCalculation(const Frame& frame)
  {
    return std::packaged_task<std::vector<OpticFlow>()>([&]()
    {
      return calculate(frame);
    });
  }

private:
  const Rect<unsigned int> roi;
  const size_t num_tracked_points;

  struct Internal;
  std::unique_ptr<Internal> internal_;

};

#endif
