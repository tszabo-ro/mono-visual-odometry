#include <motion_tracker/motion_estimation.h>


double getTurnRateFromFlow(const CameraConfig& params, const std::vector<OpticFlow>& flow)
{

  // As per the paper, the points are to be projected to a cylindrical frame and the flow (angular
  //  offset between them) is to be calculated there. The origin of the frame is the focal point of
  //  the camera and the only relevant coordinate for this calculation is the angle between the
  //  camera's principal axis and the projected point. This angle is given by the equation:
  //
  //      sin(angle) = (2*v - V)/f
  //
  // Where v is the x coordinate of the point, V is the total width of the image and f is the
  //  focal length of the camera.


  std::vector<double> angular_flow;
  angular_flow.reserve(flow.size());

  for (const auto& flow_element : flow)
  {
    double sin_start = (2.0*flow_element.start.x - params.img_width) / params.focal_length;
    double sin_end = (2.0*flow_element.end.x - params.img_width) / params.focal_length;
    double angle_start = asin(sin_start);
    double angle_end = asin(sin_end);

    double f = angle_end - angle_start;

    if (!std::isnan(f))
    {
      angular_flow.emplace_back(f);
    }
  }
  std::sort(angular_flow.begin(), angular_flow.end());

  double yaw_speed = angular_flow[angular_flow.size() / 2] / flow.front().dt;
  return yaw_speed;
}