#include <motion_tracker/motion_estimation.h>
#include <algorithm>

std::pair<double, std::vector<bool>> getTurnRateAndValidIndicatorFromFlow(const CameraConfig& params, const std::vector<OpticFlow>& flow)
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

  std::vector<bool> valid_flow;
  valid_flow.reserve(flow.size());

  for (const auto& flow_element : flow)
  {
    double sin_start = (2.0*flow_element.start.x - params.img_width) / params.focal_length;
    double sin_end = (2.0*flow_element.end.x - params.img_width) / params.focal_length;
    double angle_start = asin(sin_start);
    double angle_end = asin(sin_end);

    double f = angle_end - angle_start;
    valid_flow.emplace_back(!std::isnan(f));

    if (valid_flow.back())
    {
      angular_flow.emplace_back(f);
    }
  }
  std::sort(angular_flow.begin(), angular_flow.end());

  if (angular_flow.size() < 2)
  {
    return std::make_pair<double, std::vector<bool>>(0.0, {false});
  }

  double yaw_speed = angular_flow[angular_flow.size() / 2] / flow.front().dt;
  return std::make_pair(yaw_speed, valid_flow);
}

std::pair<double, std::vector<bool>> getSpeedAndValidIndicatorFromFlow(const CameraConfig& params, const std::vector<OpticFlow>& flow, double turn_rate)
{
  // The displacement of the camera can be calculated by projecting the flow onto the ground plane,
  //  and removing the effects of the rotation from the observed flow. As per the paper, the magnitude
  //  of the resulting flow vectors is an indication of the forward translation of the camera.

  std::vector<double> linear_flow;
  linear_flow.reserve(flow.size());

  std::vector<bool> valid_flow;
  valid_flow.reserve(flow.size());

  for (const auto& flow_element : flow)
  {
    double s_elevation = atan2(2.0 * flow_element.start.y - params.img_height, params.focal_length);
    double s_azimuth = atan2(2.0 * flow_element.start.x - params.img_width, params.focal_length);

    double e_elevation = atan2(2.0 * flow_element.end.y - params.img_height, params.focal_length);
    double e_azimuth = atan2(2.0 * flow_element.end.x - params.img_width, params.focal_length);

    double s_depth = params.ground_height * cos(s_elevation) / sin(s_elevation + params.camera_pitch);
    double e_depth = params.ground_height * cos(e_elevation) / sin(e_elevation + params.camera_pitch);

    double s_x = s_depth * tan(s_azimuth);
    double s_y = params.ground_height / tan(s_elevation + params.camera_pitch);

    double e_x = e_depth * tan(e_azimuth);
    double e_y = params.ground_height / tan(e_elevation + params.camera_pitch);

    double dx = e_x - s_x - turn_rate * s_y;
    double dy = e_y - s_y;

    double speed_estimate = hypot(dx, dy);

    valid_flow.emplace_back(!std::isnan(speed_estimate));

    if (valid_flow.back())
    {
      linear_flow.emplace_back(speed_estimate);
    }
  }
  std::sort(linear_flow.begin(), linear_flow.end());

  if (linear_flow.size() < 2)
  {
    return std::make_pair<double, std::vector<bool>>(0.0, {false});
  }
  double linear_speed = linear_flow[linear_flow.size() / 2] / flow.front().dt;
  return std::make_pair(linear_speed, valid_flow);
}

double getTurnRateFromFlow(const CameraConfig& params, const std::vector<OpticFlow>& flow)
{
  auto [turn_rate, valid_indicator] = getTurnRateAndValidIndicatorFromFlow(params, flow);
  return turn_rate;
}

double getSpeedFromFlow(const CameraConfig& params, const std::vector<OpticFlow>& flow, double turn_rate)
{
  auto [displacement, valid_indicator] = getSpeedAndValidIndicatorFromFlow(params, flow, turn_rate);
  return displacement;
}