#include <motion_tracker/motion_estimation.h>


double azimuthFromImage(const CameraConfig& params, unsigned int u)
{
  return atan2(2*u - params.img_width, params.focal_length);
}

double elevationFromImage(const CameraConfig& params, unsigned int v)
{
  return atan2(2*v - params.img_height, params.focal_length);
}

double getTurnRateFromFlow(const CameraConfig& params, const std::vector<OpticFlow>& flow)
{
  std::vector<double> angular_flow;
  angular_flow.reserve(flow.size());
  double sum = 0;

  for (const auto& flow_element : flow)
  {
    double flow = azimuthFromImage(params, flow_element.anchor.x) -
                  azimuthFromImage(params, flow_element.anchor.x + flow_element.flow.x);
    angular_flow.emplace_back(flow);

    sum += flow;
  }
  std::sort(angular_flow.begin(), angular_flow.end());

  double yaw_speed = angular_flow[angular_flow.size() / 2] / flow.front().dt;
  double yaw_speed_avg = (sum /flow.size()) / flow.front().dt;

  return yaw_speed;
}