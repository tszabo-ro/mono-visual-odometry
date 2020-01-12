#ifndef MotionEstimator_h
#define MotionEstimator_h

#include <motion_tracker/optic_flow.h>
#include <motion_tracker/camera_config.h>
#include <vector>
#include <tuple>

std::pair<double, std::vector<bool>> getTurnRateAndValidIndicatorFromFlow(const CameraConfig& params, const std::vector<OpticFlow>& flow);
double getTurnRateFromFlow(const CameraConfig& params, const std::vector<OpticFlow>& flow);

std::pair<double, std::vector<bool>> getSpeedAndValidIndicatorFromFlow(const CameraConfig& params, const std::vector<OpticFlow>& flow, double turn_rate);
double getSpeedFromFlow(const CameraConfig& params, const std::vector<OpticFlow>& flow, double turn_rate);
#endif
