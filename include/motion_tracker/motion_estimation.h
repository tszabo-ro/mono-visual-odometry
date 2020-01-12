#ifndef MotionEstimator_h
#define MotionEstimator_h

#include <motion_tracker/optic_flow.h>
#include <motion_tracker/camera_config.h>
#include <vector>

double getTurnRateFromFlow(const CameraConfig& params, const std::vector<OpticFlow>& flow);

#endif
