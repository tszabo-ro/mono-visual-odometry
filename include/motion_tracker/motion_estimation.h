#ifndef MotionEstimator_h
#define MotionEstimator_h

#include <motion_tracker/optic_flow.h>
#include <motion_tracker/camera_config.h>
#include <vector>

double azimuthFromImage(const CameraConfig& params, unsigned int u);
double elevationFromImage(const CameraConfig& params, unsigned int v);

double getTurnRateFromFlow(const CameraConfig& params, const std::vector<OpticFlow>& flow);

#endif
