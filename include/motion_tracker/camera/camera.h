#ifndef CameraWrapper_h
#define CameraWrapper_h

#include <memory>
#include <optional>
#include <motion_tracker/camera/camera_frame.h>
#include <motion_tracker/camera/camera_config.h>
#include <motion_tracker/camera/camera_calibration.h>

class Camera
{
public:
  Camera(CameraConfig config, CameraCalibration calibration);
  Camera(CameraConfig config, CameraCalibration calibration, int camera_id);
  Camera(CameraConfig config, CameraCalibration calibration, const std::string& video_src);
  ~Camera();

  std::optional<Frame> grab();
  const CameraConfig& config() const { return config_; }

private:
  struct Internals;

  CameraConfig config_;
  CameraCalibration calibration_;

  std::unique_ptr<Internals> internals_;
};

#endif
