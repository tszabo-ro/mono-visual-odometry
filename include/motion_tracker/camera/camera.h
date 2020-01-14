#ifndef CameraWrapper_h
#define CameraWrapper_h

#include <memory>
#include <optional>
#include <motion_tracker/camera/camera_frame.h>
#include <motion_tracker/camera/camera_config.h>

class Camera
{
public:
  Camera(CameraConfig config);
  Camera(CameraConfig config, int camera_id);
  Camera(CameraConfig config, const std::string& video_src);
  ~Camera();

  std::optional<Frame> grab();
  const CameraConfig& config() const { return config_; }

private:
  struct Internals;

  CameraConfig config_;

  std::unique_ptr<Internals> internals_;
};

#endif
