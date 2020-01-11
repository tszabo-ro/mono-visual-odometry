#ifndef CameraWrapper_h
#define CameraWrapper_h

#include <memory>
#include <optional>
#include <motion_tracker/camera_frame.h>

struct CameraConfig
{
  double v_fov;
  double h_fov;

  double roll;
  double pitch;

  double ground_height;

  size_t img_width;
  size_t img_height;
};

class Camera
{
public:
  Camera(CameraConfig config);
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
