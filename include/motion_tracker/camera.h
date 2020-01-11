#ifndef CameraWrapper_h
#define CameraWrapper_h

#include <memory>
#include <optional>
#include <motion_tracker/camera_frame.h>

class Camera
{
public:
  Camera(double camera_angle);
  Camera(double camera_angle, const std::string& video_src);
  ~Camera();

  std::optional<Frame> grab();

private:
  struct Internals;

  std::unique_ptr<Internals> internals_;
};

#endif
