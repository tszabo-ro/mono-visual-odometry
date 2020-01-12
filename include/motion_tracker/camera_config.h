#ifndef CameraConfig_h
#define CameraConfig_h

#include <cmath>

struct CameraConfig
{
  CameraConfig(double v_fov, double h_fov, size_t img_width, size_t img_height, double camera_roll, double camera_pitch, double ground_height)
  : v_fov(v_fov)
  , h_fov(h_fov)
  , img_width(img_width)
  , img_height(img_height)
  , focal_length(calculateFocalLength(v_fov, h_fov, img_width, img_height))
  , camera_roll(camera_roll)
  , camera_pitch(camera_pitch)
  , ground_height(ground_height)
  {
  }
  double v_fov;         // [rad]
  double h_fov;         // [rad]

  size_t img_width;     // [-]
  size_t img_height;    // [-]

  double focal_length;  // [#pixels]

  double camera_roll;   // [rad]
  double camera_pitch;  // [rad]

  double ground_height; // [m]

private:
  static double calculateFocalLength(double v_fov, double h_fov, size_t img_width, size_t img_height)
  {
    double f_vertical   = img_height / (2 * tan(v_fov/2));
    double f_horizontal = img_width  / (2 * tan(h_fov/2));

    if (std::abs(f_vertical/f_horizontal - 1) > 0.05)
    {
      printf("WARNING: Inconsistent camera config! hFoV/vFoV: %.1f/%.1f [deg], image width/height %zu/%zu [-] results in focal lengths: %.3f/%.3f [px unit], which is %.1f%% off!\n",
        h_fov*180/M_PI, v_fov*180/M_PI, img_width, img_height, f_horizontal, f_vertical, std::abs(f_vertical/f_horizontal - 1)*100);
    }

    return 0.5*(f_horizontal + f_vertical);
  }
};

#endif
