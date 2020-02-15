#ifndef CameraCalibration_h
#define CameraCalibration_h

#include <nlohmann/json_fwd.hpp>
#include <opencv2/core/mat.hpp>

struct CameraCalibration
{
  CameraCalibration();
  CameraCalibration(const std::string& file_name);

  [[nodiscard]] nlohmann::json dump() const;
  [[nodiscard]] bool valid() const { return (camera_matrix.rows > 0 && camera_matrix.cols > 0 && dist_coeffs.rows > 0 && dist_coeffs.cols > 0); }

  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;

  double total_error;
};

#endif
