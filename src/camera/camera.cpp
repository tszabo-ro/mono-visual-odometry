#include <motion_tracker/camera/camera.h>

#include <opencv2/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tuple>
#include <opencv2/calib3d.hpp>

template<class T>
static std::tuple<std::unique_ptr<cv::VideoCapture>, cv::Mat, cv::Rect2f> getCameraParams(double cam_angle, T device_params)
{
  auto capture_device = std::make_unique<cv::VideoCapture>(device_params, cv::CAP_ANY);

  cv::Mat frame;
  capture_device->grab();
  capture_device->retrieve(frame);

  cv::Point2f center((frame.cols - 1) / 2.0, (frame.rows - 1) / 2.0);
  cv::Rect2f result_frame_size = cv::RotatedRect(cv::Point2f(), frame.size(), cam_angle).boundingRect2f();

  cv::Mat rot = cv::getRotationMatrix2D(center, cam_angle, 1.0);
  rot.at<double>(0, 2) += result_frame_size.width / 2.0 - frame.cols / 2.0;
  rot.at<double>(1, 2) += result_frame_size.height / 2.0 - frame.rows / 2.0;

  result_frame_size.x = 2*std::abs(result_frame_size.x);
  result_frame_size.y = 2*std::abs(result_frame_size.y);

  printf("Cam angle: %.3f Rotated size: %.3f/%.3f\n", cam_angle, result_frame_size.x, result_frame_size.y);

  return std::tuple<std::unique_ptr<cv::VideoCapture>, cv::Mat, cv::Rect2f>(std::move(capture_device), std::move(rot), std::move(result_frame_size));
}

struct Camera::Internals
{
  Internals(std::tuple<std::unique_ptr<cv::VideoCapture>, cv::Mat, cv::Rect2f> params)
    : capture_device(std::move(std::get<0>(params)))
      , rotation_matrix(std::move(std::get<1>(params)))
      , rotated_size(std::move(std::get<2>(params)))
  {}

  Internals(std::unique_ptr<cv::VideoCapture> cap, const cv::Mat& rotation_matrix, const cv::Rect2f& rotated_size)
    : capture_device(std::move(cap))
      , rotation_matrix(rotation_matrix)
      , rotated_size(rotated_size)
  {}

  const std::unique_ptr<cv::VideoCapture> capture_device;
  const cv::Mat rotation_matrix;
  const cv::Rect2f rotated_size;
};

Camera::Camera(CameraConfig config, CameraCalibration calibration)
  : config_(config)
  , calibration_(calibration)
  , internals_(std::make_unique<Internals>(getCameraParams(config.camera_roll*180/M_PI, 0)))
{
  config_ = CameraConfig(config.v_fov, config.h_fov, internals_->rotated_size.x, internals_->rotated_size.y, config.camera_roll, config.camera_pitch, config.ground_height);
}

Camera::Camera(CameraConfig config, CameraCalibration calibration, int camera_id)
  : config_(config)
  , calibration_(calibration)
  , internals_(std::make_unique<Internals>(getCameraParams(config.camera_roll*180/M_PI, camera_id)))
{
  config_ = CameraConfig(config.v_fov, config.h_fov, internals_->rotated_size.x, internals_->rotated_size.y, config.camera_roll, config.camera_pitch, config.ground_height);
}

Camera::Camera(CameraConfig config, CameraCalibration calibration, const std::string& video_src)
  : config_(config)
  , calibration_(calibration)
  , internals_(std::make_unique<Internals>(getCameraParams(config.camera_roll*180/M_PI, video_src)))
{
  config_ = CameraConfig(config.v_fov, config.h_fov, internals_->rotated_size.x, internals_->rotated_size.y, config.camera_roll, config.camera_pitch, config.ground_height);
}

Camera::~Camera() = default;

std::optional<Frame> Camera::grab()
{
  if (!internals_->capture_device->grab())
  {
    return std::nullopt;
  }

  cv::Mat cv_frame, cv_corrected, cv_rotated;
  internals_->capture_device->retrieve(cv_frame);

  if (calibration_.valid())
  {
    cv::undistort(cv_frame, cv_corrected, calibration_.camera_matrix, calibration_.dist_coeffs);
    warpAffine(cv_corrected, cv_rotated, internals_->rotation_matrix, internals_->rotated_size.size());
  }
  else
  {
    warpAffine(cv_frame, cv_rotated, internals_->rotation_matrix, internals_->rotated_size.size());
  }

  return std::optional<Frame>(std::move(cv_rotated));
}
