#include <motion_tracker/camera.h>

#include <opencv2/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tuple>

template<class T>
static std::tuple<std::unique_ptr<cv::VideoCapture>, cv::Mat, cv::Rect2f> getCameraParams(double cam_angle, T device_params)
{
  auto capture_device = std::make_unique<cv::VideoCapture>(device_params);

  cv::Mat frame;
  capture_device->grab();
  capture_device->retrieve(frame);

  cv::Point2f center((frame.cols - 1) / 2.0, (frame.rows - 1) / 2.0);
  cv::Rect2f result_frame_size = cv::RotatedRect(cv::Point2f(), frame.size(), cam_angle).boundingRect2f();

  cv::Mat rot = cv::getRotationMatrix2D(center, cam_angle, 1.0);
  rot.at<double>(0, 2) += result_frame_size.width / 2.0 - frame.cols / 2.0;
  rot.at<double>(1, 2) += result_frame_size.height / 2.0 - frame.rows / 2.0;

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

Camera::Camera(CameraConfig config)
  : config_(config)
  , internals_(std::make_unique<Internals>(getCameraParams(config.roll, cv::CAP_ANY)))
{
  cv::Mat img;

  internals_->capture_device->grab();
  internals_->capture_device->retrieve(img);

  config_.img_width = img.cols;
  config_.img_height = img.rows;
}

Camera::Camera(CameraConfig config, const std::string& video_src)
  : config_(config)
  , internals_(std::make_unique<Internals>(getCameraParams(config.roll, video_src)))
{
  cv::Mat img;

  internals_->capture_device->grab();
  internals_->capture_device->retrieve(img);

  config_.img_width = img.cols;
  config_.img_height = img.rows;
}

Camera::~Camera() = default;

std::optional<Frame> Camera::grab()
{
  if (!internals_->capture_device->grab())
  {
    return std::nullopt;
  }

  cv::Mat cv_frame, cv_rotated;
  internals_->capture_device->retrieve(cv_frame);

  warpAffine(cv_frame, cv_rotated, internals_->rotation_matrix, internals_->rotated_size.size());

  return std::optional<Frame>(std::move(cv_rotated));
}
