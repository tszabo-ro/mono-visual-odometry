#ifndef CameraFrame_h
#define CameraFrame_h

#include <memory>
#include <opencv2/core/mat.hpp>

class Frame
{
public:
  Frame(cv::Mat src): data_(std::move(src)) {}

  [[nodiscard]] const cv::Mat& data() const { return data_; }

private:
  cv::Mat data_;
};

#endif
