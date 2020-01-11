#ifndef CameraFrame_h
#define CameraFrame_h

#include <memory>
#include <opencv2/core/mat.hpp>

class Frame
{
public:
  Frame() {}
  Frame(cv::Mat src): data_(std::move(src)) {}

  [[nodiscard]] Frame toGray() const;
  [[nodiscard]] Frame crop(unsigned int start_x, unsigned int start_y, unsigned int end_x, unsigned int end_y) const;

  const cv::Mat& data() const { return data_; }
  bool valid() const { return (data_.rows > 0) && (data_.cols > 0); }

private:
  cv::Mat data_;
};

#endif
