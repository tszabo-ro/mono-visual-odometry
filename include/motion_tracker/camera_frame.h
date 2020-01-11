#ifndef CameraFrame_h
#define CameraFrame_h

#include <memory>
#include <opencv2/core/mat.hpp>

#include <cpp-toolkit/primitives_2d.h>

class Frame
{
public:
  Frame() {}
  Frame(cv::Mat src): data_(std::move(src)) {}

  [[nodiscard]] Frame toGray() const;
  [[nodiscard]] Frame crop(Rect<unsigned int> roi) const { return crop(roi.start_x, roi.start_y, roi.end_x, roi.end_y); }
  [[nodiscard]] Frame crop(unsigned int start_x, unsigned int start_y, unsigned int end_x, unsigned int end_y) const;

  const cv::Mat& data() const { return data_; }
  bool valid() const { return (data_.rows > 0) && (data_.cols > 0); }

private:
  cv::Mat data_;
};

#endif
