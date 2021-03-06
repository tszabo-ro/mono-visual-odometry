#include <motion_tracker/camera/camera_frame.h>
#include <opencv2/imgproc.hpp>
#include <cmath>

Frame Frame::toGray() const
{
  if (!valid())
  {
    return Frame();
  }

  cv::Mat gray_frame;
  cvtColor(data_, gray_frame, cv::COLOR_BGR2GRAY);

  return Frame(std::move(gray_frame));
}

Frame Frame::crop(unsigned int start_x, unsigned int start_y, unsigned int end_x, unsigned int end_y) const
{
  if (!valid())
  {
    return Frame();
  }

  cv::Rect roi(start_x, start_y, end_x - start_x, end_y - start_y);

  bool x_ok = 0 <= roi.x;
  bool y_ok = 0 <= roi.y;
  bool w_pos = 0 <= roi.width;
  bool h_pos = 0 <= roi.height;
  bool w_ok = roi.x + roi.width <= data_.cols;
  bool h_ok = roi.y + roi.height <= data_.rows;

  if (!(x_ok && w_pos && w_ok && y_ok && h_pos && h_ok))
  {
    return Frame();
  }

  return Frame(data_(roi));
}