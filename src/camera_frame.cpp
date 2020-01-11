#include <motion_tracker/camera_frame.h>

Frame Frame::crop(unsigned int start_x, unsigned int start_y, unsigned int end_x, unsigned int end_y) const
{
  cv::Rect roi(start_x, start_y, end_x - start_x, end_y - start_y);
  return Frame(data_(roi));
}