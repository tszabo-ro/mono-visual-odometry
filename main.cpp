#include <iostream>
#include <motion_tracker/camera.h>
#include <cmath>

#include <opencv2/highgui.hpp>

int main()
{
  Camera cam(-90, "videos/straight.mp4");

  const char* window_name = "img";
  namedWindow( window_name, cv::WINDOW_AUTOSIZE );

  while (true)
  {
    auto frame = cam.grab();
    if (!frame.has_value())
    {
      break;
    }

    imshow( window_name, frame->data() );
    cv::waitKey(1);
  }

  return 0;
}
