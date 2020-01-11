#include <iostream>
#include <future>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <motion_tracker/camera/camera.h>
#include <motion_tracker/optic_flow_tracker.h>
#include <motion_tracker/motion_estimation.h>

#include <cpp-toolkit/thread_pool.h>
#include <cpp-toolkit/moving_average.h>
#include <motion_tracker/web_viewer.h>

static std::vector<cv::Scalar> color_data;

static const std::vector<cv::Scalar>& getColors()
{
  if (color_data.empty())
  {
    cv::RNG rng;
    for (int i = 0; i < 500; i++)
    {
      int r = rng.uniform(0, 256);
      int g = rng.uniform(0, 256);
      int b = rng.uniform(0, 256);
      color_data.emplace_back(r, g, b);
    }
  }

  return color_data;
}

cv::Mat mark(const cv::Mat& frame, const std::vector<Point2f>& points)
{
  cv::Mat mask = cv::Mat::zeros(frame.size(), frame.type());

  size_t i = 0;
  for (const auto& pt : points)
  {
    cv::Point point(pt.x, pt.y);
    circle(mask, point, 5, getColors()[i++], -1);
  }

  cv::Mat img;
  add(frame, mask, img);
  return img;
}

cv::Mat mark(const cv::Mat& frame, const std::vector<OpticFlow>& flow_pairs, Vector2f offset = Vector2f(0, 0))
{
  cv::Mat mask = cv::Mat::zeros(frame.size(), frame.type());

  size_t i = 0;
  for (const auto& flow : flow_pairs)
  {
    const auto start = flow.start + offset;
    const auto end = flow.end + offset;

    cv::Point start_point(start.x, start.y);
    cv::Point end_point(end.x, end.y);
    line(mask, start_point, end_point, getColors()[i], 2);
    circle(mask, end_point, 5, getColors()[i++], -1);
  }

  cv::Mat img;
  add(frame, mask, img);
  return img;
}

int main()
{
//  CameraConfig camera_conf(85*M_PI/180, 55*M_PI/180, 1080, 1920, -90*M_PI/180.0, 0, 0.2);
  CameraConfig camera_conf(85*M_PI/180, 55*M_PI/180, 640, 480, 0*M_PI/180.0, 0, 0.2);
  Camera cam(camera_conf, CameraCalibration("calib.json"), 0);

  WebViewer viewer("lo0");
  viewer.run(8080);

  const char *window_name = "img";
  namedWindow(window_name, cv::WINDOW_AUTOSIZE);

  auto initial_frame = cam.grab()->toGray();

  auto size_x = initial_frame.data().cols;

  constexpr size_t num_tracked_points = 200;

  OpticFlowTracker tracker_top(initial_frame, Rect<unsigned int>(0, 0, size_x, 240), num_tracked_points);
  OpticFlowTracker tracker_bottom(initial_frame, Rect<unsigned int>(0, 241, size_x, 480), num_tracked_points);

  Vector2f bottom_offset(0, 240);
  ThreadPool workers(4);

  MovingAverage<double, 3> turn_rate_filter;
  MovingAverage<double, 3> linear_speed_filter;

  double total_turn = 0;
  double total_dist = 0;

  while (viewer.running())
  {
    auto ref_time = std::chrono::system_clock::now();

    auto frame = cam.grab();
    if (!frame.has_value())
    {
      break;
    }
    cv::imwrite("debug1.jpg", frame->data(), {cv::IMWRITE_JPEG_QUALITY, 30});
    auto gray_frame = frame->toGray();

    auto flow_top_task = tracker_top.packageCalculation(gray_frame);
    auto flow_bottom_task = tracker_bottom.packageCalculation(gray_frame);

    workers.addWork([&flow_top_task](){ flow_top_task(); });
    workers.addWork([&flow_bottom_task](){ flow_bottom_task(); });

    auto flow_top = flow_top_task.get_future().get();
    auto flow_bottom = flow_bottom_task.get_future().get();

    auto disp_top = mark(frame->data(), flow_top);
    auto disp = mark(disp_top, flow_bottom, bottom_offset);

    double yaw_speed = turn_rate_filter.push(getTurnRateFromFlow(cam.config(), flow_top));
    double linear_speed = linear_speed_filter.push(getSpeedFromFlow(cam.config(), flow_bottom, yaw_speed));


    double dt = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - ref_time).count() / 1000.0;
    total_turn += yaw_speed * dt;
    total_dist += linear_speed * dt;

    viewer.updateFrame(disp, {linear_speed, 1.0 / dt, yaw_speed});
    printf("FPS: %.3f Yaw speed: %.5f [deg/s] linear: %.3f [m/s] total: %.2f [deg] %.2f [m]\n", 1.0 / dt, yaw_speed * 180 / M_PI, linear_speed, total_turn*180/M_PI, total_dist);
  }

  printf("Total heading change: %.2f deg\n", total_turn*180/M_PI);

  return 0;
}
