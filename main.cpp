#include <iostream>
#include <future>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <motion_tracker/camera/camera.h>
#include <motion_tracker/optic_flow_tracker.h>

#include <cpp-toolkit/thread_pool.h>

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

cv::Mat mark(const cv::Mat& frame, const std::vector<OpticFlow>& flow_pairs, Vector2f offset = Vector2f(0,0))
{
  cv::Mat mask = cv::Mat::zeros(frame.size(), frame.type());

  size_t i = 0;
  for (const auto& flow : flow_pairs)
  {
    const auto start = flow.anchor + offset;
    const auto end = flow.anchor + flow.flow + offset;

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
  Camera cam(-90, "videos/curvy.mp4");

  const char *window_name = "img";
  namedWindow(window_name, cv::WINDOW_AUTOSIZE);

  auto initial_frame = cam.grab()->toGray();

  auto size_x = initial_frame.data().cols;
  auto size_y = initial_frame.data().rows;

  constexpr size_t num_tracked_points = 200;

  OpticFlowTracker tracker_top(initial_frame, Rect<unsigned int>(0, 0, size_x, 600), num_tracked_points);
  OpticFlowTracker tracker_bottom(initial_frame, Rect<unsigned int>(0, 900, size_x, 1920), num_tracked_points);

  Vector2f bottom_offset(0, 900);

  auto ref_time = std::chrono::system_clock::now();
  size_t frame_count = 0;

  ThreadPool workers(4);

  while (true)
  {
    auto frame = cam.grab();
    if (!frame.has_value())
    {
      break;
    }

    auto gray_frame = frame->toGray();

    std::packaged_task<std::vector<OpticFlow>()> flow_top_task(
      [&tracker_top, &gray_frame]()
        {
          return tracker_top.getFlowVectors(gray_frame);
        });

    std::packaged_task<std::vector<OpticFlow>()> flow_bottom_task(
      [&tracker_bottom, &gray_frame]()
        {
          return tracker_bottom.getFlowVectors(gray_frame);
        });

    workers.addWork([&flow_top_task]() { flow_top_task(); });
    workers.addWork([&flow_bottom_task]() { flow_bottom_task(); });

    auto flow_top = flow_top_task.get_future();
    auto flow_bottom = flow_bottom_task.get_future();

    auto disp_top = mark(frame->data(), flow_top.get());
    auto disp = mark(disp_top, flow_bottom.get(), bottom_offset);

    imshow(window_name, disp);
    cv::waitKey(1);

    ++frame_count;

    if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - ref_time).count() >= 999)
    {
      printf("FPS: %zu. Frame size: %d/%d\n", frame_count, size_x, size_y);
      frame_count = 0;
      ref_time = std::chrono::system_clock::now();
    }
  }

  return 0;
}
