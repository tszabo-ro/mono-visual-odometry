#include <motion_tracker/optic_flow_tracker.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

static double distanceSquared(const cv::Point2f& p1, const cv::Point2f& p2)
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  return dx * dx + dy * dy;
}

static std::vector<cv::Point2f> findCorners(const cv::Mat& frame, unsigned int num_points, std::vector<cv::Point2f> points = std::vector<cv::Point2f>())
{
  if (points.size() >= num_points)
  {
    return points;
  }

  points.reserve(num_points);

  const double min_distance = 20;
  const double min_distance_sq = min_distance * min_distance;

  std::vector<cv::Point2f> found_points;
  goodFeaturesToTrack(frame, found_points, num_points * 2, 0.00001, min_distance);

  for (const auto& pt : found_points)
  {
    if (points.size() >= num_points)
    {
      break;
    }

    bool point_ok = true;
    for (const auto& f_pt : points) // This should be done based on a map rendering and not in n2 complexity!
    {
      if (distanceSquared(pt, f_pt) < min_distance_sq)
      {
        point_ok = false;
        break;
      }
    }
    if (point_ok)
    {
      points.emplace_back(pt);
    }
  }

  return points;
}

struct OpticFlowTracker::Internal
{
  Frame last_frame;
  std::vector<cv::Point2f> last_points;
};

OpticFlowTracker::OpticFlowTracker(const Frame& start_frame, Rect<unsigned int> roi, size_t num_points)
  : roi(roi)
    , num_tracked_points(num_points)
    , internal_(std::make_unique<Internal>())
{
  internal_->last_frame = start_frame.crop(roi);
  internal_->last_points = findCorners(internal_->last_frame.data(), num_tracked_points);
}

OpticFlowTracker::~OpticFlowTracker() = default;

std::vector<OpticFlow> OpticFlowTracker::getFlowVectors(const Frame& frame)
{
  Frame cropped_frame = frame.crop(roi);

  auto start_points = findCorners(cropped_frame.data(), num_tracked_points, internal_->last_points);

  std::vector<uchar> status_values;
  std::vector<float> err;

  std::vector<cv::Point2f> tracked_points;

  cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 20, 0.05);
  cv::calcOpticalFlowPyrLK(internal_->last_frame.data(), cropped_frame.data(), start_points, tracked_points, status_values, err, cv::Size(30, 30), 1, criteria);

  std::vector<cv::Point2f> found_points;
  found_points.reserve(tracked_points.size());

  std::vector<OpticFlow> optic_flow_vectors;
  optic_flow_vectors.reserve(tracked_points.size());

  double frame_time_difference = std::chrono::duration_cast<std::chrono::microseconds>(frame.stamp() - internal_->last_frame.stamp()).count()/1000000.0;

  for (size_t index = 0; index < status_values.size(); ++index)
  {
    if (status_values[index] == 1)
    {
      int start_x = internal_->last_points[index].x;
      int start_y = internal_->last_points[index].y;
      int end_x = tracked_points[index].x;
      int end_y = tracked_points[index].y;

      if (start_x <= 0 || start_y <= 0 ||
          end_x <= 0 || end_y <= 0 ||
          start_x >= cropped_frame.data().cols || end_x >= cropped_frame.data().cols ||
          start_y >= cropped_frame.data().rows || end_y >= cropped_frame.data().rows)
      {
        continue;
      }

        optic_flow_vectors.emplace_back(
          Point2i(start_x, start_y),
          Vector2i(end_x - start_x, end_y - start_y),
          frame_time_difference
        );
      found_points.emplace_back(tracked_points[index]);
    }
  }

  internal_->last_points.swap(found_points);
  internal_->last_frame = std::move(cropped_frame);

  return optic_flow_vectors;
}