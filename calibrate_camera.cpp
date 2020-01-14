#include <iostream>
#include <future>
#include <fstream>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <motion_tracker/camera/camera.h>
#include <motion_tracker/camera/camera_calibration.h>

#include <opencv2/calib3d.hpp>
#include <nlohmann/json.hpp>

enum Pattern
{
  Chessboard,
  CircleGrid,
  AsymmetricCircleGrid
};

static std::vector<cv::Point3f> getGridPattern(const cv::Size& board_size, float element_size)
{
  std::vector<cv::Point3f> corners;
  for (int i = 0; i < board_size.height; ++i)
  {
    for (int j = 0; j < board_size.width; ++j)
    {
      corners.emplace_back(j * element_size, i * element_size, 0);
    }
  }
  return corners;
}

static std::vector<cv::Point3f> getOffsetPattern(const cv::Size& board_size, float element_size)
{
  std::vector<cv::Point3f> corners;
  for (int i = 0; i < board_size.height; i++)
  {
    for (int j = 0; j < board_size.width; j++)
    {
      corners.emplace_back((2 * j + i % 2) * element_size, i * element_size, 0);
    }
  }
  return corners;
}

static std::vector<cv::Point3f> calcBoardCornerPositions(const cv::Size& board_size, float element_size, Pattern pattern_type)
{
  if (pattern_type == Pattern::AsymmetricCircleGrid)
  {
    return getOffsetPattern(board_size, element_size);
  }
  else //if (pattern_type ==  Pattern::Chessboard || pattern_type == Pattern::CIRCLES_GRID)
  {
    return getGridPattern(board_size, element_size);
  }

}

static double computeReprojectionErrors(
  const std::vector<std::vector<cv::Point3f> >& objectPoints, const std::vector<std::vector<cv::Point2f> >& imagePoints, const std::vector<cv::Mat>& rvecs
  , const std::vector<cv::Mat>& tvecs, const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs, std::vector<float>& frame_errors)
{
  std::vector<cv::Point2f> imagePoints2;
  int totalPoints = 0;
  double totalErr = 0, err;
  frame_errors.resize(objectPoints.size());

  for (size_t i = 0; i < objectPoints.size(); ++i)
  {
    projectPoints(cv::Mat(objectPoints[i]), rvecs[i], tvecs[i], camera_matrix, dist_coeffs, imagePoints2);
    err = norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), cv::NORM_L2);

    int n = (int) objectPoints[i].size();
    frame_errors[i] = (float) std::sqrt(err * err / n);
    totalErr += err * err;
    totalPoints += n;
  }

  return std::sqrt(totalErr / totalPoints);
}

static std::optional<CameraCalibration> runCalibration(const cv::Size& board_size, float element_size, Pattern pattern_type,
  const std::vector<std::vector<cv::Point2f>>& recorded_patterns,
  std::vector<float>& reprojection_errors)
{
  CameraCalibration calibration;

  calibration.camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  calibration.camera_matrix.at<double>(0, 0) = 1.0;

  calibration.dist_coeffs = cv::Mat::zeros(8, 1, CV_64F);

  std::vector<std::vector<cv::Point3f>> reference_patterns(
    recorded_patterns.size(), calcBoardCornerPositions(board_size, element_size, pattern_type)
  );

  int calibration_flags = 0;
//  calibration_flags = calibration_flags | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5;
  double rms = calibrateCamera(reference_patterns, recorded_patterns, board_size, calibration.camera_matrix, calibration.dist_coeffs, calibration.rvecs
                               , calibration.tvecs, calibration_flags);

  if (!checkRange(calibration.camera_matrix) || !checkRange(calibration.dist_coeffs))
  {
    printf("Calibration failed. Invalid camera configuration.\n");
    calibration.total_error = std::numeric_limits<double>::max();
    return std::nullopt;
  }

  calibration.total_error = computeReprojectionErrors(reference_patterns, recorded_patterns, calibration.rvecs, calibration.tvecs, calibration.camera_matrix
                                                      , calibration.dist_coeffs, reprojection_errors);
  printf("Calibration finished. Reported error: %.3f Calculated error: %.3f\n", rms, calibration.total_error);

  return std::optional<CameraCalibration>(calibration);
}

static void saveCameraParams(const std::string file_name, const CameraCalibration& calibration, size_t n_frames,
  Pattern calib_pattern, const cv::Size& image_size)
{
  nlohmann::json calib =
    {
      {"meta_data",
                      {
                        {"num_frames", n_frames},
                        {"pattern_type",
                                        calib_pattern == Pattern::Chessboard ? "Chessboard" : (calib_pattern == Pattern::CircleGrid ? "Circle Grid" : "Assymmetric Grid")},
                        {"pattern_width", image_size.width},
                        {"pattern_height", image_size.height}
                      }
      },
      {"calibration", calibration.dump()}
    };

  std::ofstream calib_file(file_name);
  calib_file << calib.dump(2);
  calib_file.close();
}

int main()
{
  const int camera_id = 0;
  const size_t num_images_to_use = 20;
  const cv::Size pattern_size(8, 6); //interior number of corners
  const float square_size = 25;

  const std::string calibration_file = "calib.json";

  const Pattern used_pattern = Pattern::Chessboard;

  CameraConfig camera_conf(85 * M_PI / 180, 55 * M_PI / 180, 1080, 1920, 0, 0, 0.2);
  Camera cam(camera_conf, camera_id);

  const char *window_name = "img";
  namedWindow(window_name, cv::WINDOW_AUTOSIZE);

  auto initial_frame = cam.grab()->toGray();

  std::vector<std::vector<cv::Point2f> > image_points;
  image_points.reserve(num_images_to_use);

  while (image_points.size() < num_images_to_use)
  {
    auto frame = cam.grab();
    if (!frame.has_value())
    {
      break;
    }
    auto gray_frame = frame->toGray();

    std::vector<cv::Point2f> corners; //this will be filled by the detected corners

    bool patternfound = findChessboardCorners(gray_frame.data(), pattern_size, corners,
      cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

    if (patternfound)
    {
      cv::Mat img = frame->data().clone();
      cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 30, 0.1);

      cornerSubPix(gray_frame.data(), corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
      drawChessboardCorners(img, pattern_size, cv::Mat(corners), patternfound);

      image_points.push_back(std::move(corners));
      cv::imshow(window_name, img);
      printf("%zu/%zu images found. \n", image_points.size(), num_images_to_use);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    else
    {
      cv::imshow(window_name, frame->data());
    }

    cv::waitKey(1);
  }

  printf("Collected all the images. Running calibration...\n");
  std::vector<float> reproj_errs;
  auto calibration = runCalibration(pattern_size, square_size, used_pattern, image_points, reproj_errs);
  if (calibration)
  {
    saveCameraParams(calibration_file, calibration.value(), image_points.size(), used_pattern, pattern_size);
  }

  printf("Calibration %s\n", calibration ? "succeeded." : "failed.");

  bool undistort_frame = true;

  printf("Starting diplay...\n");
  while (true)
  {
    printf("1\n");
    auto frame = cam.grab();
    if (!frame.has_value())
    {
      break;
    }

    auto img = frame->data();
    cv::Mat temp = img.clone();

    if (undistort_frame && calibration.has_value())
    {
      undistort(temp, img, calibration->camera_matrix, calibration->dist_coeffs);
    }

    cv::imshow(window_name, img);
    char key = cv::waitKey(1);

    constexpr char ESC_KEY = 27;

    if (key == ESC_KEY)
    { break; }
    if (key == 'u')
    { undistort_frame = !undistort_frame; }

  }

  return 0;
}
