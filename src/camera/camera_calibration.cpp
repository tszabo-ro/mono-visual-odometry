#include <fstream>

#include <motion_tracker/camera/camera_calibration.h>
#include <nlohmann/json.hpp>


static std::vector<std::vector<double>> mat2stlVec(const cv::Mat& mat)
{
  assert(mat.channels() == 1);

  std::vector<std::vector<double>> data;
  data.reserve(mat.rows);

  for (size_t i = 0; i < static_cast<size_t>(mat.rows); ++i)
  {
    std::vector<double> row;
    row.reserve(mat.cols);

    for (size_t j = 0; j < static_cast<size_t>(mat.cols); ++j)
    {
      row.emplace_back(mat.at<double>(i,j));
    }
    data.emplace_back(std::move(row));
  }

  return data;
}

CameraCalibration::CameraCalibration()
{
  camera_matrix = cv::Mat::zeros(0, 0, CV_64FC1);
  dist_coeffs = cv::Mat::zeros(0, 0, CV_64FC1);
}

CameraCalibration::CameraCalibration(const std::string& file_name):
{

  std::ifstream data_file(file_name);
  if (!data_file.is_open())
  {
    printf("Failed to read calibration data from %s\n", file_name.c_str());
    return;
  }

  std::string json_str((std::istreambuf_iterator<char>(data_file)),std::istreambuf_iterator<char>());
  auto json_data = nlohmann::json::parse(json_str);

  camera_matrix = cv::Mat::zeros(3, 3, CV_64FC1);
  dist_coeffs = cv::Mat::zeros(5, 1, CV_64FC1);


  auto cam_matrix_json = json_data["camera_matrix"];
  for (size_t row_index = 0; row_index < cam_matrix_json.size(); ++row_index)
  {
    auto row = cam_matrix_json[row_index];
    for (size_t col_index = 0; col_index < row.size(); ++col_index)
    {
      camera_matrix.at<double>(row_index, col_index) = row[col_index].get<double>();
    }
  }

  auto dist_coeffs_json = json_data["dist_coeffs"];
  for (size_t row_index = 0; row_index < dist_coeffs_json.size(); ++row_index)
  {
    auto row = dist_coeffs_json[row_index];
    for (size_t col_index = 0; col_index < row.size(); ++col_index)
    {
      dist_coeffs.at<double>(row_index, col_index) = row[col_index].get<double>();
    }
  }
}

nlohmann::json CameraCalibration::dump() const
{
  std::vector<std::vector<double>> camera_matrix_stl = mat2stlVec(camera_matrix);
  std::vector<std::vector<double>> dist_coeffs_stl = mat2stlVec(dist_coeffs);

  std::vector<std::vector<std::vector<double>>> rvecs_stl;
  rvecs_stl.reserve(rvecs.size());
  for (const auto& rvec : rvecs)
  {
    rvecs_stl.emplace_back(mat2stlVec(rvec));
  }

  std::vector<std::vector<std::vector<double>>> tvecs_stl;
  tvecs_stl.reserve(tvecs.size());
  for (const auto& tvec : tvecs)
  {
    rvecs_stl.emplace_back(mat2stlVec(tvec));
  }

  return nlohmann::json({
    {"camera_matrix", nlohmann::json(camera_matrix_stl)},
    {"dist_coeffs", nlohmann::json(dist_coeffs_stl)},
    {"rvecs", nlohmann::json(rvecs_stl)},
    {"tvecs", nlohmann::json(tvecs_stl)},
    {"calibration_error", total_error}
  });
}