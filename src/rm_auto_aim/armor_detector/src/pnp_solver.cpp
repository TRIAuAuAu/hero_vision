// Copyright 2022 Chen Jun

#include "armor_detector/pnp_solver.hpp"

#include <opencv2/calib3d.hpp>
#include <vector>
#include <algorithm>
#include <cmath>

namespace rm_auto_aim
{

PnPSolver::PnPSolver(
  const std::array<double, 9> & camera_matrix, 
  const std::vector<double> & dist_coeffs)
: camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
  dist_coeffs_(cv::Mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone())
{
  // Unit: m
  constexpr double small_half_y = SMALL_ARMOR_WIDTH / 2.0 / 1000.0;
  constexpr double small_half_z = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0;
  constexpr double large_half_y = LARGE_ARMOR_WIDTH / 2.0 / 1000.0;
  constexpr double large_half_z = LARGE_ARMOR_HEIGHT / 2.0 / 1000.0;

  // Start from bottom left in clockwise order
  // Model coordinate: x forward, y left, z up
  small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, -small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, -small_half_z));

  large_armor_points_.emplace_back(cv::Point3f(0, large_half_y, -large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, large_half_y, large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, -large_half_z));
}

bool PnPSolver::solvePnP(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec) 
{
  // 提取装甲板图像点
  std::vector<cv::Point2f> image_armor_points;
  image_armor_points.emplace_back(armor.left_light.bottom);
  image_armor_points.emplace_back(armor.left_light.top);
  image_armor_points.emplace_back(armor.right_light.top);
  image_armor_points.emplace_back(armor.right_light.bottom);

  // 选择3D模型点
  auto object_points = armor.type == ArmorType::SMALL ? small_armor_points_ : large_armor_points_;

  // 使用SOLVEPNP_IPPE算法进行初始解算
  bool success = cv::solvePnP(
    object_points, image_armor_points, camera_matrix_, dist_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);

  if (!success) {
    return false;
  }
  
  // 计算当前解的重投影误差
  double initial_error = calculateReprojectionError(
    object_points, image_armor_points, rvec, tvec);
    
  // 如果重投影误差异常大，可能是解算错误，直接返回
  if (initial_error > 20.0) {
    return false;
  }
    
  // 使用三分法优化装甲板朝向
  optimizeOrientation(image_armor_points, object_points, rvec, tvec);
  
  // 仅记录历史结果
  last_rvec_ = rvec.clone();
  last_tvec_ = tvec.clone();
  has_history_ = true;
  
  return true;
}

void PnPSolver::optimizeOrientation(
  const std::vector<cv::Point2f>& image_points,
  const std::vector<cv::Point3f>& object_points,
  cv::Mat& rvec,
  cv::Mat& tvec)
{
  // 从旋转向量获取旋转矩阵
  cv::Mat rotation_matrix;
  cv::Rodrigues(rvec, rotation_matrix);
  
  // 获取初始朝向角度 (装甲板法向量在相机XY平面的投影角)
  cv::Mat armor_normal = rotation_matrix.col(0); // 第一列是装甲板法向量
  double nx = armor_normal.at<double>(0, 0);
  double ny = armor_normal.at<double>(0, 1);
  double initial_angle = std::atan2(ny, nx);
  
  // 三分法搜索最优朝向角度
  double min_angle = initial_angle - ANGLE_SEARCH_RANGE;
  double max_angle = initial_angle + ANGLE_SEARCH_RANGE;
  double best_angle = initial_angle;
  double min_error = calculateOrientationError(
    image_points, object_points, best_angle, armor_normal.at<double>(0, 2), tvec);
    
  // 三分法迭代优化
  for (int i = 0; i < ANGLE_FIT_ITERATIONS; ++i) {
    double range = max_angle - min_angle;
    double mid1 = min_angle + range / 3.0;
    double mid2 = max_angle - range / 3.0;
    
    // 计算两个中点的误差
    double error1 = calculateOrientationError(
      image_points, object_points, mid1, armor_normal.at<double>(0, 2), tvec);
    double error2 = calculateOrientationError(
      image_points, object_points, mid2, armor_normal.at<double>(0, 2), tvec);
    
    // 更新搜索区间
    if (error1 < error2) {
      max_angle = mid2;
      if (error1 < min_error) {
        min_error = error1;
        best_angle = mid1;
      }
    } else {
      min_angle = mid1;
      if (error2 < min_error) {
        min_error = error2;
        best_angle = mid2;
      }
    }
  }

  // 使用最优角度重建旋转矩阵  
  double nz = armor_normal.at<double>(0, 2);
  cv::Mat optimized_normal = (cv::Mat_<double>(3,1) << 
    std::cos(best_angle), std::sin(best_angle), nz);
  cv::normalize(optimized_normal, optimized_normal);
  
  // 构建正交基
  cv::Mat camera_z_axis = (cv::Mat_<double>(3,1) << 0, 0, 1);
  cv::Mat y_axis = camera_z_axis.cross(optimized_normal);
  cv::normalize(y_axis, y_axis);
  cv::Mat z_axis = optimized_normal.cross(y_axis);
  cv::normalize(z_axis, z_axis);
  
  // 更新旋转矩阵
  rotation_matrix.col(0) = optimized_normal;
  rotation_matrix.col(1) = y_axis;
  rotation_matrix.col(2) = z_axis;
  
  // 将旋转矩阵转回旋转向量
  cv::Rodrigues(rotation_matrix, rvec);
  
  // 优化平移向量
  cv::solvePnP(
    object_points, image_points, camera_matrix_, dist_coeffs_, 
    rvec, tvec, true, cv::SOLVEPNP_ITERATIVE);
    
  // 记录优化后的朝向角度
  last_orientation_angle_ = best_angle;
  has_orientation_history_ = true;
}

double PnPSolver::calculateOrientationError(
  const std::vector<cv::Point2f>& image_points,
  const std::vector<cv::Point3f>& object_points,
  double angle,
  double z_component,
  const cv::Mat& tvec)
{
  // 构建朝向向量
  cv::Mat normal = (cv::Mat_<double>(3,1) << std::cos(angle), std::sin(angle), z_component);
  cv::normalize(normal, normal);
  
  // 构建相机坐标系的z轴
  cv::Mat camera_z_axis = (cv::Mat_<double>(3,1) << 0, 0, 1);
  
  // 构建旋转矩阵
  cv::Mat y_axis = camera_z_axis.cross(normal);
  cv::normalize(y_axis, y_axis);
  cv::Mat z_axis = normal.cross(y_axis);
  cv::normalize(z_axis, z_axis);
  
  cv::Mat rotation_matrix = cv::Mat::zeros(3, 3, CV_64F);
  normal.copyTo(rotation_matrix.col(0));
  y_axis.copyTo(rotation_matrix.col(1));
  z_axis.copyTo(rotation_matrix.col(2));
  
  // 将旋转矩阵转为旋转向量
  cv::Mat test_rvec;
  cv::Rodrigues(rotation_matrix, test_rvec);
  
  // 计算重投影误差
  return calculateReprojectionError(object_points, image_points, test_rvec, tvec);
}

double PnPSolver::calculateReprojectionError(
  const std::vector<cv::Point3f>& object_points,
  const std::vector<cv::Point2f>& image_points,
  const cv::Mat& rvec,
  const cv::Mat& tvec)
{
  std::vector<cv::Point2f> projected_points;
  cv::projectPoints(object_points, rvec, tvec, camera_matrix_, dist_coeffs_, projected_points);
  
  double total_error = 0.0;
  for (size_t i = 0; i < image_points.size(); ++i) {
    double error = cv::norm(image_points[i] - projected_points[i]);
    total_error += error * error;
  }
  
  return std::sqrt(total_error / image_points.size());
}

float PnPSolver::calculateDistanceToCenter(const cv::Point2f & image_point)
{
  float cx = camera_matrix_.at<double>(0, 2);
  float cy = camera_matrix_.at<double>(1, 2);
  return cv::norm(image_point - cv::Point2f(cx, cy));
}

}  // namespace rm_auto_aim
