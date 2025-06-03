// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__PNP_SOLVER_HPP_
#define ARMOR_DETECTOR__PNP_SOLVER_HPP_

#include <opencv2/core.hpp>

// STD
#include <array>
#include <vector>

#include "armor_detector/armor.hpp"

namespace rm_auto_aim
{
class PnPSolver
{
public:
  PnPSolver(
    const std::array<double, 9> & camera_matrix,
    const std::vector<double> & distortion_coefficients);

  // Get 3d position
  bool solvePnP(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec);

  // Calculate the distance between armor center and image center
  float calculateDistanceToCenter(const cv::Point2f & image_point);

private:
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  // Unit: mm
  static constexpr float SMALL_ARMOR_WIDTH = 135;
  static constexpr float SMALL_ARMOR_HEIGHT = 55;
  static constexpr float LARGE_ARMOR_WIDTH = 225;
  static constexpr float LARGE_ARMOR_HEIGHT = 55;

  // 三分法拟合相关常量
  static constexpr int ANGLE_FIT_ITERATIONS = 12;  // 三分法迭代次数
  static constexpr double ANGLE_SEARCH_RANGE = 0.26;  // 角度搜索范围(弧度)约15度

  // Four vertices of armor in 3d
  std::vector<cv::Point3f> small_armor_points_;
  std::vector<cv::Point3f> large_armor_points_;

  // 使用三分法优化装甲板朝向
  void optimizeOrientation(
    const std::vector<cv::Point2f>& image_points,
    const std::vector<cv::Point3f>& object_points,
    cv::Mat& rvec,
    cv::Mat& tvec);
    
  // 平滑姿态估计结果
  void smoothPose(cv::Mat& rvec, cv::Mat& tvec);
    
  // 计算特定朝向角度的误差
  double calculateOrientationError(
    const std::vector<cv::Point2f>& image_points,
    const std::vector<cv::Point3f>& object_points,
    double angle,
    double z_component,
    const cv::Mat& tvec);
    
  // 计算重投影误差
  double calculateReprojectionError(
    const std::vector<cv::Point3f>& object_points,
    const std::vector<cv::Point2f>& image_points,
    const cv::Mat& rvec,
    const cv::Mat& tvec);

  // 历史数据，用于平滑和初始化
  cv::Mat last_rvec_;
  cv::Mat last_tvec_;
  bool has_history_ = false;
  
  // 朝向历史数据
  double last_orientation_angle_ = 0.0;
  bool has_orientation_history_ = false;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__PNP_SOLVER_HPP_
