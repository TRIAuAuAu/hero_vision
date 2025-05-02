#ifndef ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
#define ARMOR_PROCESSOR__KALMAN_FILTER_HPP_

#include <Eigen/Dense>
#include <functional>

namespace rm_auto_aim
{

class ExtendedKalmanFilter
{
public:
  static constexpr int STATE_DIM = 13;  // 定义状态向量维度
  static constexpr int MEASUREMENT_DIM = 4;  // 定义测量向量维度

  ExtendedKalmanFilter() = default;

  using VecVecFunc_f = std::function<Eigen::Matrix<double, STATE_DIM, 1>(const Eigen::Matrix<double, STATE_DIM, 1> &)>;
  using VecVecFunc_h = std::function<Eigen::Matrix<double, MEASUREMENT_DIM, 1>(const Eigen::Matrix<double, STATE_DIM, 1> &)>;
  using VecMatFunc_j_f = std::function<Eigen::Matrix<double, STATE_DIM, STATE_DIM>(const Eigen::Matrix<double, STATE_DIM, 1> &)>;
  using VecMatFunc_j_h = std::function<Eigen::Matrix<double, MEASUREMENT_DIM, STATE_DIM>(const Eigen::Matrix<double, STATE_DIM, 1> &)>;
  using VecMatFunc_u_r = std::function<Eigen::Matrix<double, MEASUREMENT_DIM, MEASUREMENT_DIM>(const Eigen::Matrix<double, MEASUREMENT_DIM, 1> &)>;
  using VoidMatFunc_u_q = std::function<Eigen::Matrix<double, STATE_DIM, STATE_DIM>()>;

  explicit ExtendedKalmanFilter(
    const VecVecFunc_f & f, const VecVecFunc_h & h, const VecMatFunc_j_f & j_f, const VecMatFunc_j_h & j_h,
    const VoidMatFunc_u_q & u_q, const VecMatFunc_u_r & u_r, const Eigen::Matrix<double, STATE_DIM, STATE_DIM> & P0);

  // Set the initial state
  void setState(const Eigen::Matrix<double, STATE_DIM, 1> & x0);

  // Compute a predicted state
  Eigen::Matrix<double, STATE_DIM, 1> predict();

  // Update the estimated state based on measurement
  Eigen::Matrix<double, STATE_DIM, 1> update(const Eigen::Matrix<double, MEASUREMENT_DIM, 1> & z);

private:
  // Process nonlinear vector function
  VecVecFunc_f f;          // input:13*1 output:13*1
  // Observation nonlinear vector function
  VecVecFunc_h h;          // input:13*1 output:4*1
  // Jacobian of f()
  VecMatFunc_j_f jacobian_f; // input:13*1 output:13*13
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> F;
  // Jacobian of h()
  VecMatFunc_j_h jacobian_h; // input:13*1 output:4*13
  Eigen::Matrix<double, MEASUREMENT_DIM, STATE_DIM> H;
  // Process noise covariance matrix
  VoidMatFunc_u_q update_Q;  // input:void output:13*13
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q;
  // Measurement noise covariance matrix
  VecMatFunc_u_r update_R;   // input:4*1 output:4*4
  Eigen::Matrix<double, MEASUREMENT_DIM, MEASUREMENT_DIM> R;

  // Priori error estimate covariance matrix
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> P_pri;
  // Posteriori error estimate covariance matrix
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> P_post;

  // Kalman gain
  Eigen::Matrix<double, STATE_DIM, MEASUREMENT_DIM> K;

  // N-size identity
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> I;

  // Priori state
  Eigen::Matrix<double, STATE_DIM, 1> x_pri;
  // Posteriori state
  Eigen::Matrix<double, STATE_DIM, 1> x_post;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__KALMAN_FILTER_HPP_