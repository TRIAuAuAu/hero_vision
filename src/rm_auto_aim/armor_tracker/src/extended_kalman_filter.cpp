#include "armor_tracker/extended_kalman_filter.hpp"

namespace rm_auto_aim
{

ExtendedKalmanFilter::ExtendedKalmanFilter(
  const VecVecFunc_f & f, const VecVecFunc_h & h, const VecMatFunc_j_f & j_f, const VecMatFunc_j_h & j_h,
  const VoidMatFunc_u_q & u_q, const VecMatFunc_u_r & u_r, const Eigen::Matrix<double, STATE_DIM, STATE_DIM> & P0)
: f(f),
  h(h),
  jacobian_f(j_f),
  jacobian_h(j_h),
  update_Q(u_q),
  update_R(u_r),
  P_post(P0),
  I(Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity()),
  x_pri(Eigen::Matrix<double, STATE_DIM, 1>::Zero()),
  x_post(Eigen::Matrix<double, STATE_DIM, 1>::Zero())
{
}

void ExtendedKalmanFilter::setState(const Eigen::Matrix<double, STATE_DIM, 1> & x0) { x_post = x0; }

Eigen::Matrix<double, ExtendedKalmanFilter::STATE_DIM, 1> ExtendedKalmanFilter::predict()
{
  F = jacobian_f(x_post);
  Q = update_Q();

    //   // 添加断言检查
    // assert(F.rows() == STATE_DIM && F.cols() == STATE_DIM);  // 检查 F 的维度
    // assert(Q.rows() == STATE_DIM && Q.cols() == STATE_DIM);  // 检查 Q 的维度

  
  x_pri = f(x_post);
  P_pri = F * P_post * F.transpose() + Q;

    //   // 添加断言检查
    // assert(x_pri.rows() == STATE_DIM && x_pri.cols() == 1);  // 检查 x_pri 的维度
    // assert(P_pri.rows() == STATE_DIM && P_pri.cols() == STATE_DIM);  // 检查 P_pri 的维度

  // Handle the case when there will be no measurement before the next predict
  x_post = x_pri;
  P_post = P_pri;

  return x_pri;
}

Eigen::Matrix<double, ExtendedKalmanFilter::STATE_DIM, 1> ExtendedKalmanFilter::update(const Eigen::Matrix<double, MEASUREMENT_DIM, 1> & z)
{
  H = jacobian_h(x_pri);
  R = update_R(z);

  // // 添加断言检查
  //   assert(H.rows() == MEASUREMENT_DIM && H.cols() == STATE_DIM);  // 检查 H 的维度
  //   assert(R.rows() == MEASUREMENT_DIM && R.cols() == MEASUREMENT_DIM);  // 检查 R 的维度


  K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();
  x_post = x_pri + K * (z - h(x_pri));
  P_post = (I - K * H) * P_pri;
  
//  // 添加断言检查
//     assert(x_post.rows() == STATE_DIM && x_post.cols() == 1);  // 检查 x_post 的维度
//     assert(P_post.rows() == STATE_DIM && P_post.cols() == STATE_DIM);  // 检查 P_post 的维度

  return x_post;
}

}  // namespace rm_auto_aim