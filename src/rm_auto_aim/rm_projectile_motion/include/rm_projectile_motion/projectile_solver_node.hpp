#ifndef RM_PROJECTILE_MOTION__PROJECTILE_SOLVER_NODE_HPP_
#define RM_PROJECTILE_MOTION__PROJECTILE_SOLVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/msg/u_int8.hpp>  // 注意是u_int8而不是UInt8
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <auto_aim_interfaces/msg/target.hpp>
#include <Eigen/Dense>

#include <std_msgs/msg/float64.hpp> // 新增：弹速 
#include <mutex>                    // 新增：互斥锁

namespace rm_projectile_motion
{

class ProjectileSolverNode : public rclcpp::Node
{
public:
  explicit ProjectileSolverNode(const rclcpp::NodeOptions & options);

private:
  bool use_simple_model_; // 新增模型选择参数
  
  // 订阅目标信息
  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
  // 订阅弹速
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;

  // 发布解算结果
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr solution_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr shoot_cmd_pub_;  // 修正类型
  
  // TF相关
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                    // TF缓存
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;       // TF监听器
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; // TF广播器
  
  double initial_vel_;   //射速
  std::mutex initial_vel_mutex_; // 互斥锁，用于保护射速变量

  double friction_k_;    //空气阻力系数
  double gravity_;       //重力加速度
  std::string target_frame_;
  
  // 添加弹道参数
  double bullet_diameter_;  // 弹丸直径(m)
  double bullet_mass_;     // 弹丸质量(kg)
  double air_density_;     // 空气密度(kg/m^3)
  double drag_coef_;      // 阻力系数
  
  // 添加射击控制参数
  double max_pitch_err_;  // 最大允许俯仰角误差
  double max_yaw_err_;    // 最大允许偏航角误差
  
  // TF相关成员
  std::string source_frame_;
  
  // 回调函数
  void targetCallback(const auto_aim_interfaces::msg::Target::SharedPtr msg);
  void speedCallback(const std_msgs::msg::Float64::SharedPtr msg);  // 新增：弹速回调函数
  bool solveBallistic(const Eigen::Vector3d & target_pos, double & pitch, double & yaw);
  void publishMarker(const Eigen::Vector3d & target_pos, double pitch, double yaw);
  
  // TF相关函数
  bool getTargetPose(const auto_aim_interfaces::msg::Target::SharedPtr & msg, 
                    Eigen::Vector3d & target_pos);
  void publishTransform(const Eigen::Vector3d & target_pos, 
                       double pitch, double yaw);
};

}  // namespace rm_projectile_motion

#endif  // RM_PROJECTILE_MOTION__PROJECTILE_SOLVER_NODE_HPP_