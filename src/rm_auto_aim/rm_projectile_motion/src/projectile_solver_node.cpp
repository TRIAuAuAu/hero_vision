#include "rm_projectile_motion/projectile_solver_node.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace rm_projectile_motion
{

ProjectileSolverNode::ProjectileSolverNode(const rclcpp::NodeOptions & options)
: Node("projectile_solver", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting ProjectileSolverNode!");

  use_simple_model_ = declare_parameter("use_simple_model", false); // 默认使用完整模型

  initial_vel_ = declare_parameter("initial_velocity", 15.0);
  friction_k_ = declare_parameter("air_friction", 0.019);
  gravity_ = declare_parameter("gravity", 9.81);
  
  // TF相关参数
  target_frame_ = declare_parameter("target_frame", "odom");
  source_frame_ = declare_parameter("source_frame", "gimbal_link");
  
  // 创建发布者
  solution_pub_ = create_publisher<geometry_msgs::msg::Point>("/projectile_solution", 10);
  marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/projectile_trajectory", 10);
  shoot_cmd_pub_ = create_publisher<std_msgs::msg::UInt8>("/shoot_cmd", 10);
  
  // 设置TF相关对象
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());       
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  
  // 声明弹道参数
  bullet_diameter_ = declare_parameter("bullet_diameter", 0.0425); // 42.5mm弹丸
  bullet_mass_ = declare_parameter("bullet_mass", 0.040);          // 40g弹丸(调车用) 或 47g弹丸(比赛上场)
  air_density_ = declare_parameter("air_density", 1.169);          // 空气密度(kg/m^3)  1.225(标准值) 温度升高需降低该值
  drag_coef_ = declare_parameter("drag_coef", 0.47);               // 球形弹丸阻力系数
  
  // 计算空气阻力系数
  double bullet_area = M_PI * std::pow(bullet_diameter_ / 2.0, 2);
  friction_k_ = (drag_coef_ * air_density_ * bullet_area) / (2.0 * bullet_mass_);
  
  // 声明射击控制参数
  max_pitch_err_ = declare_parameter("max_pitch_err", 0.1);  // 约5.7度
  max_yaw_err_ = declare_parameter("max_yaw_err", 0.1);      // 约5.7度
  
  // 创建订阅者
  target_sub_ = create_subscription<auto_aim_interfaces::msg::Target>(
    "/tracker/target", rclcpp::SensorDataQoS(),
    std::bind(&ProjectileSolverNode::targetCallback, this, std::placeholders::_1));
  // 新增：创建弹速订阅者
  speed_sub_ = create_subscription<std_msgs::msg::Float64>(
    "/projectile_speed", 10, // 订阅由 SerialDriver 发布的话题
    std::bind(&ProjectileSolverNode::speedCallback, this, std::placeholders::_1));
}
void ProjectileSolverNode::speedCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  // 使用互斥锁保护共享资源
  std::lock_guard<std::mutex> lock(initial_vel_mutex_);
  initial_vel_ = msg->data;  // 更新弹速
  // debug
  // RCLCPP_DEBUG(
  //   this->get_logger(),
  //   "Updated initial velocity: %.3f m/s", initial_vel_);
}
void ProjectileSolverNode::targetCallback(const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  if (!msg->tracking) {
    // 未跟踪到目标时发送停止射击命令
    auto shoot_cmd = std_msgs::msg::UInt8();  
    shoot_cmd.data = 0;  // 0表示停止射击
    shoot_cmd_pub_->publish(shoot_cmd);
    return;
  }
  
  Eigen::Vector3d target_pos;
  if (!getTargetPose(msg, target_pos)) {
    return;
  }
  
  // 解算弹道
  double pitch, yaw;
  if (solveBallistic(target_pos, pitch, yaw)) {
    // 发布解算结果
    geometry_msgs::msg::Point solution;
    solution.x = pitch;
    solution.y = yaw;
    solution.z = 0.0;
    solution_pub_->publish(solution);
    
    // 发布可视化marker
    publishMarker(target_pos, pitch, yaw);
    
    // 判断是否满足射击条件
    bool can_shoot = std::abs(pitch) < max_pitch_err_ && 
                    std::abs(yaw) < max_yaw_err_;
    
    // 发布射击指令
    auto shoot_cmd = std_msgs::msg::UInt8();
    shoot_cmd.data = can_shoot ? 1 : 0;
    shoot_cmd_pub_->publish(shoot_cmd);
    
    RCLCPP_DEBUG(
      this->get_logger(),
      "Shoot command: %d, pitch_err: %.3f, yaw_err: %.3f",
      shoot_cmd.data, std::abs(pitch), std::abs(yaw));
    
    // 发布弹道轨迹的TF变换
    publishTransform(target_pos, pitch, yaw);
  }
}

// ----------- 修改：考虑空气阻力的弹道 -----------
bool ProjectileSolverNode::solveBallistic(
  const Eigen::Vector3d & target_pos, double & pitch, double & yaw)
{
  const bool use_simple_model = use_simple_model_; 
  constexpr double MAX_PITCH = M_PI / 3; // 60度最大仰角

  // 局部变量优化
  double x = target_pos.head(2).norm();  // 水平距离
  double y = target_pos.z();             // 垂直高度
  double g = gravity_;

  double v;
  {
    std::lock_guard<std::mutex> lock(initial_vel_mutex_);
    v = initial_vel_;  // 弹速
  }

  // 初始估计
  pitch = std::atan2(y, x);

  // 牛顿迭代法求解
  for(int i = 0;i < 20; i++)
  {
    double cos_pitch = std::cos(pitch);
    double sin_pitch = std::sin(pitch);

    double T, dy;

    // 是否使用简化模型
        if (use_simple_model) {
            // 简化模型（无空气阻力）
            T = x / (v * cos_pitch);
            dy = v * sin_pitch * T - 0.5 * g * T * T;
        } else {
            // 完整模型（含空气阻力）
            const double k_x = friction_k_ * x;
            T = (std::exp(k_x) - 1.0) / (friction_k_ * v * cos_pitch);
            dy = v * sin_pitch * T - 0.5 * g * T * T;
        }
    // 收敛判断
        const double error = y - dy;
        if (std::abs(error) < 0.001) 
        {
          // 计算yaw角度
          yaw = std::atan2(target_pos.y(), target_pos.x());
          return true;
        }

    // 计算导数
    double ddy_dpitch;
    if (use_simple_model) {
        ddy_dpitch = v * x / (cos_pitch * cos_pitch);  // 简化模型的导数
    } else {
        const double dt_dp = T * sin_pitch / cos_pitch;
        ddy_dpitch = v * (sin_pitch * dt_dp + T * cos_pitch) - g * T * dt_dp;
    }

    // 更新pitch角度
    pitch += error / ddy_dpitch;
    pitch = std::clamp(pitch, -MAX_PITCH, MAX_PITCH);
  }
  return false;
}

void ProjectileSolverNode::publishMarker(
  [[maybe_unused]] const Eigen::Vector3d & target_pos,
  double pitch, double yaw)
{
  const bool use_simple = use_simple_model_;
  visualization_msgs::msg::Marker marker;     // 可视化marker
  marker.header.frame_id = target_frame_;     // 目标坐标系
  marker.header.stamp = this->now();          // 时间戳为当前时间
  marker.ns = "projectile_trajectory";        // 命名空间
  marker.id = 0;                              // Marker ID
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;  // 类型:折线
  marker.action = visualization_msgs::msg::Marker::ADD;       // 添加
  marker.pose.orientation.w = 1.0;            // 无旋转
  marker.scale.x = 0.05;                      // 线宽
  marker.color.a = 1.0;                       // 透明度
  marker.color.r = 1.0;                       // 红色
  
  // 计算弹道轨迹点
  double v;
  {
    std::lock_guard<std::mutex> lock(initial_vel_mutex_);
    v = initial_vel_;  // 弹速
  }

  double g = gravity_;
  double dt = 0.01;                          
  double t_max = 3.0;
  const double k = friction_k_;

  double vx0 = v * std::cos(pitch);
  double vy0 = v * std::sin(pitch);
  double x = 0.0, z = 0.0;  // 初始位置

  for (double t = 0; t < t_max; t += dt) {
    geometry_msgs::msg::Point p;
    
    if (use_simple) {
      // 无阻力抛物线模型
       x = vx0 * t;
       z = vy0 * t - 0.5 * g * t * t;
    } else {
      // 解析解来自微分方程：
      // dvx/dt = -k*vx
      // dvz/dt = -k*vz - g
      const double exp_kt = std::exp(-k * t);
      
      // 位置分量（积分得到）
      x = (vx0 / k) * (1 - exp_kt);
      z = ((vy0 + g/k)/k) * (1 - exp_kt) - (g * t)/k;
      // 如果弹丸落到地面以下，停止绘制
    }
    if (z < 0) { break; } 
    
    // 将轨迹点转换到世界坐标系
    p.x = x * std::cos(yaw);
    p.y = x * std::sin(yaw);
    p.z = z;
    
    marker.points.push_back(p);               // 添加点到Marker
  }
  
  marker_pub_->publish(marker);
}

bool ProjectileSolverNode::getTargetPose(
  const auto_aim_interfaces::msg::Target::SharedPtr & msg,
  Eigen::Vector3d & target_pos)
{
  try {
    // 查找目标坐标系到世界坐标系的变换(TF 缓存器)
    geometry_msgs::msg::TransformStamped transform = 
      tf_buffer_->lookupTransform(target_frame_, source_frame_, msg->header.stamp,
                                 rclcpp::Duration::from_seconds(0.1));
                                 
    // 转换目标位置到世界坐标系
    geometry_msgs::msg::Point p;
    p.x = msg->position.x;
    p.y = msg->position.y;
    p.z = msg->position.z;
    
    tf2::doTransform(p, p, transform);  // transform表示一种变换关系，
                                        // 这里是将目标坐标系下的点转换到世界坐标系下
    
    target_pos = Eigen::Vector3d(p.x, p.y, p.z);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
    return false;
  }
}

void ProjectileSolverNode::publishTransform(
  const Eigen::Vector3d & target_pos, double pitch, double yaw)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->now();
  t.header.frame_id = target_frame_;
  t.child_frame_id = "projectile_trajectory";
  
  // 设置变换
  t.transform.translation.x = target_pos.x();
  t.transform.translation.y = target_pos.y();
  t.transform.translation.z = target_pos.z();
  
  tf2::Quaternion q;
  q.setRPY(0.0, pitch, yaw);  // 设置欧拉角
  t.transform.rotation = tf2::toMsg(q);
  
  // 发布变换
  tf_broadcaster_->sendTransform(t);
}

}  // namespace rm_projectile_motion

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_projectile_motion::ProjectileSolverNode)