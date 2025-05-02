// Copyright 2022 Chen Jun

#include "armor_tracker/tracker_node.hpp"

// STD
#include <memory>
#include <vector>
#include <cmath>

namespace rm_auto_aim
{
ArmorTrackerNode::ArmorTrackerNode(const rclcpp::NodeOptions & options)
: Node("armor_tracker", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting TrackerNode!");

  // Maximum allowable armor distance in the XOY plane
  max_armor_distance_ = this->declare_parameter("max_armor_distance", 10.0);

  // Tracker
  double max_match_distance = this->declare_parameter("tracker.max_match_distance", 0.15);
  double max_match_yaw_diff = this->declare_parameter("tracker.max_match_yaw_diff", 1.0);
  tracker_ = std::make_unique<Tracker>(max_match_distance, max_match_yaw_diff);
  tracker_->tracking_thres = this->declare_parameter("tracker.tracking_thres", 5);
  lost_time_thres_ = this->declare_parameter("tracker.lost_time_thres", 0.3);

  // EKF
  // xa = x_armor, xc = x_robot_center
  // old_state: xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r (9维)
  // new_state: xc, v_xc, a_xc, yc, v_yc, a_yc, za, v_za, a_za, yaw, v_yaw, a_yaw, r (13维)
  // measurement: xa, ya, za, yaw
  // f - Process function
  auto f = [this](const Eigen::Matrix<double, ExtendedKalmanFilter::STATE_DIM, 1> & x) {
    Eigen::Matrix<double, ExtendedKalmanFilter::STATE_DIM, 1> x_new = x;
    // update position - position = old_position + v*t + 0.5*a*t^2
    x_new(0) += x(1) * dt_ + 0.5 * x(2) * dt_ * dt_;  // xc update
    // update v - v = old_v + a*t
    x_new(1) += x(2) * dt_;                           // v_xc update
    // 加速度保持不变
    
    // y方向
    x_new(3) += x(4) * dt_ + 0.5 * x(5) * dt_ * dt_;  // yc update
    x_new(4) += x(5) * dt_;                           // v_yc update
    
    // z方向
    x_new(6) += x(7) * dt_ + 0.5 * x(8) * dt_ * dt_;  // za update
    x_new(7) += x(8) * dt_;                           // v_za update
    
    // yaw
    x_new(9) += x(10) * dt_ + 0.5 * x(11) * dt_ * dt_; // yaw update
    x_new(10) += x(11) * dt_;                         // v_yaw update
    
    // r (x(12))
    return x_new;
  };
  
  // J_f - Jacobian of process function
  auto j_f = [this](const Eigen::Matrix<double, ExtendedKalmanFilter::STATE_DIM, 1> &) {
    Eigen::Matrix<double, ExtendedKalmanFilter::STATE_DIM, ExtendedKalmanFilter::STATE_DIM> f;
    double dt2 = dt_ * dt_ * 0.5;
    // clang-format off
    //    xc  v_xc a_xc yc  v_yc a_yc za  v_za a_za yaw v_yaw a_yaw r
    f <<  1,  dt_, dt2, 0,  0,   0,   0,  0,   0,   0,  0,    0,    0,  // xc
         0,  1,   dt_, 0,  0,   0,   0,  0,   0,   0,  0,    0,    0,  // v_xc
         0,  0,   1,   0,  0,   0,   0,  0,   0,   0,  0,    0,    0,  // a_xc
         0,  0,   0,   1,  dt_, dt2, 0,  0,   0,   0,  0,    0,    0,  // yc
         0,  0,   0,   0,  1,   dt_, 0,  0,   0,   0,  0,    0,    0,  // v_yc
         0,  0,   0,   0,  0,   1,   0,  0,   0,   0,  0,    0,    0,  // a_yc
         0,  0,   0,   0,  0,   0,   1,  dt_, dt2, 0,  0,    0,    0,  // za
         0,  0,   0,   0,  0,   0,   0,  1,   dt_, 0,  0,    0,    0,  // v_za
         0,  0,   0,   0,  0,   0,   0,  0,   1,   0,  0,    0,    0,  // a_za
         0,  0,   0,   0,  0,   0,   0,  0,   0,   1,  dt_,  dt2,  0,  // yaw
         0,  0,   0,   0,  0,   0,   0,  0,   0,   0,  1,    dt_,  0,  // v_yaw
         0,  0,   0,   0,  0,   0,   0,  0,   0,   0,  0,    1,    0,  // a_yaw
         0,  0,   0,   0,  0,   0,   0,  0,   0,   0,  0,    0,    1;  // r
    // clang-format on
    return f;
  };
  
  // h - Observation function
  auto h = [](const Eigen::Matrix<double, ExtendedKalmanFilter::STATE_DIM, 1> & x) {
    Eigen::Matrix<double, ExtendedKalmanFilter::MEASUREMENT_DIM, 1> z;
    double xc = x(0), yc = x(3), yaw = x(9), r = x(12);
    z(0) = xc - r * cos(yaw);  // xa
    z(1) = yc - r * sin(yaw);  // ya
    z(2) = x(6);               // za
    z(3) = x(9);               // yaw
    return z;
  };
  
  // J_h - Jacobian of observation function
  auto j_h = [](const Eigen::Matrix<double, ExtendedKalmanFilter::STATE_DIM, 1> & x) {
    Eigen::Matrix<double, ExtendedKalmanFilter::MEASUREMENT_DIM, ExtendedKalmanFilter::STATE_DIM> h;
    h.setZero();
    double yaw = x(9), r = x(12);
    // clang-format off
    //       xc    v_xc  a_xc  yc    v_yc  a_yc  za    v_za  a_za  yaw         v_yaw a_yaw r
    h(0,0) = 1;                                               h(0,9) = r*sin(yaw);               h(0,12) = -cos(yaw);  // xa
    h(1,3) = 1;                                               h(1,9) = -r*cos(yaw);              h(1,12) = -sin(yaw);  // ya
    h(2,6) = 1;                                                                                                       // za
    h(3,9) = 1;                                                                                                       // yaw
    // clang-format on
    return h;
  };
  
  // 噪声参数
  s2qxyz_ = declare_parameter("ekf.sigma2_q_xyz", 0.05);
  s2qyaw_ = declare_parameter("ekf.sigma2_q_yaw", 5.0);
  s2qr_ = declare_parameter("ekf.sigma2_q_r", 70.0);
  s2qaac_xyz = declare_parameter("ekf.sigma2_qacc_xyz", 0.1);  // 匹配头文件中的变量名
  s2qaac_yaw = declare_parameter("ekf.sigma2_qacc_yaw", 0.1);  // 匹配头文件中的变量名
  
  // update_Q - process noise covariance matrix
  auto u_q = [this]() {
    Eigen::Matrix<double, ExtendedKalmanFilter::STATE_DIM, ExtendedKalmanFilter::STATE_DIM> q;
    q.setZero();
    double t = dt_;
    
    // 位置和速度的噪声系数
    double x = s2qxyz_, y = s2qyaw_, r = s2qr_;
    // 加速度的噪声系数
    double acc_xyz = s2qaac_xyz, acc_yaw = s2qaac_yaw;  // 匹配头文件中的变量名
    
    // 位置和速度的噪声计算
    double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
    double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
    double q_r = pow(t, 4) / 4 * r;
    
    // 填充噪声协方差矩阵
    // X方向的位置、速度、加速度
    q(0,0) = q_x_x;    q(0,1) = q_x_vx;   
    q(1,0) = q_x_vx;   q(1,1) = q_vx_vx;   
    q(2,2) = acc_xyz;  // x方向加速度噪声
    
    // Y方向的位置、速度、加速度
    q(3,3) = q_x_x;    q(3,4) = q_x_vx;   
    q(4,3) = q_x_vx;   q(4,4) = q_vx_vx;   
    q(5,5) = acc_xyz;  // y方向加速度噪声
    
    // Z方向的位置、速度、加速度
    q(6,6) = q_x_x;    q(6,7) = q_x_vx;   
    q(7,6) = q_x_vx;   q(7,7) = q_vx_vx;
    q(8,8) = acc_xyz;  // z方向加速度噪声
    
    // 偏航角、角速度、角加速度
    q(9,9) = q_y_y;     q(9,10) = q_y_vy;
    q(10,9) = q_y_vy;   q(10,10) = q_vy_vy;
    q(11,11) = acc_yaw;  // 角加速度噪声
    
    // 半径r的噪声
    q(12,12) = q_r;
    
    return q;
  };
  
  // update_R - measurement noise covariance matrix
  r_xyz_factor = declare_parameter("ekf.r_xyz_factor", 0.05); 
  r_yaw = declare_parameter("ekf.r_yaw", 0.02);          
auto u_r = [this](const Eigen::Matrix<double, ExtendedKalmanFilter::MEASUREMENT_DIM, 1> & z) {
    Eigen::Matrix<double, ExtendedKalmanFilter::MEASUREMENT_DIM, ExtendedKalmanFilter::MEASUREMENT_DIM> r = Eigen::Matrix<double, 4, 4>::Zero();
    double x = r_xyz_factor;  
    r(0, 0) = abs(x * z[0]);
    r(1, 1) = abs(x * z[1]);
    r(2, 2) = abs(x * z[2]);
    r(3, 3) = r_yaw;
    return r;
};
  
  // P - error estimate covariance matrix
  Eigen::Matrix<double, ExtendedKalmanFilter::STATE_DIM, ExtendedKalmanFilter::STATE_DIM> p0 =
    Eigen::Matrix<double, ExtendedKalmanFilter::STATE_DIM, ExtendedKalmanFilter::STATE_DIM>::Identity();
    
  tracker_->ekf = ExtendedKalmanFilter{f, h, j_f, j_h, u_q, u_r, p0};

  // Reset tracker service
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  reset_tracker_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/tracker/reset", [this](
                        const std_srvs::srv::Trigger::Request::SharedPtr,
                        std_srvs::srv::Trigger::Response::SharedPtr response) {
      tracker_->tracker_state = Tracker::LOST;
      response->success = true;
      RCLCPP_INFO(this->get_logger(), "Tracker reset!");
      return;
    });

  // // Change target servi-ce
  // change_target_srv_ = this->create_service<std_srvs::srv::Trigger>(
  //   "/tracker/change", [this](
  //                        const std_srvs::srv::Trigger::Request::SharedPtr,
  //                        std_srvs::srv::Trigger::Response::SharedPtr response) {
  //     tracker_->tracker_state = Tracker::LOST;  // 使用LOST替代CHANGE_TARGET，因为没有CHANGE_TARGET枚举值
  //     response->success = true;
  //     RCLCPP_INFO(this->get_logger(), "Target change!");
  //     return;
  //   });

  // Subscriber with tf2 message_filter
  // tf2 relevant
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  // subscriber and filter
  armors_sub_.subscribe(this, "/detector/armors", rmw_qos_profile_sensor_data);
  target_frame_ = this->declare_parameter("target_frame", "odom");
  tf2_filter_ = std::make_shared<tf2_filter>(
    armors_sub_, *tf2_buffer_, target_frame_, 10, this->get_node_logging_interface(),
    this->get_node_clock_interface(), std::chrono::duration<int>(1));
  // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
  tf2_filter_->registerCallback(&ArmorTrackerNode::armorsCallback, this);

  // Measurement publisher (for debug usage)
  info_pub_ = this->create_publisher<auto_aim_interfaces::msg::TrackerInfo>("/tracker/info", 10);

  // Publisher
  target_pub_ = this->create_publisher<auto_aim_interfaces::msg::Target>(
    "/tracker/target", rclcpp::SensorDataQoS());

  // Visualization Marker Publisher
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  position_marker_.ns = "position";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
  position_marker_.color.a = 1.0;
  position_marker_.color.g = 1.0;
  linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  linear_v_marker_.ns = "linear_v";
  linear_v_marker_.scale.x = 0.03;
  linear_v_marker_.scale.y = 0.05;
  linear_v_marker_.color.a = 1.0;
  linear_v_marker_.color.r = 1.0;
  linear_v_marker_.color.g = 1.0;
  angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  angular_v_marker_.ns = "angular_v";
  angular_v_marker_.scale.x = 0.03;
  angular_v_marker_.scale.y = 0.05;
  angular_v_marker_.color.a = 1.0;
  angular_v_marker_.color.b = 1.0;
  angular_v_marker_.color.g = 1.0;
  armor_marker_.ns = "armors";
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.03;
  armor_marker_.scale.z = 0.125;
  armor_marker_.color.a = 1.0;
  armor_marker_.color.r = 1.0;
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tracker/marker", 10);
}

void ArmorTrackerNode::armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_msg)
{
  // Tranform armor position from image frame to world coordinate
  for (auto & armor : armors_msg->armors) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = armors_msg->header;
    ps.pose = armor.pose;
    try {
      armor.pose = tf2_buffer_->transform(ps, target_frame_).pose;
    } catch (const tf2::ExtrapolationException & ex) {
      RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
      return;
    }
  }

  // Filter abnormal armors
  armors_msg->armors.erase(
    std::remove_if(
      armors_msg->armors.begin(), armors_msg->armors.end(),
      [this](const auto_aim_interfaces::msg::Armor & armor) {
        return abs(armor.pose.position.z) > 1.2 ||
               Eigen::Vector2d(armor.pose.position.x, armor.pose.position.y).norm() >
                 max_armor_distance_;
      }),
    armors_msg->armors.end());

  // Init message
  auto_aim_interfaces::msg::TrackerInfo info_msg;
  auto_aim_interfaces::msg::Target target_msg;
  rclcpp::Time time = armors_msg->header.stamp;
  target_msg.header.stamp = time;
  target_msg.header.frame_id = target_frame_;

  // Update tracker
  if (tracker_->tracker_state == Tracker::LOST) {
    tracker_->init(armors_msg);
    target_msg.tracking = false;
  } else {
    dt_ = (time - last_time_).seconds();
    tracker_->lost_thres = static_cast<int>(lost_time_thres_ / dt_);
    tracker_->update(armors_msg);

    // Publish Info
    info_msg.position_diff = tracker_->info_position_diff;
    info_msg.yaw_diff = tracker_->info_yaw_diff;
    info_msg.position.x = tracker_->measurement(0);
    info_msg.position.y = tracker_->measurement(1);
    info_msg.position.z = tracker_->measurement(2);
    info_msg.yaw = tracker_->measurement(3);
    info_pub_->publish(info_msg);

    if (tracker_->tracker_state == Tracker::DETECTING) {
      target_msg.tracking = false;
    } else if (
      tracker_->tracker_state == Tracker::TRACKING ||
      tracker_->tracker_state == Tracker::TEMP_LOST) {
      target_msg.tracking = true;
      // Fill target message
      const auto & state = tracker_->target_state;
      target_msg.id = tracker_->tracked_id;
      target_msg.armors_num = static_cast<int>(tracker_->tracked_armors_num);
      
      target_msg.position.x = state(0);
      target_msg.velocity.x = state(1);
      target_msg.acceleration.x = state(2);  
      
      target_msg.position.y = state(3);
      target_msg.velocity.y = state(4);
      target_msg.acceleration.y = state(5); 
      
      target_msg.position.z = state(6);
      target_msg.velocity.z = state(7);
      target_msg.acceleration.z = state(8);  
      
      target_msg.yaw = state(9);
      target_msg.v_yaw = state(10);
      target_msg.a_yaw = state(11);          
      
      target_msg.radius_1 = state(12);       
      target_msg.radius_2 = tracker_->another_r;
      target_msg.dz = tracker_->dz;
    } else {
      target_msg.tracking = false;
    }
  }

  last_time_ = time;

  target_pub_->publish(target_msg);

  publishMarkers(target_msg);
}

void ArmorTrackerNode::publishMarkers(const auto_aim_interfaces::msg::Target & target_msg)
{
  position_marker_.header = target_msg.header;
  linear_v_marker_.header = target_msg.header;
  angular_v_marker_.header = target_msg.header;
  armor_marker_.header = target_msg.header;

  visualization_msgs::msg::MarkerArray marker_array;
  if (target_msg.tracking) {
    double yaw = target_msg.yaw, r1 = target_msg.radius_1, r2 = target_msg.radius_2;
    double xc = target_msg.position.x, yc = target_msg.position.y, za = target_msg.position.z;
    double vx = target_msg.velocity.x, vy = target_msg.velocity.y, vz = target_msg.velocity.z;
    double dz = target_msg.dz;

    position_marker_.action = visualization_msgs::msg::Marker::ADD;
    position_marker_.pose.position.x = xc;
    position_marker_.pose.position.y = yc;
    position_marker_.pose.position.z = za + dz / 2;

    linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    linear_v_marker_.points.clear();
    linear_v_marker_.points.emplace_back(position_marker_.pose.position);
    geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
    arrow_end.x += vx;
    arrow_end.y += vy;
    arrow_end.z += vz;
    linear_v_marker_.points.emplace_back(arrow_end);

    angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    angular_v_marker_.points.clear();
    angular_v_marker_.points.emplace_back(position_marker_.pose.position);
    arrow_end = position_marker_.pose.position;
    arrow_end.z += target_msg.v_yaw / M_PI;
    angular_v_marker_.points.emplace_back(arrow_end);

    armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    armor_marker_.scale.y = tracker_->tracked_armor.type == "small" ? 0.135 : 0.23;
    bool is_current_pair = true;
    size_t a_n = target_msg.armors_num;
    geometry_msgs::msg::Point p_a;
    double r = 0;
    for (size_t i = 0; i < a_n; i++) {
      double tmp_yaw = yaw + i * (2 * M_PI / a_n);
      // Only 4 armors has 2 radius and height
      if (a_n == 4) {
        r = is_current_pair ? r1 : r2;
        p_a.z = za + (is_current_pair ? 0 : dz);
        is_current_pair = !is_current_pair;
      } else {
        r = r1;
        p_a.z = za;
      }
      p_a.x = xc - r * cos(tmp_yaw);
      p_a.y = yc - r * sin(tmp_yaw);

      armor_marker_.id = i;
      armor_marker_.pose.position = p_a;
      tf2::Quaternion q;
      q.setRPY(0, target_msg.id == "outpost" ? -0.26 : 0.26, tmp_yaw);
      armor_marker_.pose.orientation = tf2::toMsg(q);
      marker_array.markers.emplace_back(armor_marker_);
    }
  } else {
    position_marker_.action = visualization_msgs::msg::Marker::DELETE;
    linear_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
    angular_v_marker_.action = visualization_msgs::msg::Marker::DELETE;

    armor_marker_.action = visualization_msgs::msg::Marker::DELETE;
    marker_array.markers.emplace_back(armor_marker_);
  }

  marker_array.markers.emplace_back(position_marker_);
  marker_array.markers.emplace_back(linear_v_marker_);
  marker_array.markers.emplace_back(angular_v_marker_);
  marker_pub_->publish(marker_array);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorTrackerNode)