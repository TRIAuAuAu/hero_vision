// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"
#include <geometry_msgs/msg/point.hpp>

namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

  getParams();

  // TF broadcaster
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Create Publisher
  latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);
  projectile_speed_pub_ = this->create_publisher<std_msgs::msg::Float64>("/projectile_speed", 10); // 新增：创建弹速发布者

  // Detect parameter client
  detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");

  // Tracker reset service client
  reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

  try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }

  aiming_point_.header.frame_id = "odom";
  aiming_point_.ns = "aiming_point";
  aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
  aiming_point_.action = visualization_msgs::msg::Marker::ADD;
  aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
  aiming_point_.color.r = 1.0;
  aiming_point_.color.g = 1.0;
  aiming_point_.color.b = 1.0;
  aiming_point_.color.a = 1.0;
  aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);

  // Create Subscription
  target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
    "/tracker/target", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));

  // 声明参数
  max_pitch_err_ = this->declare_parameter("max_pitch_err", 0.1);
  max_yaw_err_ = this->declare_parameter("max_yaw_err", 0.1);

  // 创建弹道解算结果订阅
  projectile_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
    "/projectile_solution", 10,
    std::bind(&RMSerialDriver::projectileSolutionCallback, this, std::placeholders::_1));
}

RMSerialDriver::~RMSerialDriver()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void RMSerialDriver::receiveData()
{
  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data;
  data.reserve(sizeof(ReceivePacket));

  while (rclcpp::ok()) {
    try {
      serial_driver_->port()->receive(header);

      if (header[0] == 0x5A) {
        data.resize(sizeof(ReceivePacket) - 1);
        serial_driver_->port()->receive(data);

        data.insert(data.begin(), header[0]);
        ReceivePacket packet = fromVector(data);

        bool crc_ok =
          crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
        if (crc_ok) {

          // RCLCPP_INFO(get_logger(), "CRC OK!");

          // LOG [Receive] aim_xyz
          // RCLCPP_INFO(get_logger(), "[Receive] aim_pitch %f!", packet.aim_x);
          // RCLCPP_INFO(get_logger(), "[Receive] aim_yaw %f!", packet.aim_y);
          // RCLCPP_INFO(get_logger(), "[Receive] aim_roll %f!", packet.aim_z);
          
          // LOG [Receive] [Receive] rpy
          // RCLCPP_INFO(get_logger(), "[Receive] roll %f!", packet.roll);
          // RCLCPP_INFO(get_logger(), "[Receive] pitch %f!", packet.pitch);
          // RCLCPP_INFO(get_logger(), "[Receive] yaw %f!", packet.yaw);

          // RCLCPP_INFO(get_logger(), "----------------------------");
          
          std_msgs::msg ::Float64 speed_msg;
          speed_msg.data = packet.projectile_speed;
          projectile_speed_pub_->publish(speed_msg);

          // LOG [Receive] speed
          // RCLCPP_INFO(get_logger(), "[Receive] speed %.3f m/s!", packet.projectile_speed);


          if (!initial_set_param_ || packet.detect_color != previous_receive_color_) {
            setParam(rclcpp::Parameter("detect_color", packet.detect_color));
            previous_receive_color_ = packet.detect_color;
          }

          if (packet.reset_tracker) {
            resetTracker();
          }

          geometry_msgs::msg::TransformStamped t;
          timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
          t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
          t.header.frame_id = "odom";
          t.child_frame_id = "gimbal_link";
          tf2::Quaternion q;
          q.setRPY(packet.roll, packet.pitch, packet.yaw);
          t.transform.rotation = tf2::toMsg(q);
          tf_broadcaster_->sendTransform(t);

          if (abs(packet.aim_x) > 0.01) {
            aiming_point_.header.stamp = this->now();
            aiming_point_.pose.position.x = packet.aim_x;
            aiming_point_.pose.position.y = packet.aim_y;
            aiming_point_.pose.position.z = packet.aim_z;
            marker_pub_->publish(aiming_point_);
          }
        } else {
          RCLCPP_ERROR(get_logger(), "CRC error!");
        }
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

void RMSerialDriver::sendData(const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  try {
    SendPacket packet;
    packet.tracking = msg->tracking;
    packet.id = id_unit8_map_.at(msg->id);  // 注意这里添加了下划线
    packet.armors_num = msg->armors_num;

    // 使用最新的弹道解算结果
    {
      std::lock_guard<std::mutex> lock(solution_mutex_);
      packet.pitch = latest_solution_.x;  // pitch角
      packet.yaw = latest_solution_.y;    // yaw角
    }

    // 设置射击指令
    packet.shoot_cmd = (msg->tracking && 
                       std::abs(packet.pitch) < max_pitch_err_ && 
                       std::abs(packet.yaw) < max_yaw_err_) ? 1 : 0;

    // 计算校验和
    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

    // 发送数据
    auto data = toVector(packet);
    serial_driver_->port()->send(data);

    RCLCPP_DEBUG(
      get_logger(),
      "Send success! tracking: %d, yaw: %.3f, pitch: %.3f",
      packet.tracking, packet.yaw, packet.pitch);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending: %s", ex.what());
  }
}

void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}

void RMSerialDriver::setParam(const rclcpp::Parameter & param)
{
  if (!detector_param_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
    return;
  }

  if (
    !set_param_future_.valid() ||
    set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
    set_param_future_ = detector_param_client_->set_parameters(
      {param}, [this, param](const ResultFuturePtr & results) {
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
        initial_set_param_ = true;
      });
  }
}

void RMSerialDriver::resetTracker()
{
  if (!reset_tracker_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_tracker_client_->async_send_request(request);
  RCLCPP_INFO(get_logger(), "Reset tracker!");
}

void RMSerialDriver::projectileSolutionCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(solution_mutex_);
  latest_solution_ = *msg;
  RCLCPP_DEBUG(
    get_logger(),
    "Received solution: pitch = %.3f, yaw = %.3f",
    msg->x, msg->y);
}

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
