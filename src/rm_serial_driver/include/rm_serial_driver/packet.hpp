// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{
struct ReceivePacket
{
  uint8_t header = 0x5A;
  uint8_t detect_color;  // 0-red 1-blue
  uint8_t reset_tracker; 
  uint8_t reserved;
  float roll;
  float pitch;
  float yaw;
  float aim_x;
  float aim_y;
  float aim_z;
  float projectile_speed; // 新增弹速字段
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacket
{
  uint8_t header = 0xA5;
  uint8_t tracking;      // 是否正在跟踪目标
  uint8_t id;           // 装甲板ID
  uint8_t armors_num;   // 装甲板数量
  uint8_t shoot_cmd;    
  float yaw;           
  float pitch;         
  uint16_t checksum = 0;
} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

template<typename T>
inline std::vector<uint8_t> toVector(const T & data)
{
  std::vector<uint8_t> packet(sizeof(T));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data), reinterpret_cast<const uint8_t *>(&data) + sizeof(T),
    packet.begin());
  return packet;
}
// inline std::vector<uint8_t> toVector(const SendPacket & data)
// {
//   std::vector<uint8_t> packet(sizeof(SendPacket));
//   std::copy(
//     reinterpret_cast<const uint8_t *>(&data),
//     reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
//   return packet;
// }

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
