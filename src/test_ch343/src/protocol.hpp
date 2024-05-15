#ifndef RM_SERIAL_DRIVER__PROTOCOL_HPP_
#define RM_SERIAL_DRIVER__PROTOCOL_HPP_

#include <sys/cdefs.h>

#include <algorithm>
#include <boost/cstdint.hpp>
#include <cstdint>
#include <iostream>
#include <vector>

// Protocol version: v3.0 (RM2024)

// note: CRC operations are defined in CRC.hpp
/*
 * CRC parameters:
 * width = 16, i.e. CRC-16
 * init = 0xFFFF
 * poly = 0x1189
 * final_xor_value = 0 (do nothing)
 */

// UART parameters:
/*
 * baud_rate = 460800
 * stop bits = 1
 * parity = none
 * hardware_flow_ctrl = no
 * data_size = 8-bit
 */

namespace rm_serial_driver
{
// 0x stands for msg received (1 - 10 for msg from pcb to nuc)
// 1x stands for msg sent (11 - 20 for msg from nuc to pcb)
enum CommunicationType : uint8_t {
  BEAT_MSG = 0x01,           // for test
  GIMBAL_MSG = 0x02,         // gimbal packet received for hero and infantry
  SENTRY_GIMBAL_MSG = 0x03,  // packet received for sentry

  GIMBAL_CMD = 0x11,         // gimbal command sent for hero and infantry
  SENTRY_GIMBAL_CMD = 0x12,  // gimbal command sent for sentry
};

struct BeatMsg  // BEAT_MSG
{
  // header
  uint8_t sof = 0xAAu;
  uint8_t dataLen = 0;
  uint8_t protocolID = 0;
  // data
  uint32_t beat = 0;
  // crc checksum
  uint16_t checksum = 0;
} __attribute__((packed));

struct GimbalMsg  // GIMBAL_MSG
{
  // header
  uint8_t sof = 0xAAu;
  uint8_t dataLen = 0;
  uint8_t protocolID = 0;
  // gimbal msg
  uint8_t cur_cv_mode;
  uint8_t target_color;
  float bullet_speed;
  float q_w;
  float q_x;
  float q_y;
  float q_z;
  // chassis msg
  uint16_t checksum = 0;
} __attribute__((packed));

struct SentryGimbalMsg  // SENTRY_GIMBAL_MSG
{
  // header
  uint8_t sof = 0xAAu;
  uint8_t dataLen = 0;
  uint8_t protocolID = 0;
  // gimbal
  uint8_t target_color;
  float left_gimbal_q_w;
  float left_gimbal_q_x;
  float left_gimbal_q_y;
  float left_gimbal_q_z;
  float right_gimbal_q_w;
  float right_gimbal_q_x;
  float right_gimbal_q_y;
  float right_gimbal_q_z;
  // chassis
  float chassis_vel_x;
  float chassis_vel_y;
  float chassis_vel_w;
  // control mode
  uint8_t path_planner;
  uint8_t control_mode;
  uint8_t action;
  // crc checksum
  uint16_t checksum = 0;
} __attribute__((packed));

struct GimbalCommand  // GIMBAL_CMD
{
  // header
  uint8_t sof = 0xAAu;
  uint8_t dataLen = 0;
  uint8_t protocolID = 0;
  // gimbal control command
  float target_pitch;
  float target_yaw;
  uint8_t shoot_mode;
  // crc
  uint8_t crc_1;
  uint8_t crc_2;
} __attribute__((packed));

struct SentryGimbalCommand  // SENTRY_GIMBAL_CMD
{
  // header
  uint8_t sof = 0xAAu;
  uint8_t dataLen = 0;
  uint8_t protocolID = 0;
  // left gimbal control command
  float left_target_pitch;
  float left_target_yaw;
  uint8_t left_shoot_mode;
  // right gimbal control command
  float right_target_pitch;
  float right_target_yaw;
  uint8_t right_shoot_mode;
  // navigation control command
  float vel_x;
  float vel_y;
  float vel_w;
  // crc
  uint8_t crc_1;
  uint8_t crc_2;

} __attribute__((packed));

// for hero and infantry
inline GimbalMsg fromVector(const std::vector<uint8_t> & data)
{
  GimbalMsg received_packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&received_packet));
  return received_packet;
}

inline std::vector<uint8_t> toVector(const GimbalCommand & data)
{
  std::vector<uint8_t> sent_packet(sizeof(GimbalCommand));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(GimbalCommand), sent_packet.begin());
  return sent_packet;
}

// especially for sentry
inline SentryGimbalMsg fromSentryVector(const std::vector<uint8_t> & data)
{
  SentryGimbalMsg received_packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&received_packet));
  return received_packet;
}

inline std::vector<uint8_t> toSentryVector(const SentryGimbalCommand & data)
{
  std::vector<uint8_t> sent_packet(sizeof(SentryGimbalCommand));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SentryGimbalCommand), sent_packet.begin());
  return sent_packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PROTOCOL_HPP_