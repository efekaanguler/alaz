// Copyright 2026 SDC Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "my_vehicle_interface/can_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace my_vehicle_interface {
namespace can_utils {

// =============================================================================
// ENCODING FUNCTIONS
// =============================================================================

can_msgs::msg::Frame encodeSteeringCommand(float steering_value,
                                           const CanIds &can_ids) {
  can_msgs::msg::Frame frame;
  frame.id = can_ids.steering_command;
  frame.is_extended = false;
  frame.is_rtr = false;
  frame.dlc = 4;

  // Clamp to valid range per wiki: -1.25f to 1.25f
  steering_value = std::clamp(steering_value, -1.25f, 1.25f);

  // IEEE 754 float, little-endian encoding (same as struct.pack("f", value))
  // Example: 0.5f -> [0x00, 0x00, 0x00, 0x3F]
  // Example: 1.0f -> [0x00, 0x00, 0x80, 0x3F]
  std::memcpy(&frame.data[0], &steering_value, sizeof(float));

  frame.data[4] = 0;
  frame.data[5] = 0;
  frame.data[6] = 0;
  frame.data[7] = 0;

  return frame;
}

can_msgs::msg::Frame encodeBrakeCommand(uint8_t brake_percent,
                                        const CanIds &can_ids) {
  can_msgs::msg::Frame frame;
  frame.id = can_ids.brake_command;
  frame.is_extended = false;
  frame.is_rtr = false;
  frame.dlc = 8;

  // Clamp brake value to 0-100 range
  brake_percent = std::min(brake_percent, static_cast<uint8_t>(100));

  // Byte 0: brake percentage (0-100)
  frame.data[0] = brake_percent;
  frame.data[1] = 0;
  frame.data[2] = 0;
  frame.data[3] = 0;
  frame.data[4] = 0;
  frame.data[5] = 0;
  frame.data[6] = 0;
  frame.data[7] = 0;

  return frame;
}

can_msgs::msg::Frame encodeMotorCommand(uint8_t throttle_percent, uint8_t gear,
                                        const CanIds &can_ids) {
  can_msgs::msg::Frame frame;
  frame.id = can_ids.motor_command;
  frame.is_extended = false;
  frame.is_rtr = false;
  frame.dlc = 8;

  // Clamp throttle to 0-100
  throttle_percent = std::min(throttle_percent, static_cast<uint8_t>(100));

  // Clamp gear to valid values (0=neutral, 1=forward, 2=reverse)
  // Wiki: if gear >= 3, entire message is ignored!
  gear = std::min(gear, static_cast<uint8_t>(2));

  // Byte 0: throttle percentage (0-100)
  // Byte 1: reserved (0x00)
  // Byte 2: gear (0=N, 1=F, 2=R)
  // Bytes 3-7: reserved (0x00)
  frame.data[0] = throttle_percent;
  frame.data[1] = 0;
  frame.data[2] = gear;
  frame.data[3] = 0;
  frame.data[4] = 0;
  frame.data[5] = 0;
  frame.data[6] = 0;
  frame.data[7] = 0;

  return frame;
}

// =============================================================================
// DECODING FUNCTIONS
// =============================================================================

double decodeSpeedSensor(const can_msgs::msg::Frame &frame) {
  // Bytes 0-1: BIG-endian uint16, speed in hectometers/hour
  // Example: [0x01, 0x4A] = 330 hm/h = 33.0 km/h = 9.17 m/s
  // NOTE: If speed readings are incorrect (e.g. extremely low/high), check if
  // bytes [1] and [2] should be used instead (like steering sensor 0x1E5).
  uint16_t speed_hmh = (static_cast<uint16_t>(frame.data[0]) << 8) |
                       static_cast<uint16_t>(frame.data[1]);

  return hmhToMs(static_cast<double>(speed_hmh));
}

int16_t decodeSteeringSensor(const can_msgs::msg::Frame &frame) {
  // Bytes 1-2: BIG-endian signed int16
  // Verified from data_recorder.py: (data[1] << 8 | data[2])
  // Example: [0xFF, 0xFC] = -4
  // Range: -800 to 800
  int16_t raw =
      static_cast<int16_t>((static_cast<uint16_t>(frame.data[1]) << 8) |
                           static_cast<uint16_t>(frame.data[2]));

  return raw;
}

void decodeMotorFeedback(const can_msgs::msg::Frame &frame,
                         uint8_t &throttle_dac, bool &is_braking, uint8_t &gear,
                         bool &is_idle) {
  // Byte 0: internal throttle (DAC voltage 0-255)
  // Byte 1: braking flag (1 = brake applied, cannot throttle or change gear)
  // Byte 2: gear feedback (0=N, 1=F, 2=R)
  // Byte 3: idle flag (1 = ECU timed out after 200ms without 0x330 messages)
  throttle_dac = frame.data[0];
  is_braking = (frame.data[1] != 0);
  gear = frame.data[2];
  is_idle = (frame.data[3] != 0);
}

void decodeSteeringEcuFeedback(const can_msgs::msg::Frame &frame,
                               int16_t &current_angle, int16_t &target_angle,
                               bool &has_error) {
  // Bytes 0-1: last known angle (big-endian int16)
  current_angle =
      static_cast<int16_t>((static_cast<uint16_t>(frame.data[0]) << 8) |
                           static_cast<uint16_t>(frame.data[1]));

  // Bytes 2-3: target angle (big-endian int16)
  target_angle =
      static_cast<int16_t>((static_cast<uint16_t>(frame.data[2]) << 8) |
                           static_cast<uint16_t>(frame.data[3]));

  // Byte 5: error flag (failsafe state)
  // Failsafe triggers when:
  //   1. No steering sensor CAN messages for 0.2 sec
  //   2. Steering angle outside -800 to 800 range
  has_error = (frame.data[5] != 0);
}

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

float radToKartSteering(double rad, double max_steering_rad) {
  // Convert Autoware steering angle (radians) to kart value (-1.25 to 1.25)
  // max_steering_rad = physical max steering angle of the kart
  // When rad == max_steering_rad, output should be 1.25f (or close to 1.0f)
  // The wiki says original max was 1.0f, increased to 1.25f

  if (max_steering_rad <= 0.0) {
    return 0.0f;
  }

  float normalized = static_cast<float>(rad / max_steering_rad);
  return std::clamp(normalized, -1.25f, 1.25f);
}

} // namespace can_utils
} // namespace my_vehicle_interface
