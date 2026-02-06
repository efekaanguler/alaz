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

namespace my_vehicle_interface
{
namespace can_utils
{

// =============================================================================
// ENCODING FUNCTIONS
// =============================================================================

can_msgs::msg::Frame encodeSteeringCommand(
  double steering_angle_rad,
  const CanIds & can_ids)
{
  can_msgs::msg::Frame frame;
  frame.id = can_ids.steering_command;
  frame.is_extended = false;
  frame.is_rtr = false;
  frame.dlc = 8;

  // Convert radians to degrees
  double steering_deg = radToDeg(steering_angle_rad);

  // Clamp to valid range (adjust based on your kart's limits)
  steering_deg = std::clamp(steering_deg, -30.0, 30.0);

  // Format: 16-bit signed integer, scale 0.1 degrees
  int16_t steering_raw = static_cast<int16_t>(steering_deg * 10.0);

  // Little-endian encoding (verify byte order from wiki)
  frame.data[0] = steering_raw & 0xFF;
  frame.data[1] = (steering_raw >> 8) & 0xFF;
  frame.data[2] = 0;
  frame.data[3] = 0;
  frame.data[4] = 0;
  frame.data[5] = 0;
  frame.data[6] = 0;
  frame.data[7] = 0;

  return frame;
}

can_msgs::msg::Frame encodeBrakeCommand(
  double brake_value,
  const CanIds & can_ids)
{
  can_msgs::msg::Frame frame;
  frame.id = can_ids.brake_command;
  frame.is_extended = false;
  frame.is_rtr = false;
  frame.dlc = 8;

  // Clamp brake value to 0-1 range
  brake_value = std::clamp(brake_value, 0.0, 1.0);

  // Format: 8-bit unsigned, 0-100%
  uint8_t brake_raw = static_cast<uint8_t>(brake_value * 100.0);

  frame.data[0] = brake_raw;
  frame.data[1] = 0;
  frame.data[2] = 0;
  frame.data[3] = 0;
  frame.data[4] = 0;
  frame.data[5] = 0;
  frame.data[6] = 0;
  frame.data[7] = 0;

  return frame;
}

can_msgs::msg::Frame encodeThrottleCommand(
  double throttle_value,
  const CanIds & can_ids)
{
  can_msgs::msg::Frame frame;
  frame.id = can_ids.throttle_command;
  frame.is_extended = false;
  frame.is_rtr = false;
  frame.dlc = 8;

  // Clamp throttle value to 0-1 range
  throttle_value = std::clamp(throttle_value, 0.0, 1.0);

  // Format: 8-bit unsigned, 0-100%
  uint8_t throttle_raw = static_cast<uint8_t>(throttle_value * 100.0);

  frame.data[0] = throttle_raw;
  frame.data[1] = 0;
  frame.data[2] = 0;
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

double decodeSpeedSensor(const can_msgs::msg::Frame & frame)
{
  // Format: 16-bit unsigned integer in hectometer/hour
  // Little-endian decoding (verify byte order from wiki)
  uint16_t speed_hmh = static_cast<uint16_t>(frame.data[0]) |
    (static_cast<uint16_t>(frame.data[1]) << 8);

  // Convert hectometer/hour to m/s for Autoware
  // Note: Minimum detection is 15 hm/h per regulations
  double speed_mps = hmhToMs(static_cast<double>(speed_hmh));

  return speed_mps;
}

double decodeSteeringFeedback(const can_msgs::msg::Frame & frame)
{
  // Format: 16-bit signed integer, scale 0.1 degrees
  int16_t steering_raw = static_cast<int16_t>(frame.data[0]) |
    (static_cast<int16_t>(frame.data[1]) << 8);

  double steering_deg = static_cast<double>(steering_raw) * 0.1;
  double steering_rad = degToRad(steering_deg);

  return steering_rad;
}

} // namespace can_utils
} // namespace my_vehicle_interface
