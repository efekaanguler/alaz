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

#ifndef MY_VEHICLE_INTERFACE__CAN_UTILS_HPP_
#define MY_VEHICLE_INTERFACE__CAN_UTILS_HPP_

#include <cstdint>
#include <vector>

#include "can_msgs/msg/frame.hpp"

namespace my_vehicle_interface
{
namespace can_utils
{

// =============================================================================
// CAN ID CONFIGURATION - UPDATE THESE FROM WIKI (A 3.3)
// =============================================================================
struct CanIds
{
  // Commands TO kart (Autoware -> Kart)
  uint32_t steering_command = 0x100; // Update from wiki
  uint32_t brake_command = 0x101;    // Update from wiki
  uint32_t throttle_command = 0x102; // Update from wiki

  // Status FROM kart (Kart -> Autoware)
  uint32_t speed_sensor = 0x200;      // Update from wiki
  uint32_t steering_feedback = 0x201; // Update from wiki (if available)
};

// =============================================================================
// ENCODING FUNCTIONS - Convert Autoware commands to CAN frames
// =============================================================================

/**
 * @brief Encode steering command to CAN frame
 * @param steering_angle_rad Target steering angle in radians
 * @param can_ids CAN ID configuration
 * @return CAN frame ready to send
 */
can_msgs::msg::Frame encodeSteeringCommand(
  double steering_angle_rad,
  const CanIds & can_ids);

/**
 * @brief Encode brake command to CAN frame
 * @param brake_value Brake value 0.0 to 1.0 (0% to 100%)
 * @param can_ids CAN ID configuration
 * @return CAN frame ready to send
 */
can_msgs::msg::Frame encodeBrakeCommand(
  double brake_value,
  const CanIds & can_ids);

/**
 * @brief Encode throttle command to CAN frame
 * @param throttle_value Throttle value 0.0 to 1.0 (0% to 100%)
 * @param can_ids CAN ID configuration
 * @return CAN frame ready to send
 */
can_msgs::msg::Frame encodeThrottleCommand(
  double throttle_value,
  const CanIds & can_ids);

// =============================================================================
// DECODING FUNCTIONS - Convert CAN frames to Autoware status
// =============================================================================

/**
 * @brief Decode speed sensor CAN frame
 * @param frame Received CAN frame
 * @return Speed in m/s (converted from hectometer/hour)
 */
double decodeSpeedSensor(const can_msgs::msg::Frame & frame);

/**
 * @brief Decode steering feedback CAN frame (if available)
 * @param frame Received CAN frame
 * @return Current steering angle in radians
 */
double decodeSteeringFeedback(const can_msgs::msg::Frame & frame);

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

/**
 * @brief Convert degrees to radians
 */
inline double degToRad(double deg) {return deg * 0.017453292519943295;}

/**
 * @brief Convert radians to degrees
 */
inline double radToDeg(double rad) {return rad * 57.29577951308232;}

/**
 * @brief Convert hectometer/hour to m/s
 */
inline double hmhToMs(double hmh) {return hmh * 0.027777777777777776;}

/**
 * @brief Convert m/s to hectometer/hour
 */
inline double msToHmh(double ms) {return ms * 36.0;}

} // namespace can_utils
} // namespace my_vehicle_interface

#endif // MY_VEHICLE_INTERFACE__CAN_UTILS_HPP_
