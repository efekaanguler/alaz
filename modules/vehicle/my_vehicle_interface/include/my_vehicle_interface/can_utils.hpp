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

#include "can_msgs/msg/frame.hpp"

namespace my_vehicle_interface {
namespace can_utils {

// =============================================================================
// CAN ID CONFIGURATION - FROM SDC WIKI
// =============================================================================
struct CanIds {
  // Commands TO kart (Autoware -> Kart)
  uint32_t steering_command = 0x220; // Steering servo (IEEE 754 float)
  uint32_t brake_command = 0x110;    // Brake linear actuator (0-100%)
  uint32_t motor_command = 0x330;    // Motor throttle + gear

  // Feedback FROM kart (Kart -> Autoware)
  uint32_t speed_sensor = 0x440;    // Speed sensor (hm/h, big-endian)
  uint32_t steering_sensor = 0x1E5; // Steering angle sensor (signed int16)
  uint32_t steering_ecu_feedback = 0x720; // Steering ECU feedback
  uint32_t motor_feedback = 0x730;        // Motor ECU feedback
  uint32_t brake_feedback = 0x710;        // Brake ECU feedback
};

// =============================================================================
// ENCODING FUNCTIONS - Convert Autoware commands to CAN frames
// =============================================================================

/**
 * @brief Encode steering command to CAN frame
 *
 * Uses IEEE 754 float encoding (little-endian) as per SDC wiki.
 * Value range: -1.25f (max left) to 1.25f (max right).
 * DLC=4 for angle only, DLC=8 to include steering speed.
 *
 * @param steering_value Normalized steering value (-1.25 to 1.25)
 * @param can_ids CAN ID configuration
 * @return CAN frame ready to send
 */
can_msgs::msg::Frame encodeSteeringCommand(float steering_value,
                                           const CanIds &can_ids);

/**
 * @brief Encode brake command to CAN frame
 *
 * Byte 0: brake percentage 0-100.
 *
 * @param brake_percent Brake value 0-100 (percentage of max force)
 * @param can_ids CAN ID configuration
 * @return CAN frame ready to send
 */
can_msgs::msg::Frame encodeBrakeCommand(uint8_t brake_percent,
                                        const CanIds &can_ids);

/**
 * @brief Encode motor command to CAN frame
 *
 * Byte 0: throttle (0-100, percentage of max speed).
 * Byte 2: gear (0=neutral, 1=forward, 2=reverse).
 * Motor ECU timeout: 200ms (will idle if no messages received).
 *
 * @param throttle_percent Throttle value 0-100
 * @param gear Gear selection: 0=neutral, 1=forward, 2=reverse
 * @param can_ids CAN ID configuration
 * @return CAN frame ready to send
 */
can_msgs::msg::Frame encodeMotorCommand(uint8_t throttle_percent, uint8_t gear,
                                        const CanIds &can_ids);

// =============================================================================
// DECODING FUNCTIONS - Convert CAN frames to Autoware status
// =============================================================================

/**
 * @brief Decode speed sensor CAN frame (0x440)
 *
 * Bytes 0-1: big-endian uint16, speed in hectometers/hour.
 * Minimum detection: ~1.5 km/h (~15 hm/h).
 *
 * @param frame Received CAN frame
 * @return Speed in m/s
 */
double decodeSpeedSensor(const can_msgs::msg::Frame &frame);

/**
 * @brief Decode steering angle sensor CAN frame (0x1E5)
 *
 * Bytes 1-2: big-endian signed int16 (verified from data_recorder.py).
 * Range: -800 to 800.
 *
 * @param frame Received CAN frame
 * @return Steering sensor raw value
 */
int16_t decodeSteeringSensor(const can_msgs::msg::Frame &frame);

/**
 * @brief Decode motor feedback CAN frame (0x730)
 *
 * Byte 0: internal throttle (DAC voltage 0-255).
 * Byte 1: braking flag.
 * Byte 2: gear (0/1/2).
 * Byte 3: idle flag (1 = motor ECU entered idle due to 200ms timeout).
 *
 * @param frame Received CAN frame
 * @param[out] throttle_dac DAC voltage 0-255
 * @param[out] is_braking true if brake is applied
 * @param[out] gear current gear
 * @param[out] is_idle true if motor ECU timed out
 */
void decodeMotorFeedback(const can_msgs::msg::Frame &frame,
                         uint8_t &throttle_dac, bool &is_braking, uint8_t &gear,
                         bool &is_idle);

/**
 * @brief Decode steering ECU feedback CAN frame (0x720)
 *
 * Bytes 0-1: last known angle (big-endian int16).
 * Bytes 2-3: target angle (big-endian int16).
 * Byte 4: direction (0=clockwise, 1=counterclockwise).
 * Byte 5: error flag.
 *
 * @param frame Received CAN frame
 * @param[out] current_angle Current steering angle raw value
 * @param[out] target_angle Target steering angle raw value
 * @param[out] has_error true if ECU is in failsafe
 */
void decodeSteeringEcuFeedback(const can_msgs::msg::Frame &frame,
                               int16_t &current_angle, int16_t &target_angle,
                               bool &has_error);

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

/**
 * @brief Convert steering angle in radians to kart float value (-1.25 to 1.25)
 *
 * Autoware sends steering_tire_angle in radians.
 * Kart expects a float in range -1.25 to 1.25.
 * This requires calibration of the max steering angle.
 *
 * @param rad Steering angle in radians from Autoware
 * @param max_steering_rad Maximum physical steering angle in radians
 * @return Normalized value for kart (-1.25 to 1.25)
 */
float radToKartSteering(double rad, double max_steering_rad);

/**
 * @brief Convert hectometer/hour to m/s
 */
inline double hmhToMs(double hmh) { return hmh * 0.027777777777777776; }

/**
 * @brief Convert m/s to hectometer/hour
 */
inline double msToHmh(double ms) { return ms * 36.0; }

} // namespace can_utils
} // namespace my_vehicle_interface

#endif // MY_VEHICLE_INTERFACE__CAN_UTILS_HPP_
