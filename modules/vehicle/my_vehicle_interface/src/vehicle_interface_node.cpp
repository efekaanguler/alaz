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

#include "my_vehicle_interface/vehicle_interface_node.hpp"

#include <chrono>
#include <cmath>
#include <functional>

namespace my_vehicle_interface {

using namespace std::chrono_literals;
using std::placeholders::_1;

VehicleInterfaceNode::VehicleInterfaceNode(const rclcpp::NodeOptions &options)
    : Node("vehicle_interface_node", options) {
  // ==========================================================================
  // PARAMETERS
  // ==========================================================================
  loop_rate_hz_ = this->declare_parameter("loop_rate_hz", 25.0);
  command_timeout_sec_ = this->declare_parameter("command_timeout_sec", 0.2);

  // Max steering angle (radians) - needs calibration on real kart
  max_steering_angle_rad_ =
      this->declare_parameter("max_steering_angle_rad", 0.5236);

  // Acceleration to throttle/brake mapping gains
  accel_to_throttle_gain_ =
      this->declare_parameter("accel_to_throttle_gain", 0.33);
  decel_to_brake_gain_ = this->declare_parameter("decel_to_brake_gain", 0.20);

  RCLCPP_INFO(this->get_logger(), "Vehicle Interface Node starting...");
  RCLCPP_INFO(this->get_logger(), "  Loop rate: %.1f Hz", loop_rate_hz_);
  RCLCPP_INFO(this->get_logger(), "  Command timeout: %.2f sec",
              command_timeout_sec_);
  RCLCPP_INFO(this->get_logger(), "  Max steering angle: %.4f rad",
              max_steering_angle_rad_);
  RCLCPP_INFO(this->get_logger(), "  CAN IDs (from SDC Wiki):");
  RCLCPP_INFO(this->get_logger(), "    Steering:  0x%X",
              can_ids_.steering_command);
  RCLCPP_INFO(this->get_logger(), "    Brake:     0x%X",
              can_ids_.brake_command);
  RCLCPP_INFO(this->get_logger(), "    Motor:     0x%X",
              can_ids_.motor_command);
  RCLCPP_INFO(this->get_logger(), "    Speed:     0x%X", can_ids_.speed_sensor);
  RCLCPP_INFO(this->get_logger(), "    Steer FB:  0x%X",
              can_ids_.steering_sensor);
  RCLCPP_INFO(this->get_logger(), "    Motor FB:  0x%X",
              can_ids_.motor_feedback);

  // ==========================================================================
  // SUBSCRIBERS - From Autoware
  // ==========================================================================
  control_cmd_sub_ =
      this->create_subscription<autoware_control_msgs::msg::Control>(
          "/control/command/control_cmd", rclcpp::QoS{1},
          std::bind(&VehicleInterfaceNode::onControlCmd, this, _1));

  gear_cmd_sub_ =
      this->create_subscription<autoware_vehicle_msgs::msg::GearCommand>(
          "/control/command/gear_cmd", rclcpp::QoS{1},
          std::bind(&VehicleInterfaceNode::onGearCmd, this, _1));

  turn_indicators_cmd_sub_ = this->create_subscription<
      autoware_vehicle_msgs::msg::TurnIndicatorsCommand>(
      "/control/command/turn_indicators_cmd", rclcpp::QoS{1},
      std::bind(&VehicleInterfaceNode::onTurnIndicatorsCmd, this, _1));

  hazard_lights_cmd_sub_ = this->create_subscription<
      autoware_vehicle_msgs::msg::HazardLightsCommand>(
      "/control/command/hazard_lights_cmd", rclcpp::QoS{1},
      std::bind(&VehicleInterfaceNode::onHazardLightsCmd, this, _1));

  // Subscriber from CAN bus (via ros2_socketcan)
  can_frame_sub_ = this->create_subscription<can_msgs::msg::Frame>(
      "/from_can_bus", rclcpp::QoS{100},
      std::bind(&VehicleInterfaceNode::onCanFrame, this, _1));

  // ==========================================================================
  // PUBLISHERS - To Autoware
  // ==========================================================================
  velocity_report_pub_ =
      this->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(
          "/vehicle/status/velocity_status", rclcpp::QoS{1});

  steering_report_pub_ =
      this->create_publisher<autoware_vehicle_msgs::msg::SteeringReport>(
          "/vehicle/status/steering_status", rclcpp::QoS{1});

  gear_report_pub_ =
      this->create_publisher<autoware_vehicle_msgs::msg::GearReport>(
          "/vehicle/status/gear_status", rclcpp::QoS{1});

  control_mode_report_pub_ =
      this->create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>(
          "/vehicle/status/control_mode", rclcpp::QoS{1});

  turn_indicators_report_pub_ =
      this->create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>(
          "/vehicle/status/turn_indicators_status", rclcpp::QoS{1});

  hazard_lights_report_pub_ =
      this->create_publisher<autoware_vehicle_msgs::msg::HazardLightsReport>(
          "/vehicle/status/hazard_lights_status", rclcpp::QoS{1});

  // Publisher to CAN bus (via ros2_socketcan)
  can_frame_pub_ = this->create_publisher<can_msgs::msg::Frame>(
      "/to_can_bus", rclcpp::QoS{100});

  // ==========================================================================
  // TIMER - Match wiki example sending speed of 0.04s = 25 Hz
  // ==========================================================================
  auto timer_period = std::chrono::duration<double>(1.0 / loop_rate_hz_);
  timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&VehicleInterfaceNode::onTimer, this));

  last_command_time_ = this->now();

  RCLCPP_INFO(this->get_logger(),
              "Vehicle Interface Node initialized successfully!");
}

// =============================================================================
// CALLBACKS - From Autoware
// =============================================================================

void VehicleInterfaceNode::onControlCmd(
    const autoware_control_msgs::msg::Control::ConstSharedPtr msg) {
  control_cmd_ptr_ = msg;
  last_command_time_ = this->now();
}

void VehicleInterfaceNode::onGearCmd(
    const autoware_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg) {
  gear_cmd_ptr_ = msg;
  current_kart_gear_ = autowareGearToKartGear(msg->command);
}

void VehicleInterfaceNode::onTurnIndicatorsCmd(
    const autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr
        msg) {
  turn_indicators_cmd_ptr_ = msg;
}

void VehicleInterfaceNode::onHazardLightsCmd(
    const autoware_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg) {
  hazard_lights_cmd_ptr_ = msg;
}

// =============================================================================
// CALLBACKS - From CAN bus
// =============================================================================

void VehicleInterfaceNode::onCanFrame(
    const can_msgs::msg::Frame::ConstSharedPtr msg) {
  // Route CAN frames based on ID
  if (msg->id == can_ids_.speed_sensor) {
    // Speed sensor (0x440): big-endian uint16, hm/h
    current_velocity_mps_ = can_utils::decodeSpeedSensor(*msg);
    RCLCPP_DEBUG(this->get_logger(), "Speed: %.2f m/s", current_velocity_mps_);

  } else if (msg->id == can_ids_.steering_sensor) {
    // Steering angle sensor (0x1E5): big-endian signed int16
    current_steering_sensor_raw_ = can_utils::decodeSteeringSensor(*msg);
    // Convert raw sensor value to radians for Autoware
    // Raw range is approximately -800 to 800
    current_steering_angle_rad_ =
        static_cast<double>(current_steering_sensor_raw_) / 800.0 *
        max_steering_angle_rad_;
    RCLCPP_DEBUG(this->get_logger(), "Steering sensor: %d (%.4f rad)",
                 current_steering_sensor_raw_, current_steering_angle_rad_);

  } else if (msg->id == can_ids_.motor_feedback) {
    // Motor ECU feedback (0x730)
    can_utils::decodeMotorFeedback(*msg, motor_throttle_dac_, motor_is_braking_,
                                   motor_gear_, motor_is_idle_);
    RCLCPP_DEBUG(
        this->get_logger(), "Motor FB: DAC=%d brake=%d gear=%d idle=%d",
        motor_throttle_dac_, motor_is_braking_, motor_gear_, motor_is_idle_);

    if (motor_is_idle_) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Motor ECU reports IDLE (no 0x330 messages for 200ms)");
    }

  } else if (msg->id == can_ids_.steering_ecu_feedback) {
    // Steering ECU feedback (0x720)
    can_utils::decodeSteeringEcuFeedback(*msg, steer_ecu_current_angle_,
                                         steer_ecu_target_angle_,
                                         steer_ecu_has_error_);
    RCLCPP_DEBUG(this->get_logger(), "Steer ECU: current=%d target=%d error=%d",
                 steer_ecu_current_angle_, steer_ecu_target_angle_,
                 steer_ecu_has_error_);

    if (steer_ecu_has_error_) {
      RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Steering ECU FAILSAFE! Check steering sensor or angle range.");
    }
  }
  // Brake feedback (0x710) - log only for diagnostics
}

// =============================================================================
// TIMER CALLBACK - Main control loop
// =============================================================================

void VehicleInterfaceNode::onTimer() {
  // Check for command timeout
  double elapsed = (this->now() - last_command_time_).seconds();
  if (elapsed > command_timeout_sec_ && control_cmd_ptr_) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Control command timeout (%.2f sec), sending zero commands", elapsed);
  }

  // Send commands to vehicle
  sendToVehicle();

  // Publish vehicle status to Autoware
  publishVehicleStatus();
}

// =============================================================================
// HELPER METHODS
// =============================================================================

void VehicleInterfaceNode::sendToVehicle() {
  // ==========================================================================
  // Determine command values
  // ==========================================================================
  float steer_value = 0.0f;
  uint8_t throttle_percent = 0;
  uint8_t brake_percent = 0;
  uint8_t gear = current_kart_gear_;

  if (control_cmd_ptr_) {
    double elapsed = (this->now() - last_command_time_).seconds();
    bool timed_out = (elapsed > command_timeout_sec_);

    if (!timed_out) {
      // Steering: convert Autoware radians -> kart float (-1.25 to 1.25)
      double steering_angle = control_cmd_ptr_->lateral.steering_tire_angle;
      steer_value =
          can_utils::radToKartSteering(steering_angle, max_steering_angle_rad_);

      // Throttle/Brake: convert Autoware acceleration -> kart percentages
      double target_accel = control_cmd_ptr_->longitudinal.acceleration;

      if (target_accel > 0.0) {
        // Positive acceleration -> throttle
        double throttle_frac =
            std::clamp(target_accel * accel_to_throttle_gain_, 0.0, 1.0);
        throttle_percent = static_cast<uint8_t>(throttle_frac * 100.0);
        brake_percent = 0;
      } else if (target_accel < 0.0) {
        // Negative acceleration -> brake
        double brake_frac =
            std::clamp(-target_accel * decel_to_brake_gain_, 0.0, 1.0);
        throttle_percent = 0;
        brake_percent = static_cast<uint8_t>(brake_frac * 100.0);
      }
    }
    // If timed out: steer_value=0, throttle=0, brake=0 (safe defaults)
  }

  // ==========================================================================
  // Encode and send CAN frames (matching wiki example 0.04s / 25Hz cycle)
  // ==========================================================================

  // Steering command (0x220): IEEE 754 float, little-endian
  auto steering_frame = can_utils::encodeSteeringCommand(steer_value, can_ids_);
  can_frame_pub_->publish(steering_frame);

  // Motor command (0x330): throttle + gear (must send at least every 200ms!)
  auto motor_frame =
      can_utils::encodeMotorCommand(throttle_percent, gear, can_ids_);
  can_frame_pub_->publish(motor_frame);

  // Brake command (0x110): brake percentage
  auto brake_frame = can_utils::encodeBrakeCommand(brake_percent, can_ids_);
  can_frame_pub_->publish(brake_frame);
}

void VehicleInterfaceNode::publishVehicleStatus() {
  auto stamp = this->now();

  // Velocity Report
  {
    autoware_vehicle_msgs::msg::VelocityReport msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "base_link";
    msg.longitudinal_velocity = static_cast<float>(current_velocity_mps_);
    msg.lateral_velocity = 0.0f;
    msg.heading_rate = 0.0f;
    velocity_report_pub_->publish(msg);
  }

  // Steering Report
  {
    autoware_vehicle_msgs::msg::SteeringReport msg;
    msg.stamp = stamp;
    msg.steering_tire_angle = static_cast<float>(current_steering_angle_rad_);
    steering_report_pub_->publish(msg);
  }

  // Gear Report - use motor feedback if available
  {
    autoware_vehicle_msgs::msg::GearReport msg;
    msg.stamp = stamp;
    switch (motor_gear_) {
    case 0:
      msg.report = autoware_vehicle_msgs::msg::GearReport::NEUTRAL;
      break;
    case 1:
      msg.report = autoware_vehicle_msgs::msg::GearReport::DRIVE;
      break;
    case 2:
      msg.report = autoware_vehicle_msgs::msg::GearReport::REVERSE;
      break;
    default:
      msg.report = autoware_vehicle_msgs::msg::GearReport::DRIVE;
      break;
    }
    gear_report_pub_->publish(msg);
  }

  // Control Mode Report
  {
    autoware_vehicle_msgs::msg::ControlModeReport msg;
    msg.stamp = stamp;
    msg.mode = autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
    control_mode_report_pub_->publish(msg);
  }

  // Turn Indicators Report (kart does not have these)
  {
    autoware_vehicle_msgs::msg::TurnIndicatorsReport msg;
    msg.stamp = stamp;
    msg.report = autoware_vehicle_msgs::msg::TurnIndicatorsReport::DISABLE;
    turn_indicators_report_pub_->publish(msg);
  }

  // Hazard Lights Report (kart does not have these)
  {
    autoware_vehicle_msgs::msg::HazardLightsReport msg;
    msg.stamp = stamp;
    msg.report = autoware_vehicle_msgs::msg::HazardLightsReport::DISABLE;
    hazard_lights_report_pub_->publish(msg);
  }
}

uint8_t
VehicleInterfaceNode::autowareGearToKartGear(uint8_t autoware_gear) const {
  // Convert Autoware gear values to kart gear values
  // Autoware: NEUTRAL=1, DRIVE(D)=2, REVERSE=20, PARK=22, etc.
  // Kart: 0=neutral, 1=forward, 2=reverse
  switch (autoware_gear) {
  case autoware_vehicle_msgs::msg::GearCommand::NEUTRAL:
    return 0;
  case autoware_vehicle_msgs::msg::GearCommand::DRIVE:
  case autoware_vehicle_msgs::msg::GearCommand::DRIVE_2:
  case autoware_vehicle_msgs::msg::GearCommand::DRIVE_3:
  case autoware_vehicle_msgs::msg::GearCommand::LOW:
  case autoware_vehicle_msgs::msg::GearCommand::LOW_2:
    return 1;
  case autoware_vehicle_msgs::msg::GearCommand::REVERSE:
  case autoware_vehicle_msgs::msg::GearCommand::REVERSE_2:
    return 2;
  case autoware_vehicle_msgs::msg::GearCommand::PARK:
    return 0; // Park -> neutral (kart has no park)
  default:
    return 1; // Default to forward
  }
}

} // namespace my_vehicle_interface

// Register as component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_vehicle_interface::VehicleInterfaceNode)
