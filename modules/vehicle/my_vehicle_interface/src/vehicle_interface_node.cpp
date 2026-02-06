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
#include <functional>

namespace my_vehicle_interface
{

using namespace std::chrono_literals;
using std::placeholders::_1;

VehicleInterfaceNode::VehicleInterfaceNode(const rclcpp::NodeOptions & options)
: Node("vehicle_interface_node", options)
{
  // ==========================================================================
  // PARAMETERS
  // ==========================================================================
  loop_rate_hz_ = this->declare_parameter("loop_rate_hz", 100.0);
  command_timeout_sec_ = this->declare_parameter("command_timeout_sec", 0.5);

  // CAN IDs - Update these from wiki (A 3.3)
  can_ids_.steering_command =
    static_cast<uint32_t>(this->declare_parameter("steering_can_id", 0x100));
  can_ids_.brake_command =
    static_cast<uint32_t>(this->declare_parameter("brake_can_id", 0x101));
  can_ids_.throttle_command =
    static_cast<uint32_t>(this->declare_parameter("throttle_can_id", 0x102));
  can_ids_.speed_sensor =
    static_cast<uint32_t>(this->declare_parameter("speed_can_id", 0x200));
  can_ids_.steering_feedback = static_cast<uint32_t>(
    this->declare_parameter("steering_feedback_can_id", 0x201));

  RCLCPP_INFO(this->get_logger(), "Vehicle Interface Node starting...");
  RCLCPP_INFO(this->get_logger(), "  Loop rate: %.1f Hz", loop_rate_hz_);
  RCLCPP_INFO(
    this->get_logger(), "  Steering CAN ID: 0x%X",
    can_ids_.steering_command);
  RCLCPP_INFO(
    this->get_logger(), "  Brake CAN ID: 0x%X",
    can_ids_.brake_command);
  RCLCPP_INFO(
    this->get_logger(), "  Throttle CAN ID: 0x%X",
    can_ids_.throttle_command);
  RCLCPP_INFO(
    this->get_logger(), "  Speed sensor CAN ID: 0x%X",
    can_ids_.speed_sensor);

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
  // TIMER
  // ==========================================================================
  auto timer_period = std::chrono::duration<double>(1.0 / loop_rate_hz_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
    std::bind(&VehicleInterfaceNode::onTimer, this));

  last_command_time_ = this->now();

  RCLCPP_INFO(
    this->get_logger(),
    "Vehicle Interface Node initialized successfully!");
}

// =============================================================================
// CALLBACKS - From Autoware
// =============================================================================

void VehicleInterfaceNode::onControlCmd(
  const autoware_control_msgs::msg::Control::ConstSharedPtr msg)
{
  control_cmd_ptr_ = msg;
  last_command_time_ = this->now();
}

void VehicleInterfaceNode::onGearCmd(
  const autoware_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg)
{
  gear_cmd_ptr_ = msg;
}

void VehicleInterfaceNode::onTurnIndicatorsCmd(
  const autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr
  msg)
{
  turn_indicators_cmd_ptr_ = msg;
}

void VehicleInterfaceNode::onHazardLightsCmd(
  const autoware_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg)
{
  hazard_lights_cmd_ptr_ = msg;
}

// =============================================================================
// CALLBACKS - From CAN bus
// =============================================================================

void VehicleInterfaceNode::onCanFrame(
  const can_msgs::msg::Frame::ConstSharedPtr msg)
{
  // Route CAN frames based on ID
  if (msg->id == can_ids_.speed_sensor) {
    current_velocity_mps_ = can_utils::decodeSpeedSensor(*msg);
    RCLCPP_DEBUG(this->get_logger(), "Speed: %.2f m/s", current_velocity_mps_);
  } else if (msg->id == can_ids_.steering_feedback) {
    current_steering_angle_rad_ = can_utils::decodeSteeringFeedback(*msg);
    RCLCPP_DEBUG(
      this->get_logger(), "Steering: %.2f rad",
      current_steering_angle_rad_);
  }
  // Add more CAN ID handlers as needed
}

// =============================================================================
// TIMER CALLBACK - Main control loop
// =============================================================================

void VehicleInterfaceNode::onTimer()
{
  // Check for command timeout
  double elapsed = (this->now() - last_command_time_).seconds();
  if (elapsed > command_timeout_sec_ && control_cmd_ptr_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Control command timeout (%.2f sec), stopping vehicle",
      elapsed);
    // Could implement emergency stop here
  }

  // Send commands to vehicle
  sendToVehicle();

  // Publish vehicle status to Autoware
  publishVehicleStatus();
}

// =============================================================================
// HELPER METHODS
// =============================================================================

void VehicleInterfaceNode::sendToVehicle()
{
  if (!control_cmd_ptr_) {
    return;
  }

  // Extract control values from Autoware command
  // New Autoware Universe uses Control message with lateral/longitudinal
  double steering_angle = control_cmd_ptr_->lateral.steering_tire_angle;
  double target_accel = control_cmd_ptr_->longitudinal.acceleration;

  // ==========================================================================
  // Steering command (Type A - target angle to servo)
  // ==========================================================================
  auto steering_frame =
    can_utils::encodeSteeringCommand(steering_angle, can_ids_);
  can_frame_pub_->publish(steering_frame);

  // ==========================================================================
  // Throttle/Brake commands (Type B - direct actuation)
  // Convert target acceleration to throttle/brake percentages
  // ==========================================================================

  double throttle_cmd = 0.0;
  double brake_cmd = 0.0;

  if (target_accel > 0.0) {
    // Positive acceleration -> throttle
    // Calibrate this mapping based on kart behavior
    // Simple linear mapping: 0-3 m/s^2 -> 0-100% throttle
    throttle_cmd = std::clamp(target_accel / 3.0, 0.0, 1.0);
    brake_cmd = 0.0;
  } else if (target_accel < 0.0) {
    // Negative acceleration -> brake
    // Calibrate this mapping based on kart behavior
    // Simple linear mapping: 0 to -5 m/s^2 -> 0-100% brake
    throttle_cmd = 0.0;
    brake_cmd = std::clamp(-target_accel / 5.0, 0.0, 1.0);
  }

  auto throttle_frame =
    can_utils::encodeThrottleCommand(throttle_cmd, can_ids_);
  can_frame_pub_->publish(throttle_frame);

  auto brake_frame = can_utils::encodeBrakeCommand(brake_cmd, can_ids_);
  can_frame_pub_->publish(brake_frame);
}

void VehicleInterfaceNode::publishVehicleStatus()
{
  auto stamp = this->now();

  // Velocity Report
  {
    autoware_vehicle_msgs::msg::VelocityReport msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "base_link";
    msg.longitudinal_velocity = static_cast<float>(current_velocity_mps_);
    msg.lateral_velocity = 0.0f;
    msg.heading_rate = 0.0f; // Could compute from steering + velocity
    velocity_report_pub_->publish(msg);
  }

  // Steering Report
  {
    autoware_vehicle_msgs::msg::SteeringReport msg;
    msg.stamp = stamp;
    msg.steering_tire_angle = static_cast<float>(current_steering_angle_rad_);
    steering_report_pub_->publish(msg);
  }

  // Gear Report (kart is always in "Drive")
  {
    autoware_vehicle_msgs::msg::GearReport msg;
    msg.stamp = stamp;
    msg.report = autoware_vehicle_msgs::msg::GearReport::DRIVE;
    gear_report_pub_->publish(msg);
  }

  // Control Mode Report
  {
    autoware_vehicle_msgs::msg::ControlModeReport msg;
    msg.stamp = stamp;
    msg.mode = is_autonomous_mode_ ?
      autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS :
      autoware_vehicle_msgs::msg::ControlModeReport::MANUAL;
    control_mode_report_pub_->publish(msg);
  }

  // Turn Indicators Report (kart may not have these)
  {
    autoware_vehicle_msgs::msg::TurnIndicatorsReport msg;
    msg.stamp = stamp;
    msg.report = autoware_vehicle_msgs::msg::TurnIndicatorsReport::DISABLE;
    turn_indicators_report_pub_->publish(msg);
  }

  // Hazard Lights Report (kart may not have these)
  {
    autoware_vehicle_msgs::msg::HazardLightsReport msg;
    msg.stamp = stamp;
    msg.report = autoware_vehicle_msgs::msg::HazardLightsReport::DISABLE;
    hazard_lights_report_pub_->publish(msg);
  }
}

} // namespace my_vehicle_interface

// Register as component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_vehicle_interface::VehicleInterfaceNode)
