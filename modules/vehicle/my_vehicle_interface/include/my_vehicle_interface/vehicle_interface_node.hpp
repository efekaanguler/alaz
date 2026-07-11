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

#ifndef MY_VEHICLE_INTERFACE__VEHICLE_INTERFACE_NODE_HPP_
#define MY_VEHICLE_INTERFACE__VEHICLE_INTERFACE_NODE_HPP_

#include <memory>

#include "my_vehicle_interface/can_utils.hpp"
#include "rclcpp/rclcpp.hpp"

// Autoware messages
#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_vehicle_msgs/msg/gear_command.hpp"
#include "autoware_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_vehicle_msgs/msg/hazard_lights_command.hpp"
#include "autoware_vehicle_msgs/msg/hazard_lights_report.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_vehicle_msgs/msg/turn_indicators_command.hpp"
#include "autoware_vehicle_msgs/msg/turn_indicators_report.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_vehicle_msgs/srv/control_mode_command.hpp"

// CAN messages
#include "can_msgs/msg/frame.hpp"

namespace my_vehicle_interface
{

class VehicleInterfaceNode : public rclcpp::Node
{
public:
  explicit VehicleInterfaceNode(const rclcpp::NodeOptions & options);

private:
  // =========================================================================
  // CALLBACKS - From Autoware
  // =========================================================================
  void
  onControlCmd(const autoware_control_msgs::msg::Control::ConstSharedPtr msg);
  void
  onGearCmd(const autoware_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg);
  void onTurnIndicatorsCmd(
    const autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr
    msg);
  void onHazardLightsCmd(
    const autoware_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr
    msg);
  void onControlModeRequest(
    const std::shared_ptr<autoware_vehicle_msgs::srv::ControlModeCommand::Request>
    request,
    std::shared_ptr<autoware_vehicle_msgs::srv::ControlModeCommand::Response>
    response);

  // =========================================================================
  // CALLBACKS - From CAN bus (via ros2_socketcan)
  // =========================================================================
  void onCanFrame(const can_msgs::msg::Frame::ConstSharedPtr msg);

  // =========================================================================
  // TIMER CALLBACKS
  // =========================================================================
  void onTimer();

  // =========================================================================
  // HELPER METHODS
  // =========================================================================
  void sendToVehicle();
  void publishVehicleStatus();

  /**
   * @brief Convert Autoware gear command to kart gear value
   * @return 0=neutral, 1=forward, 2=reverse
   */
  uint8_t autowareGearToKartGear(uint8_t autoware_gear) const;

  // =========================================================================
  // SUBSCRIBERS - From Autoware
  // =========================================================================
  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr
    control_cmd_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr
    gear_cmd_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>::
  SharedPtr turn_indicators_cmd_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::HazardLightsCommand>::
  SharedPtr hazard_lights_cmd_sub_;
  rclcpp::Service<autoware_vehicle_msgs::srv::ControlModeCommand>::SharedPtr
    control_mode_service_;

  // Subscriber from CAN bus
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_frame_sub_;

  // =========================================================================
  // PUBLISHERS - To Autoware
  // =========================================================================
  rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr
    velocity_report_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr
    steering_report_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::GearReport>::SharedPtr
    gear_report_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr
    control_mode_report_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr
    turn_indicators_report_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::HazardLightsReport>::SharedPtr
    hazard_lights_report_pub_;

  // Publisher to CAN bus
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_frame_pub_;

  // =========================================================================
  // TIMERS
  // =========================================================================
  rclcpp::TimerBase::SharedPtr timer_;

  // =========================================================================
  // STATE - Latest commands from Autoware
  // =========================================================================
  autoware_control_msgs::msg::Control::ConstSharedPtr control_cmd_ptr_;
  autoware_vehicle_msgs::msg::GearCommand::ConstSharedPtr gear_cmd_ptr_;
  autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr
    turn_indicators_cmd_ptr_;
  autoware_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr
    hazard_lights_cmd_ptr_;

  // =========================================================================
  // STATE - Latest status from vehicle (CAN feedback)
  // =========================================================================
  double current_velocity_mps_{0.0};
  int16_t current_steering_sensor_raw_{0};
  double current_steering_angle_rad_{0.0};

  // Motor feedback state
  uint8_t motor_throttle_dac_{0};
  bool motor_is_braking_{false};
  uint8_t motor_gear_{0};
  bool motor_is_idle_{false};

  // Steering ECU feedback state
  int16_t steer_ecu_current_angle_{0};
  int16_t steer_ecu_target_angle_{0};
  bool steer_ecu_has_error_{false};
  uint8_t current_control_mode_{
    autoware_vehicle_msgs::msg::ControlModeReport::MANUAL};

  // Current gear for CAN commands
  uint8_t current_kart_gear_{0};  // Stay neutral until Autoware requests a gear.

  // =========================================================================
  // CONFIGURATION
  // =========================================================================
  can_utils::CanIds can_ids_;
  double loop_rate_hz_;
  double command_timeout_sec_;
  double max_steering_angle_rad_;
  double accel_to_throttle_gain_;
  double decel_to_brake_gain_;
  bool software_test_mode_;
  bool can_command_output_enabled_;
  rclcpp::Time last_command_time_;
};

}  // namespace my_vehicle_interface

#endif  // MY_VEHICLE_INTERFACE__VEHICLE_INTERFACE_NODE_HPP_
