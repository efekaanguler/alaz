#ifndef MODE_PARK_HPP
#define MODE_PARK_HPP

#include <mission_control/mode_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <autoware_system_msgs/srv/change_autoware_control.hpp>
#include <autoware_system_msgs/srv/change_operation_mode.hpp>
#include <autoware_vehicle_msgs/msg/engage.hpp>

class ParkMode : public ModeBase
{
public:
  std::string ENGAGE_PUBLISHER_TOPIC = "/autoware/engage";
  std::string CHANGE_OPERATION_MODE_SERVICE = "/system/operation_mode/change_operation_mode";
  std::string CHANGE_AUTOWARE_CONTROL_SERVICE = "/system/operation_mode/change_autoware_control";

  ParkMode(rclcpp::Node::SharedPtr node);
  unsigned int execute() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::Engage>::SharedPtr engage_publisher_;
  rclcpp::Client<autoware_system_msgs::srv::ChangeOperationMode>::SharedPtr
    change_operation_mode_client_;
  rclcpp::Client<autoware_system_msgs::srv::ChangeAutowareControl>::SharedPtr
    change_autoware_control_client_;

  bool park_started_ = false;
  bool stop_mode_request_sent_ = false;
  rclcpp::Time park_start_time_;
  rclcpp::Duration park_duration_ = rclcpp::Duration::from_seconds(5.0);
};

#endif
