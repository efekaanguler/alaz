#include <mission_control/mode_park.hpp>

ParkMode::ParkMode(rclcpp::Node::SharedPtr node)
: node_(node)
{
  engage_publisher_ = node_->create_publisher<autoware_vehicle_msgs::msg::Engage>(
    ENGAGE_PUBLISHER_TOPIC, 10);
  change_operation_mode_client_ =
    node_->create_client<autoware_system_msgs::srv::ChangeOperationMode>(
    CHANGE_OPERATION_MODE_SERVICE);
  change_autoware_control_client_ =
    node_->create_client<autoware_system_msgs::srv::ChangeAutowareControl>(
    CHANGE_AUTOWARE_CONTROL_SERVICE);
}

unsigned int ParkMode::execute()
{
  if (!park_started_) {
    // Disengage autoware to stop trajectory following
    auto engage_msg = autoware_vehicle_msgs::msg::Engage();
    engage_msg.stamp = node_->now();
    engage_msg.engage = false;
    engage_publisher_->publish(engage_msg);
    if (!stop_mode_request_sent_) {
      if (change_autoware_control_client_->service_is_ready()) {
        auto request = std::make_shared<
          autoware_system_msgs::srv::ChangeAutowareControl::Request>();
        request->autoware_control = false;
        change_autoware_control_client_->async_send_request(request);
      }
      if (change_operation_mode_client_->service_is_ready()) {
        auto request = std::make_shared<
          autoware_system_msgs::srv::ChangeOperationMode::Request>();
        request->mode =
          autoware_system_msgs::srv::ChangeOperationMode::Request::STOP;
        change_operation_mode_client_->async_send_request(request);
        stop_mode_request_sent_ = true;
        RCLCPP_INFO(node_->get_logger(), "Requested Autoware stop operation mode");
      } else {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 5000,
          "Autoware stop operation-mode service is not ready; legacy disengage was published");
      }
    }

    park_start_time_ = node_->now();
    park_started_ = true;
    RCLCPP_INFO(node_->get_logger(), "PARK mode initiated. Holding stop for 5 seconds.");
    return MODE_PARK;
  }

  if ((node_->now() - park_start_time_) >= park_duration_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "Parking hold complete. Remaining disengaged in PAUSE mode.");

    park_started_ = false;
    stop_mode_request_sent_ = false;
    return MODE_PAUSE;
  }

  return MODE_PARK;
}
