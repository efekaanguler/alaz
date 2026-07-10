#ifndef MODE_PARK_HPP
#define MODE_PARK_HPP

#include <mission_control/mode_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <std_msgs/msg/bool.hpp>

class ParkMode : public ModeBase {
public:
    std::string GEAR_PUBLISHER_TOPIC = "/control/command/gear_cmd";
    std::string ENGAGE_PUBLISHER_TOPIC = "/autoware/engage";
    std::string CHANGE_TO_STOP_SERVICE = "/api/operation_mode/change_to_stop";

    ParkMode(rclcpp::Node::SharedPtr node);
    unsigned int execute() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr gear_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr engage_publisher_;
    rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr change_to_stop_client_;

    bool park_started_ = false;
    bool stop_mode_request_sent_ = false;
    rclcpp::Time park_start_time_;
    rclcpp::Duration park_duration_ = rclcpp::Duration::from_seconds(5.0);
};

#endif
