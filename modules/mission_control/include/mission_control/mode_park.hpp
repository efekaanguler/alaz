#ifndef MODE_PARK_HPP
#define MODE_PARK_HPP

#include <mission_control/mode_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <std_msgs/msg/bool.hpp>

class ParkMode : public ModeBase {
public:
    std::string GEAR_PUBLISHER_TOPIC = "/control/command/gear_cmd";
    std::string ENGAGE_PUBLISHER_TOPIC = "/autoware/engage";

    ParkMode(rclcpp::Node::SharedPtr node);
    unsigned int execute() override;

    // Called by the mission controller when a park is triggered by an operator
    // (e.g. via /mission_control/manual_park). A manual hold does NOT time out
    // on its own - it only exits once requestResume() is called.
    void requestManualHold();

    // Called by the mission controller when the operator explicitly asks to
    // resume driving out of a manual park.
    void requestResume();

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr gear_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr engage_publisher_;

    bool park_started_ = false;
    bool manual_hold_ = false;      // true if this park was triggered by an operator override
    bool resume_requested_ = false; // true once the operator asks to resume from a manual hold
    rclcpp::Time park_start_time_;
    rclcpp::Duration park_duration_ = rclcpp::Duration::from_seconds(5.0);      // scripted/automatic park duration
    rclcpp::Duration min_hold_duration_ = rclcpp::Duration::from_seconds(2.0);  // min settle time before a manual resume is honored
};

#endif
