#include <mission_control/mode_base.hpp>
#include <rclcpp/rclcpp.hpp>

class PauseMode : public ModeBase {
public:
    PauseMode(rclcpp::Node::SharedPtr node);
    unsigned int execute() override;

private:
    rclcpp::Node::SharedPtr node_;
    bool pause_started_ = false;
    rclcpp::Time pause_start_time_;
    rclcpp::Duration pause_duration_ = rclcpp::Duration::from_seconds(5.0);
};