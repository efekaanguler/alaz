#include <mission_control/mode_base.hpp>
#include <rclcpp/rclcpp.hpp>

class PauseMode : public ModeBase {
public:
    double PAUSE_DURATION_SECONDS = 5.0;
    
    PauseMode(rclcpp::Node::SharedPtr node);
    unsigned int execute() override;
    void requestResume();

private:
    rclcpp::Node::SharedPtr node_;
    bool pause_started_ = false;
    bool resume_requested_ = false;
    rclcpp::Time pause_start_time_;
};