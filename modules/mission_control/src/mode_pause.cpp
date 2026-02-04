#include <mission_control/mode_pause.hpp>

PauseMode::PauseMode(rclcpp::Node::SharedPtr node) : node_(node) {}

unsigned int PauseMode::execute() {
    if (!pause_started_) {
        pause_start_time_ = node_->now();
        pause_started_ = true;
        RCLCPP_INFO(node_->get_logger(), "Pause started. Waiting 5 seconds...");
        return MODE_PAUSE;
    }

    if ((node_->now() - pause_start_time_) >= pause_duration_) {
        pause_started_ = false;
        RCLCPP_INFO(node_->get_logger(), "Pause complete. Switching to RUN mode.");
        return MODE_RUN;
    }

    return MODE_PAUSE;
}
