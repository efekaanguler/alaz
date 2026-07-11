#include <mission_control/mode_pause.hpp>

PauseMode::PauseMode(rclcpp::Node::SharedPtr node) : node_(node) {}

void PauseMode::requestResume() {
    resume_requested_ = true;
}

unsigned int PauseMode::execute() {
    if (!pause_started_) {
        pause_start_time_ = node_->now();
        pause_started_ = true;
        resume_requested_ = false;
        RCLCPP_INFO(node_->get_logger(), "Pause started. Waiting for operator to resume...");
        return MODE_PAUSE;
    }

    if (resume_requested_) {
        pause_started_ = false;
        resume_requested_ = false;
        RCLCPP_INFO(node_->get_logger(), "Pause complete. Switching to RUN mode.");
        return MODE_RUN;
    }

    return MODE_PAUSE;
}
