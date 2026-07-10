#include <mission_control/mode_park.hpp>

ParkMode::ParkMode(rclcpp::Node::SharedPtr node) : node_(node) {
    gear_publisher_ = node_->create_publisher<autoware_vehicle_msgs::msg::GearCommand>(
        GEAR_PUBLISHER_TOPIC, 10);
    engage_publisher_ = node_->create_publisher<std_msgs::msg::Bool>(
        ENGAGE_PUBLISHER_TOPIC, 10);
}

void ParkMode::requestManualHold() {
    manual_hold_ = true;
    resume_requested_ = false;
}

void ParkMode::requestResume() {
    resume_requested_ = true;
}

unsigned int ParkMode::execute() {
    if (!park_started_) {
        // Disengage autoware to stop trajectory following
        auto engage_msg = std_msgs::msg::Bool();
        engage_msg.data = false;
        engage_publisher_->publish(engage_msg);

        // Publish PARK gear command
        auto gear_msg = autoware_vehicle_msgs::msg::GearCommand();
        gear_msg.stamp = node_->now();
        gear_msg.command = autoware_vehicle_msgs::msg::GearCommand::PARK;
        gear_publisher_->publish(gear_msg);

        park_start_time_ = node_->now();
        park_started_ = true;

        if (manual_hold_) {
            RCLCPP_INFO(node_->get_logger(),
                "PARK Mode initiated (manual hold). Shifting to PARK and waiting for operator resume.");
        } else {
            RCLCPP_INFO(node_->get_logger(),
                "PARK Mode initiated. Shifting to PARK and waiting %.1f seconds.", park_duration_.seconds());
        }
        return MODE_PARK;
    }

    if (manual_hold_) {
        // Manual park never times out on its own. It only exits once the
        // operator explicitly requests resume, and only after a minimum
        // settle time so the gear shift has had time to take effect.
        if (resume_requested_ && (node_->now() - park_start_time_) >= min_hold_duration_) {
            RCLCPP_INFO(node_->get_logger(), "Manual park released by operator. Resuming.");

            auto engage_msg = std_msgs::msg::Bool();
            engage_msg.data = true;
            engage_publisher_->publish(engage_msg);

            auto gear_msg = autoware_vehicle_msgs::msg::GearCommand();
            gear_msg.stamp = node_->now();
            gear_msg.command = autoware_vehicle_msgs::msg::GearCommand::DRIVE;
            gear_publisher_->publish(gear_msg);

            park_started_ = false;
            manual_hold_ = false;
            resume_requested_ = false;
            return MODE_RUN;
        }
        return MODE_PARK;
    }

    // Scripted/automatic park (e.g. a future waypoint-triggered park):
    // keep the original fixed-duration behavior.
    if ((node_->now() - park_start_time_) >= park_duration_) {
        RCLCPP_INFO(node_->get_logger(), "Parking complete. Shifting to DRIVE and resuming.");

        auto engage_msg = std_msgs::msg::Bool();
        engage_msg.data = true;
        engage_publisher_->publish(engage_msg);

        auto gear_msg = autoware_vehicle_msgs::msg::GearCommand();
        gear_msg.stamp = node_->now();
        gear_msg.command = autoware_vehicle_msgs::msg::GearCommand::DRIVE;
        gear_publisher_->publish(gear_msg);

        park_started_ = false; // Reset for future
        return MODE_RUN;
    }

    return MODE_PARK;
}
