#include <mission_control/mode_park.hpp>

ParkMode::ParkMode(rclcpp::Node::SharedPtr node) : node_(node) {
    gear_publisher_ = node_->create_publisher<autoware_vehicle_msgs::msg::GearCommand>(
        GEAR_PUBLISHER_TOPIC, 10);
    engage_publisher_ = node_->create_publisher<std_msgs::msg::Bool>(
        ENGAGE_PUBLISHER_TOPIC, 10);
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
        RCLCPP_INFO(node_->get_logger(), "PARK Mode initiated. Shifting to PARK and waiting 5 seconds.");
        return MODE_PARK;
    }
    
    if ((node_->now() - park_start_time_) >= park_duration_) {
        RCLCPP_INFO(node_->get_logger(), "Parking complete. Shifting to DRIVE and resuming.");
        
        // Re-engage autoware
        auto engage_msg = std_msgs::msg::Bool();
        engage_msg.data = true;
        engage_publisher_->publish(engage_msg);
        
        // Publish DRIVE gear command
        auto gear_msg = autoware_vehicle_msgs::msg::GearCommand();
        gear_msg.stamp = node_->now();
        gear_msg.command = autoware_vehicle_msgs::msg::GearCommand::DRIVE;
        gear_publisher_->publish(gear_msg);

        park_started_ = false; // Reset for future
        return MODE_RUN;
    }
    
    return MODE_PARK;
}
