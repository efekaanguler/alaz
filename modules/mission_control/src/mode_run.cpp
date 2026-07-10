#include <mission_control/mode_run.hpp>

#include <cmath>

RunMode::RunMode(rclcpp::Node::SharedPtr node) : node_(node) {
    
    // Publishers to command autoware
    goal_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        GOAL_PUBLISHER_TOPIC, 10);
    
    engage_publisher_ = node_->create_publisher<std_msgs::msg::Bool>(
        ENGAGE_PUBLISHER_TOPIC, 10);

    change_to_autonomous_client_ =
        node_->create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>(
            CHANGE_TO_AUTONOMOUS_SERVICE);
    
    // Subscribe to goal array
    goal_array_subscriber_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
        GOAL_ARRAY_SUBSCRIBER_TOPIC, 10,
        std::bind(&RunMode::goal_array_callback, this, std::placeholders::_1));
    
    // Subscribe to route and planned trajectory
    route_subscriber_ = node_->create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
        ROUTE_SUBSCRIBER_TOPIC, 10,
        std::bind(&RunMode::route_callback, this, std::placeholders::_1));
    
    trajectory_subscriber_ = node_->create_subscription<autoware_planning_msgs::msg::Trajectory>(
        TRAJECTORY_SUBSCRIBER_TOPIC, 10,
        std::bind(&RunMode::trajectory_callback, this, std::placeholders::_1));
    
    // Subscribe to vehicle kinematics
    kinematics_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        KINEMATICS_SUBSCRIBER_TOPIC, 10,
        std::bind(&RunMode::kinematics_callback, this, std::placeholders::_1));

    odom_subscriber_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        ODOM_SUBSCRIBER_TOPIC, 10,
        std::bind(&RunMode::odom_callback, this, std::placeholders::_1));
    
    // TODO: Subscribe to velocity and steering topics when proper control message types are available
    // velocity_subscriber_ = node_->create_subscription<autoware_control_msgs::msg::LongitudinalOutput>(
    //     "/control/trajectory_follower/longitudinal/output", 10,
    //     std::bind(&RunMode::velocity_callback, this, std::placeholders::_1));
    //
    // steering_subscriber_ = node_->create_subscription<autoware_control_msgs::msg::LateralOutput>(
    //     "/control/trajectory_follower/lateral/output", 10,
    //     std::bind(&RunMode::steering_callback, this, std::placeholders::_1));
    
    // Subscribe to emergency topic
    emergency_subscriber_ = node_->create_subscription<std_msgs::msg::Bool>(
        EMERGENCY_SUBSCRIBER_TOPIC, 10,
        std::bind(&RunMode::emergency_callback, this, std::placeholders::_1));
    
    // TODO: Subscribe to routing state to check if goal reached
    // route_state_subscriber_ = node_->create_subscription<autoware_planning_msgs::msg::LaneletRouteState>(
    //     "/api/routing/state", 10,
    //     std::bind(&RunMode::route_state_callback, this, std::placeholders::_1));
}

void RunMode::goal_array_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    goal_array_ = msg->poses;
    current_goal_index_ = 0;
    current_goal_reached_ = false;
    goal_sent_current_ = false;
    RCLCPP_INFO(node_->get_logger(), "Received %zu goals", goal_array_.size());
}

void RunMode::engage_autoware() {
    if (!engaged_) {
        auto engage_msg = std_msgs::msg::Bool();
        engage_msg.data = true;
        engage_publisher_->publish(engage_msg);
        if (!operation_mode_request_sent_) {
            if (change_to_autonomous_client_->service_is_ready()) {
                auto request =
                    std::make_shared<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>();
                change_to_autonomous_client_->async_send_request(request);
                operation_mode_request_sent_ = true;
                RCLCPP_INFO(node_->get_logger(), "Requested Autoware autonomous operation mode");
            } else {
                RCLCPP_WARN_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 5000,
                    "Autoware operation-mode service is not ready; legacy /autoware/engage was published");
            }
        }
        RCLCPP_INFO(node_->get_logger(), "Autoware engage command published");
        engaged_ = true;
    }
}

void RunMode::send_next_goal() {
    if (current_goal_index_ < goal_array_.size()) {
        auto goal_msg = geometry_msgs::msg::PoseStamped();
        goal_msg.header.stamp = node_->now();
        goal_msg.header.frame_id = "map";
        goal_msg.pose = goal_array_[current_goal_index_];
        
        goal_publisher_->publish(goal_msg);
        RCLCPP_INFO(node_->get_logger(), "Sent goal %zu/%zu to autoware", 
                    current_goal_index_ + 1, goal_array_.size());
        
        current_goal_reached_ = false;
        goal_sent_current_ = true;
    }
}

void RunMode::route_callback(const autoware_planning_msgs::msg::LaneletRoute::SharedPtr msg) {
    current_route_ = *msg;
}

void RunMode::trajectory_callback(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg) {
    current_trajectory_ = *msg;
}

void RunMode::kinematics_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    vehicle_kinematics_ = *msg;
}

void RunMode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pose_ = msg->pose.pose;
    has_current_pose_ = true;
    update_goal_reached();
}

// TODO: Implement velocity and steering callbacks when proper control message types are available
// void RunMode::velocity_callback(const autoware_control_msgs::msg::LongitudinalOutput::SharedPtr msg) {
//     target_velocity_ = *msg;
// }
//
// void RunMode::steering_callback(const autoware_control_msgs::msg::LateralOutput::SharedPtr msg) {
//     target_steering_ = *msg;
// }

void RunMode::emergency_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    emergency_flag_ = msg->data;
}

// TODO: Implement route state callback when LaneletRouteState message becomes available
// void RunMode::route_state_callback(const autoware_planning_msgs::msg::LaneletRouteState::SharedPtr msg) {
//     if (msg->state == autoware_planning_msgs::msg::LaneletRouteState::ARRIVED) {
//         current_goal_reached_ = true;
//         RCLCPP_INFO(node_->get_logger(), "Reached goal %zu/%zu", 
//                     current_goal_index_ + 1, goal_array_.size());
//     }
// }

void RunMode::update_goal_reached() {
    if (!goal_sent_current_ || current_goal_reached_ || !has_current_pose_) {
        return;
    }
    if (current_goal_index_ >= goal_array_.size()) {
        return;
    }

    const auto &goal = goal_array_[current_goal_index_];
    const double dx = current_pose_.position.x - goal.position.x;
    const double dy = current_pose_.position.y - goal.position.y;
    const double distance = std::hypot(dx, dy);

    if (distance <= goal_reach_distance_m_) {
        current_goal_reached_ = true;
        RCLCPP_INFO(node_->get_logger(), "Reached goal %zu/%zu (distance %.2f m)",
                    current_goal_index_ + 1, goal_array_.size(), distance);
    }
}

unsigned int RunMode::execute() {
    // Engage autoware on first execution
    engage_autoware();
    
    // Check for emergency
    if (emergency_flag_) {
        RCLCPP_WARN(node_->get_logger(), "Emergency detected! Switching to EMERGENCY mode");
        return MODE_EMERGENCY;
    }
    
    // If no goals, stay in PAUSE mode
    if (goal_array_.empty()) {
        RCLCPP_INFO(node_->get_logger(), "No goal. Switching to Pause Mode");
        return MODE_PAUSE;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Goals detected.");

    update_goal_reached();

    // If current goal reached, pause before next goal
    if (current_goal_reached_) {
        current_goal_index_++;
        goal_sent_current_ = false;
        
        // If all goals completed, park once before returning to idle flow.
        if (current_goal_index_ >= goal_array_.size()) {
            RCLCPP_INFO(node_->get_logger(), "All goals completed! Switching to PARK mode");
            goal_array_.clear();
            current_goal_index_ = 0;
            return MODE_PARK;
        }

        // Pause after each goal
        RCLCPP_INFO(node_->get_logger(), "Goal reached. Switching to PAUSE before next goal");
        return MODE_PAUSE;
    }

    // Send current goal if not sent yet
    if (!goal_sent_current_) {
        send_next_goal();
    }
    
    return MODE_RUN;
}
