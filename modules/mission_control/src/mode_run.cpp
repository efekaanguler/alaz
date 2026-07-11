#include <mission_control/mode_run.hpp>

RunMode::RunMode(rclcpp::Node::SharedPtr node) : node_(node) {
    
    goal_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        GOAL_PUBLISHER_TOPIC, 10);
    
    engage_publisher_ = node_->create_publisher<std_msgs::msg::Bool>(
        ENGAGE_PUBLISHER_TOPIC, 10);

    route_state_debug_publisher_ = node_->create_publisher<std_msgs::msg::UInt8>(
        ROUTE_STATE_DEBUG_PUBLISHER_TOPIC, 10);
    
    goal_array_subscriber_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
        GOAL_ARRAY_SUBSCRIBER_TOPIC, 10,
        std::bind(&RunMode::goal_array_callback, this, std::placeholders::_1));
    
    route_subscriber_ = node_->create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
        ROUTE_SUBSCRIBER_TOPIC, 10,
        std::bind(&RunMode::route_callback, this, std::placeholders::_1));
    
    trajectory_subscriber_ = node_->create_subscription<autoware_planning_msgs::msg::Trajectory>(
        TRAJECTORY_SUBSCRIBER_TOPIC, 10,
        std::bind(&RunMode::trajectory_callback, this, std::placeholders::_1));
    
    kinematics_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        KINEMATICS_SUBSCRIBER_TOPIC, 10,
        std::bind(&RunMode::kinematics_callback, this, std::placeholders::_1));
    
    control_cmd_subscriber_ = node_->create_subscription<autoware_control_msgs::msg::Control>(
        CONTROL_CMD_SUBSCRIBER_TOPIC, 10,
        std::bind(&RunMode::control_cmd_callback, this, std::placeholders::_1));
    
    emergency_subscriber_ = node_->create_subscription<std_msgs::msg::Bool>(
        EMERGENCY_SUBSCRIBER_TOPIC, 10,
        std::bind(&RunMode::emergency_callback, this, std::placeholders::_1));
    
    route_state_subscriber_ = node_->create_subscription<autoware_adapi_v1_msgs::msg::RouteState>(
        ROUTE_STATE_SUBSCRIBER_TOPIC, 10,
        std::bind(&RunMode::route_state_callback, this, std::placeholders::_1));
}

void RunMode::onEnter() {
    engaged_ = false; // Reset engagement so it republishes when re-entering
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
        RCLCPP_INFO(node_->get_logger(), "Autoware engaged - autonomous driving started");
        engaged_ = true;
    }
}

void RunMode::send_next_goal() {
    // Don't publish a new goal while Autoware is still processing the
    // previous route change - avoids overlapping/racing route requests.
    if (current_route_state_ == autoware_adapi_v1_msgs::msg::RouteState::CHANGING) {
        RCLCPP_INFO(node_->get_logger(), "Route still changing - waiting before sending next goal");
        return;
    }

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

void RunMode::control_cmd_callback(const autoware_control_msgs::msg::Control::SharedPtr msg) {
    target_velocity_ = msg->longitudinal.velocity;
    target_steering_ = msg->lateral.steering_tire_angle;
}

void RunMode::emergency_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    emergency_flag_ = msg->data;
}

void RunMode::route_state_callback(const autoware_adapi_v1_msgs::msg::RouteState::SharedPtr msg) {
    current_route_state_ = msg->state;

    // Publish the raw state for diagnostics/testing
    auto dbg_msg = std_msgs::msg::UInt8();
    dbg_msg.data = msg->state;
    route_state_debug_publisher_->publish(dbg_msg);

    // Guarded with goal_sent_current_ so a stale/late-arriving ARRIVED
    // (e.g. left over from a previous goal, if this topic is ever
    // republished with transient_local durability) can't prematurely mark
    // a goal we haven't actually sent yet as reached.
    if (msg->state == autoware_adapi_v1_msgs::msg::RouteState::ARRIVED && goal_sent_current_) {
        current_goal_reached_ = true;
        RCLCPP_INFO(node_->get_logger(), "Reached goal %zu/%zu", 
                    current_goal_index_ + 1, goal_array_.size());
    }
}

unsigned int RunMode::execute() {
    if (emergency_flag_) {
        RCLCPP_WARN(node_->get_logger(), "Emergency detected! Switching to EMERGENCY mode");
        return MODE_EMERGENCY; 
    }
    
    engage_autoware();
    
    if (goal_array_.empty()) {
        RCLCPP_INFO(node_->get_logger(), "No goal. Switching to Pause Mode");
        return MODE_PAUSE;
    }
    
    if (current_goal_reached_) {
        current_goal_index_++;
        goal_sent_current_ = false;
        
        if (current_goal_index_ >= goal_array_.size()) {
            RCLCPP_INFO(node_->get_logger(), "All goals completed! Switching to PAUSE mode");
            goal_array_.clear();
            current_goal_index_ = 0;
            return MODE_PAUSE;
        }

        RCLCPP_INFO(node_->get_logger(), "Goal reached. Switching to PAUSE before next goal");
        return MODE_PAUSE;
    }

    if (!goal_sent_current_) {
        send_next_goal();
    }
    
    return MODE_RUN; 
}