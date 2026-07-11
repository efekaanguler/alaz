#include <mission_control/mode_run.hpp>

#include <cmath>

RunMode::RunMode(rclcpp::Node::SharedPtr node)
: node_(node)
{
  last_engage_attempt_ = node_->now() - rclcpp::Duration::from_seconds(10.0);
  last_route_attempt_ = node_->now() - rclcpp::Duration::from_seconds(10.0);

  // Publishers to command autoware
  engage_publisher_ = node_->create_publisher<autoware_vehicle_msgs::msg::Engage>(
    ENGAGE_PUBLISHER_TOPIC, 10);

  change_operation_mode_client_ =
    node_->create_client<autoware_system_msgs::srv::ChangeOperationMode>(
    CHANGE_OPERATION_MODE_SERVICE);
  change_autoware_control_client_ =
    node_->create_client<autoware_system_msgs::srv::ChangeAutowareControl>(
    CHANGE_AUTOWARE_CONTROL_SERVICE);

  set_route_client_ =
    node_->create_client<autoware_internal_planning_msgs::srv::SetWaypointRoute>(
    SET_ROUTE_SERVICE);

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

  operation_mode_subscriber_ = node_->create_subscription<
    autoware_adapi_v1_msgs::msg::OperationModeState>(
    "/system/operation_mode/state", rclcpp::QoS(1).transient_local().reliable(),
    std::bind(
      &RunMode::operation_mode_callback, this,
      std::placeholders::_1));

  // TODO: Subscribe to routing state to check if goal reached
  // route_state_subscriber_ = node_->create_subscription<autoware_planning_msgs::msg::LaneletRouteState>(
  //     "/api/routing/state", 10,
  //     std::bind(&RunMode::route_state_callback, this, std::placeholders::_1));
}

void RunMode::goal_array_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  goal_array_ = msg->poses;
  current_goal_index_ = 0;
  current_goal_reached_ = false;
  goal_sent_current_ = false;
  route_request_in_flight_ = false;
  has_trajectory_ = false;
  RCLCPP_INFO(node_->get_logger(), "Received %zu goals", goal_array_.size());
}

void RunMode::engage_autoware()
{
  if (engaged_) {
    return;
  }

  const auto now = node_->now();
  if ((now - last_engage_attempt_).seconds() < 1.0) {
    return;
  }
  last_engage_attempt_ = now;

  if (!operation_mode_state_seen_ ||
    current_operation_mode_ !=
    autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS)
  {
    if (!change_operation_mode_client_->service_is_ready()) {
      auto engage_msg = autoware_vehicle_msgs::msg::Engage();
      engage_msg.stamp = now;
      engage_msg.engage = true;
      engage_publisher_->publish(engage_msg);
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "Operation-mode service is not ready; legacy engage remains active");
      return;
    }
    if (operation_mode_request_in_flight_) {
      return;
    }

    auto request = std::make_shared<
      autoware_system_msgs::srv::ChangeOperationMode::Request>();
    request->mode =
      autoware_system_msgs::srv::ChangeOperationMode::Request::AUTONOMOUS;
    operation_mode_request_in_flight_ = true;
    change_operation_mode_client_->async_send_request(
      request,
      [this](rclcpp::Client<autoware_system_msgs::srv::ChangeOperationMode>::SharedFuture future) {
        operation_mode_request_in_flight_ = false;
        try {
          const auto response = future.get();
          if (!response->status.success) {
            RCLCPP_WARN(
              node_->get_logger(),
              "Autonomous mode request rejected: %s",
              response->status.message.c_str());
          }
        } catch (const std::exception & error) {
          RCLCPP_ERROR(
            node_->get_logger(),
            "Autonomous mode request failed: %s", error.what());
        }
      });
    RCLCPP_INFO(node_->get_logger(), "Requested autonomous operation mode");
    return;
  }

  if (operation_mode_in_transition_) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Waiting for operation-mode transition to complete");
    return;
  }

  if (!autoware_control_enabled_ && !autoware_control_request_in_flight_) {
    if (!change_autoware_control_client_->service_is_ready()) {
      auto engage_msg = autoware_vehicle_msgs::msg::Engage();
      engage_msg.stamp = now;
      engage_msg.engage = true;
      engage_publisher_->publish(engage_msg);
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "Autoware-control service is not ready; legacy engage remains active");
      return;
    }

    auto request = std::make_shared<
      autoware_system_msgs::srv::ChangeAutowareControl::Request>();
    request->autoware_control = true;
    autoware_control_request_in_flight_ = true;
    change_autoware_control_client_->async_send_request(
      request,
      [this](rclcpp::Client<
        autoware_system_msgs::srv::ChangeAutowareControl>::SharedFuture
      future) {
        autoware_control_request_in_flight_ = false;
        try {
          const auto response = future.get();
          if (!response->status.success) {
            RCLCPP_WARN(
              node_->get_logger(),
              "Autoware-control request rejected: %s",
              response->status.message.c_str());
          }
        } catch (const std::exception & error) {
          RCLCPP_ERROR(
            node_->get_logger(),
            "Autoware-control request failed: %s",
            error.what());
        }
      });
    RCLCPP_INFO(node_->get_logger(), "Requested Autoware control enable");
  } else if (!autoware_control_enabled_) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Waiting for Autoware control enable response");
  }
}

void RunMode::send_next_goal()
{
  if (current_goal_index_ >= goal_array_.size() || route_request_in_flight_) {
    return;
  }

  const auto now = node_->now();
  if ((now - last_route_attempt_).seconds() < 1.0) {
    return;
  }
  last_route_attempt_ = now;

  if (!set_route_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Planning route service is not ready: %s", SET_ROUTE_SERVICE.c_str());
    return;
  }

  auto request = std::make_shared<
    autoware_internal_planning_msgs::srv::SetWaypointRoute::Request>();
  request->header.stamp = now;
  request->header.frame_id = "map";
  request->goal_pose = goal_array_[current_goal_index_];
  request->allow_modification = false;

  const auto stamp = static_cast<uint64_t>(now.nanoseconds());
  const auto index = static_cast<uint64_t>(current_goal_index_);
  for (size_t i = 0; i < 8; ++i) {
    request->uuid.uuid[i] = static_cast<uint8_t>((stamp >> (8 * i)) & 0xFFU);
    request->uuid.uuid[i + 8] =
      static_cast<uint8_t>((index >> (8 * i)) & 0xFFU);
  }

  route_request_in_flight_ = true;
  set_route_client_->async_send_request(
    request,
    [this](rclcpp::Client<
      autoware_internal_planning_msgs::srv::SetWaypointRoute>::SharedFuture
    future) {
      route_request_in_flight_ = false;
      try {
        const auto response = future.get();
        if (!response->status.success) {
          RCLCPP_WARN(
            node_->get_logger(), "Route request rejected: %s",
            response->status.message.c_str());
          return;
        }
        current_goal_reached_ = false;
        goal_sent_current_ = true;
        RCLCPP_INFO(
          node_->get_logger(), "Route accepted for goal %zu/%zu",
          current_goal_index_ + 1, goal_array_.size());
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          node_->get_logger(), "Route request failed: %s",
          error.what());
      }
    });
}

void RunMode::route_callback(const autoware_planning_msgs::msg::LaneletRoute::SharedPtr msg)
{
  current_route_ = *msg;
}

void RunMode::trajectory_callback(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg)
{
  current_trajectory_ = *msg;
  has_trajectory_ = !msg->points.empty();
}

void RunMode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
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

void RunMode::emergency_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  emergency_flag_ = msg->data;
}

void RunMode::operation_mode_callback(
  const autoware_adapi_v1_msgs::msg::OperationModeState::SharedPtr msg)
{
  operation_mode_state_seen_ = true;
  current_operation_mode_ = msg->mode;
  autoware_control_enabled_ = msg->is_autoware_control_enabled;
  operation_mode_in_transition_ = msg->is_in_transition;
  engaged_ =
    msg->mode == autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS &&
    msg->is_autoware_control_enabled && !msg->is_in_transition;
}

// TODO: Implement route state callback when LaneletRouteState message becomes available
// void RunMode::route_state_callback(const autoware_planning_msgs::msg::LaneletRouteState::SharedPtr msg) {
//     if (msg->state == autoware_planning_msgs::msg::LaneletRouteState::ARRIVED) {
//         current_goal_reached_ = true;
//         RCLCPP_INFO(node_->get_logger(), "Reached goal %zu/%zu",
//                     current_goal_index_ + 1, goal_array_.size());
//     }
// }

void RunMode::update_goal_reached()
{
  if (!goal_sent_current_ || current_goal_reached_ || !has_current_pose_) {
    return;
  }
  if (current_goal_index_ >= goal_array_.size()) {
    return;
  }

  const auto & goal = goal_array_[current_goal_index_];
  const double dx = current_pose_.position.x - goal.position.x;
  const double dy = current_pose_.position.y - goal.position.y;
  const double distance = std::hypot(dx, dy);

  if (distance <= goal_reach_distance_m_) {
    current_goal_reached_ = true;
    RCLCPP_INFO(
      node_->get_logger(), "Reached goal %zu/%zu (distance %.2f m)",
      current_goal_index_ + 1, goal_array_.size(), distance);
  }
}

unsigned int RunMode::execute()
{
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
    return MODE_RUN;
  }

  if (!has_trajectory_) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Waiting for a non-empty /planning/trajectory before engage");
    return MODE_RUN;
  }

  engage_autoware();

  return MODE_RUN;
}
