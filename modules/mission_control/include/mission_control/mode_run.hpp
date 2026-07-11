#include <mission_control/mode_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_internal_planning_msgs/srv/set_waypoint_route.hpp>
#include <autoware_system_msgs/srv/change_autoware_control.hpp>
#include <autoware_system_msgs/srv/change_operation_mode.hpp>
#include <autoware_vehicle_msgs/msg/engage.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vector>

class RunMode : public ModeBase
{
public:
  std::string SET_ROUTE_SERVICE = "/planning/set_waypoint_route";
  std::string ENGAGE_PUBLISHER_TOPIC = "/autoware/engage";
  std::string GOAL_ARRAY_SUBSCRIBER_TOPIC = "/mission_control/goal_array";
  std::string ROUTE_SUBSCRIBER_TOPIC = "/planning/mission_planning/route";
  std::string TRAJECTORY_SUBSCRIBER_TOPIC = "/planning/trajectory";
  std::string ODOM_SUBSCRIBER_TOPIC = "/localization/kinematic_state";
  std::string EMERGENCY_SUBSCRIBER_TOPIC = "/mission_control/emergency_stop";
  std::string CHANGE_OPERATION_MODE_SERVICE = "/system/operation_mode/change_operation_mode";
  std::string CHANGE_AUTOWARE_CONTROL_SERVICE = "/system/operation_mode/change_autoware_control";

  RunMode(rclcpp::Node::SharedPtr node);
  unsigned int execute() override;

private:
  rclcpp::Node::SharedPtr node_;

  // Publishers
  rclcpp::Publisher<autoware_vehicle_msgs::msg::Engage>::SharedPtr engage_publisher_;
  rclcpp::Client<autoware_system_msgs::srv::ChangeOperationMode>::SharedPtr
    change_operation_mode_client_;
  rclcpp::Client<autoware_system_msgs::srv::ChangeAutowareControl>::SharedPtr
    change_autoware_control_client_;
  rclcpp::Client<autoware_internal_planning_msgs::srv::SetWaypointRoute>::SharedPtr
    set_route_client_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goal_array_subscriber_;
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_subscriber_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr trajectory_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_subscriber_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr
    operation_mode_subscriber_;
  // TODO: velocity_subscriber_ and steering_subscriber_ when proper control message types are available
  // rclcpp::Subscription<autoware_control_msgs::msg::LongitudinalOutput>::SharedPtr velocity_subscriber_;
  // rclcpp::Subscription<autoware_control_msgs::msg::LateralOutput>::SharedPtr steering_subscriber_;
  // TODO: route_state_subscriber_ when LaneletRouteState message is available
  // rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRouteState>::SharedPtr route_state_subscriber_;

  // Data members
  std::vector<geometry_msgs::msg::Pose> goal_array_;
  size_t current_goal_index_ = 0;
  bool goal_sent_current_ = false;
  bool route_request_in_flight_ = false;
  autoware_planning_msgs::msg::LaneletRoute current_route_;
  autoware_planning_msgs::msg::Trajectory current_trajectory_;
  geometry_msgs::msg::Pose current_pose_;
  // TODO: target_velocity_ and target_steering_ when proper control message types are available
  // std_msgs::msg::Float64 target_velocity_;
  // std_msgs::msg::Float64 target_steering_;
  bool emergency_flag_ = false;
  bool engaged_ = false;
  bool operation_mode_request_in_flight_ = false;
  bool autoware_control_request_in_flight_ = false;
  bool operation_mode_state_seen_ = false;
  uint16_t current_operation_mode_ = autoware_adapi_v1_msgs::msg::OperationModeState::STOP;
  bool autoware_control_enabled_ = false;
  bool operation_mode_in_transition_ = false;
  bool has_trajectory_ = false;
  bool current_goal_reached_ = false;
  bool has_current_pose_ = false;
  double goal_reach_distance_m_ = 1.0;
  rclcpp::Time last_engage_attempt_;
  rclcpp::Time last_route_attempt_;

  // Callbacks
  void goal_array_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void route_callback(const autoware_planning_msgs::msg::LaneletRoute::SharedPtr msg);
  void trajectory_callback(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void emergency_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void operation_mode_callback(
    const autoware_adapi_v1_msgs::msg::OperationModeState::SharedPtr msg);
  // void velocity_callback(const autoware_control_msgs::msg::LongitudinalOutput::SharedPtr msg);
  // void steering_callback(const autoware_control_msgs::msg::LateralOutput::SharedPtr msg);
  // void route_state_callback(const autoware_planning_msgs::msg::LaneletRouteState::SharedPtr msg);

  void send_next_goal();
  void engage_autoware();
  void update_goal_reached();
};
