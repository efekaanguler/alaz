#include <mission_control/mode_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vector>

class RunMode : public ModeBase {
public:
    std::string GOAL_PUBLISHER_TOPIC="/planning/mission_planning/goal";
    std::string ENGAGE_PUBLISHER_TOPIC="/autoware/engage";
    std::string GOAL_ARRAY_SUBSCRIBER_TOPIC="/mission_control/goal_array";
    std::string ROUTE_SUBSCRIBER_TOPIC="/api/routing/route";
    std::string TRAJECTORY_SUBSCRIBER_TOPIC="/planning/scenario_planning/lane_driving/trajectory";
    std::string KINEMATICS_SUBSCRIBER_TOPIC="/api/vehicle/kinematics";
    std::string EMERGENCY_SUBSCRIBER_TOPIC="/api/autoware/get/emergency";
    
    RunMode(rclcpp::Node::SharedPtr node);
    unsigned int execute() override;

private:
    rclcpp::Node::SharedPtr node_;
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr engage_publisher_;
    
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goal_array_subscriber_;
    rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_subscriber_;
    rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr trajectory_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr kinematics_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_subscriber_;
    // TODO: velocity_subscriber_ and steering_subscriber_ when proper control message types are available
    // rclcpp::Subscription<autoware_control_msgs::msg::LongitudinalOutput>::SharedPtr velocity_subscriber_;
    // rclcpp::Subscription<autoware_control_msgs::msg::LateralOutput>::SharedPtr steering_subscriber_;
    // TODO: route_state_subscriber_ when LaneletRouteState message is available
    // rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRouteState>::SharedPtr route_state_subscriber_;
    
    // Data members
    std::vector<geometry_msgs::msg::Pose> goal_array_;
    size_t current_goal_index_ = 0;
    bool goal_sent_current_ = false;
    autoware_planning_msgs::msg::LaneletRoute current_route_;
    autoware_planning_msgs::msg::Trajectory current_trajectory_;
    geometry_msgs::msg::Twist vehicle_kinematics_;
    // TODO: target_velocity_ and target_steering_ when proper control message types are available
    // std_msgs::msg::Float64 target_velocity_;
    // std_msgs::msg::Float64 target_steering_;
    bool emergency_flag_ = false;
    bool engaged_ = false;
    bool current_goal_reached_ = false;
    
    // Callbacks
    void goal_array_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void route_callback(const autoware_planning_msgs::msg::LaneletRoute::SharedPtr msg);
    void trajectory_callback(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg);
    void kinematics_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void emergency_callback(const std_msgs::msg::Bool::SharedPtr msg);
    // void velocity_callback(const autoware_control_msgs::msg::LongitudinalOutput::SharedPtr msg);
    // void steering_callback(const autoware_control_msgs::msg::LateralOutput::SharedPtr msg);
    // void route_state_callback(const autoware_planning_msgs::msg::LaneletRouteState::SharedPtr msg);
    
    void send_next_goal();
    void engage_autoware();
};