#include <mission_control/mode_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp"
#include <autoware_system_msgs/srv/change_autoware_control.hpp>
#include <autoware_system_msgs/srv/change_operation_mode.hpp>
#include <autoware_vehicle_msgs/msg/engage.hpp>
#include <std_msgs/msg/bool.hpp>

class EmergencyMode : public ModeBase
{
public:
  std::string LIDAR_TOPIC = "/sensing/scan";
  std::string GNSS_TOPIC = "";
  std::string IMU_TOPIC = "";
  std::string CAMERA_TOPIC = "/sensing/image_raw";
  std::string ODOM_TOPIC = "/odom";
  std::string EMERGENCY_PUBLISHER_TOPIC = "/mission_control/emergency_stop";
  std::string ENGAGE_PUBLISHER_TOPIC = "/autoware/engage";
  std::string CHANGE_OPERATION_MODE_SERVICE = "/system/operation_mode/change_operation_mode";
  std::string CHANGE_AUTOWARE_CONTROL_SERVICE = "/system/operation_mode/change_autoware_control";

  EmergencyMode(rclcpp::Node::SharedPtr node);
  unsigned int execute() override;
  bool isHealthy();

private:
  double TIMEOUT = 1.0;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Time last_lidar;
  rclcpp::Time last_gnss;
  rclcpp::Time last_imu;
  rclcpp::Time last_camera;
  rclcpp::Time last_odom;
  bool localization_seen = false;
  bool localization_initialized = false;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_subscriber;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscriber;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>::SharedPtr
    localization_subscriber;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_publisher_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::Engage>::SharedPtr engage_publisher_;
  rclcpp::Client<autoware_system_msgs::srv::ChangeOperationMode>::SharedPtr
    change_operation_mode_client_;
  rclcpp::Client<autoware_system_msgs::srv::ChangeAutowareControl>::SharedPtr
    change_autoware_control_client_;
  rclcpp::Time last_stop_request_;

  void lidar_callback(sensor_msgs::msg::LaserScan::SharedPtr msg);
  void gnss_callback(sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void imu_callback(sensor_msgs::msg::Imu::SharedPtr msg);
  void camera_callback(sensor_msgs::msg::Image::SharedPtr msg);
  void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg);
  void localization_callback(
    autoware_adapi_v1_msgs::msg::LocalizationInitializationState::SharedPtr msg);

  bool checkState();
};
