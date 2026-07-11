#ifndef MODE_EMERGENCY_HPP
#define MODE_EMERGENCY_HPP

#include <mission_control/mode_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
class EmergencyMode : public ModeBase {
public:
    std::string LIDAR_TOPIC="/sensing/lidar/top/scan";
    std::string GNSS_TOPIC="";
    std::string IMU_TOPIC="";
    std::string CAMERA_TOPIC="/sensing/camera/camera0/image_raw";
    std::string ODOM_TOPIC="/odom";
    std::string LOCALIZATION_TOPIC="/localization/kinematic_state";
    std::string EMERGENCY_PUBLISHER_TOPIC="/mission_control/emergency_stop";
    EmergencyMode(rclcpp::Node::SharedPtr node);
    unsigned int execute() override;
    // Side-effect-free status check (no publishing, no logging)
    bool isEmergencyTriggered();
    void publishEmergencyStop(bool active);
    void requestReset();
    private:
    bool reset_requested_ = false;
    double TIMEOUT = 1.0;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Time last_lidar;
    rclcpp::Time last_gnss;
    rclcpp::Time last_imu;
    rclcpp::Time last_camera;
    rclcpp::Time last_odom;
    rclcpp::Time last_localized;
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr localization_subscriber;
    
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_publisher_;
    void lidar_callback(sensor_msgs::msg::LaserScan::SharedPtr msg);
    void gnss_callback(sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void imu_callback(sensor_msgs::msg::Imu::SharedPtr msg);
    void camera_callback(sensor_msgs::msg::Image::SharedPtr msg);
    void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg);
    void localization_callback(nav_msgs::msg::Odometry::SharedPtr msg);
    bool checkState();
};

#endif
