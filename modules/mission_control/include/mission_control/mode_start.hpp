#ifndef MODE_START_HPP
#define MODE_START_HPP

#include <mission_control/mode_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>

class StartMode : public ModeBase {
    public:
    std::string LIDAR_TOPIC="/sensing/lidar/top/scan";
    std::string GNSS_TOPIC="";
    std::string IMU_TOPIC="";
    std::string CAMERA_TOPIC="/sensing/camera/camera0/image_raw";
    std::string ODOM_TOPIC="/odom";
    
    // FIXED: Now correctly points to the ADAPI topic matching the message type
    std::string LOCALIZATION_TOPIC="/localization/kinematic_state";

    StartMode(rclcpp::Node::SharedPtr node);
    unsigned int execute() override;

    private:
    rclcpp::Node::SharedPtr node_;
    bool lidar_read=false;
    bool gnss_read=false;
    bool imu_read=false;
    bool camera_read=false;
    bool odom_read=false;
    bool localized=false;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr localization_subscriber;

    void lidar_callback(sensor_msgs::msg::LaserScan::SharedPtr msg);
    void gnss_callback(sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void imu_callback(sensor_msgs::msg::Imu::SharedPtr msg);
    void camera_callback(sensor_msgs::msg::Image::SharedPtr msg);
    void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg);
    void localization_callback(nav_msgs::msg::Odometry::SharedPtr msg);
};

#endif // MODE_START_HPP
