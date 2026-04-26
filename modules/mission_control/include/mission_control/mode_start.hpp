#include <mission_control/mode_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp"


class StartMode : public ModeBase {
    public:
    std::string LIDAR_TOPIC="/scan";
    std::string GNSS_TOPIC="";
    std::string IMU_TOPIC="";
    std::string CAMERA_TOPIC="/image_raw";
    std::string ODOM_TOPIC="/odom";

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
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp::Subscription<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>::SharedPtr localization_subscriber;

    void lidar_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void gnss_callback(sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void imu_callback(sensor_msgs::msg::Imu::SharedPtr msg);
    void camera_callback(sensor_msgs::msg::Image::SharedPtr msg);
    void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg);
    void localization_callback(autoware_adapi_v1_msgs::msg::LocalizationInitializationState::SharedPtr msg);

};
