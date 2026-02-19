#include <mission_control/mode_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp"

class EmergencyMode : public ModeBase {
public:
    std::string LIDAR_TOPIC="/carla/ego_vehicle/lidar_2d";
    std::string GNSS_TOPIC="";
    std::string IMU_TOPIC="";
    std::string CAMERA_TOPIC="/carla/ego_vehicle/front_camera/image";
    std::string ODOM_TOPIC="/odom";

    EmergencyMode(rclcpp::Node::SharedPtr node);
    unsigned int execute() override;

    private:

    double TIMEOUT = 1.0;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Time last_lidar;
    rclcpp::Time last_gnss;
    rclcpp::Time last_imu;
    rclcpp::Time last_camera;
    rclcpp::Time last_odom;
    rclcpp::Time last_localized;
    
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

    bool checkState();
};
