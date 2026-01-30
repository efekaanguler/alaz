#include <mission_control/mode_start.hpp>

StartMode::StartMode(rclcpp::Node::SharedPtr node) : node_(node) {
    
    if(LIDAR_TOPIC.empty())
        lidar_subscriber = node_->create_subscription<sensor_msgs::msg::PointCloud2>(LIDAR_TOPIC, 10, std::bind(&StartMode::lidar_callback, this, std::placeholders::_1));
    
    if(!GNSS_TOPIC.empty())
        gnss_subscriber = node_->create_subscription<sensor_msgs::msg::NavSatFix>(GNSS_TOPIC, 10, std::bind(&StartMode::gnss_callback, this, std::placeholders::_1));
    
    if(!IMU_TOPIC.empty())
        imu_subscriber = node_->create_subscription<sensor_msgs::msg::Imu>(IMU_TOPIC, 10, std::bind(&StartMode::imu_callback, this, std::placeholders::_1));
    
    if(!CAMERA_TOPIC.empty())
        camera_subscriber = node_->create_subscription<sensor_msgs::msg::Image>(CAMERA_TOPIC, 10, std::bind(&StartMode::camera_callback, this, std::placeholders::_1));
    
    if(!ODOM_TOPIC.empty())
        odom_subscriber = node_->create_subscription<nav_msgs::msg::Odometry>(ODOM_TOPIC, 10, std::bind(&StartMode::odom_callback, this, std::placeholders::_1));
    
    localization_subscriber = node->create_subscription<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>("/localization/initialization_state", 10, std::bind(&StartMode::localization_callback, this, std::placeholders::_1));

}


unsigned int StartMode::execute() {
    
    if(!LIDAR_TOPIC.empty() && !lidar_read) {
        RCLCPP_INFO(node_->get_logger(), "No Lidar Data");
        return MODE_START;
    }
    if(!GNSS_TOPIC.empty() && !gnss_read) {
        RCLCPP_INFO(node_->get_logger(), "No GNSS Data");
        return MODE_START;
    }
    if(!IMU_TOPIC.empty() && !imu_read) {
        RCLCPP_INFO(node_->get_logger(), "No IMU Data");
        return MODE_START;
    }
    if(!CAMERA_TOPIC.empty() && !camera_read) {
        RCLCPP_INFO(node_->get_logger(), "No Camera Data");
        return MODE_START;
    }
    if(!ODOM_TOPIC.empty() && !odom_read) {
        RCLCPP_INFO(node_->get_logger(), "No Odometry Data");
        return MODE_START;
    }

    RCLCPP_INFO(node_->get_logger(), "All Sensors Checked");

    if(!localized) {
        RCLCPP_INFO(node_->get_logger(), "Localization Failed");
        return MODE_START;
    }

    RCLCPP_INFO(node_->get_logger(), "Localization Successfull");
    RCLCPP_INFO(node_->get_logger(), "Vehicle Started, Entering Pause Mode");

    return MODE_PAUSE;
}


void StartMode::lidar_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    lidar_read=true;
}

void StartMode::gnss_callback(sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    gnss_read=true;
}

void StartMode::imu_callback(sensor_msgs::msg::Imu::SharedPtr msg) {
    imu_read=true;
}

void StartMode::camera_callback(sensor_msgs::msg::Image::SharedPtr msg) {
    camera_read=true;
}

void StartMode::odom_callback(nav_msgs::msg::Odometry::SharedPtr msg) {
    odom_read=true;
}

void StartMode::localization_callback(autoware_adapi_v1_msgs::msg::LocalizationInitializationState::SharedPtr msg) {
    if(msg->state == autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZED) localized=true;
    else localized=false;
}