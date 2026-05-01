#include <mission_control/mode_start.hpp>

StartMode::StartMode(rclcpp::Node::SharedPtr node) : node_(node) {
    
    if(!LIDAR_TOPIC.empty())
        lidar_subscriber = node_->create_subscription<sensor_msgs::msg::LaserScan>(LIDAR_TOPIC, 10, std::bind(&StartMode::lidar_callback, this, std::placeholders::_1));
    
    if(!GNSS_TOPIC.empty())
        gnss_subscriber = node_->create_subscription<sensor_msgs::msg::NavSatFix>(GNSS_TOPIC, 10, std::bind(&StartMode::gnss_callback, this, std::placeholders::_1));
    
    if(!IMU_TOPIC.empty())
        imu_subscriber = node_->create_subscription<sensor_msgs::msg::Imu>(IMU_TOPIC, 10, std::bind(&StartMode::imu_callback, this, std::placeholders::_1));
    
    if(!CAMERA_TOPIC.empty())
        camera_subscriber = node_->create_subscription<sensor_msgs::msg::Image>(CAMERA_TOPIC, 10, std::bind(&StartMode::camera_callback, this, std::placeholders::_1));
    
    if(!ODOM_TOPIC.empty())
        odom_subscriber = node_->create_subscription<nav_msgs::msg::Odometry>(ODOM_TOPIC, 10, std::bind(&StartMode::odom_callback, this, std::placeholders::_1));
    
    localization_subscriber = node->create_subscription<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>("/localization/initialization_state", 10, std::bind(&StartMode::localization_callback, this, std::placeholders::_1));

    localized = true;
}


unsigned int StartMode::execute() {

    bool returnStart=false;
    
    if(!LIDAR_TOPIC.empty() && !lidar_read) {
        RCLCPP_ERROR(node_->get_logger(), "No Lidar Data");
        returnStart = true;
    }
    if(!GNSS_TOPIC.empty() && !gnss_read) {
        RCLCPP_ERROR(node_->get_logger(), "No GNSS Data");
        returnStart = true;

    }
    if(!IMU_TOPIC.empty() && !imu_read) {
        RCLCPP_ERROR(node_->get_logger(), "No IMU Data");
        returnStart = true;
    }
    if(!CAMERA_TOPIC.empty() && !camera_read) {
        RCLCPP_ERROR(node_->get_logger(), "No Camera Data");
        returnStart = true;

    }
    if(!ODOM_TOPIC.empty() && !odom_read) {
        RCLCPP_ERROR(node_->get_logger(), "No Odometry Data");
        returnStart = true;
    }

    if(!returnStart) RCLCPP_DEBUG(node_->get_logger(), "All Sensors Checked");

    if(!localized) {
        RCLCPP_ERROR(node_->get_logger(), "Localization Failed");
        returnStart=true;
    } else {
        RCLCPP_DEBUG(node_->get_logger(), "Localization Successfull");
    }

    if(returnStart) return MODE_START;

    RCLCPP_INFO(node_->get_logger(), "Vehicle Started, Entering Pause Mode");
    return MODE_PAUSE;
}


void StartMode::lidar_callback(sensor_msgs::msg::LaserScan::SharedPtr msg) {
    (void)msg;
    lidar_read=true;
}

void StartMode::gnss_callback(sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    (void)msg;
    gnss_read=true;
}

void StartMode::imu_callback(sensor_msgs::msg::Imu::SharedPtr msg) {
    (void)msg;
    imu_read=true;
}

void StartMode::camera_callback(sensor_msgs::msg::Image::SharedPtr msg) {
    (void)msg;
    camera_read=true;
}

void StartMode::odom_callback(nav_msgs::msg::Odometry::SharedPtr msg) {
    (void)msg;
    odom_read=true;
}

void StartMode::localization_callback(autoware_adapi_v1_msgs::msg::LocalizationInitializationState::SharedPtr msg) {
    if(msg->state == autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZED) localized=true;
    else localized=false;
}
