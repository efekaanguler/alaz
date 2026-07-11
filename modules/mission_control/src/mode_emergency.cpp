#include <mission_control/mode_emergency.hpp>

EmergencyMode::EmergencyMode(rclcpp::Node::SharedPtr node) : node_(node) {
    if(!LIDAR_TOPIC.empty())
        lidar_subscriber = node_->create_subscription<sensor_msgs::msg::LaserScan>(LIDAR_TOPIC, 10, std::bind(&EmergencyMode::lidar_callback, this, std::placeholders::_1));
    if(!GNSS_TOPIC.empty())
        gnss_subscriber = node_->create_subscription<sensor_msgs::msg::NavSatFix>(GNSS_TOPIC, 10, std::bind(&EmergencyMode::gnss_callback, this, std::placeholders::_1));
    if(!IMU_TOPIC.empty())
        imu_subscriber = node_->create_subscription<sensor_msgs::msg::Imu>(IMU_TOPIC, 10, std::bind(&EmergencyMode::imu_callback, this, std::placeholders::_1));
    if(!CAMERA_TOPIC.empty())
        camera_subscriber = node_->create_subscription<sensor_msgs::msg::Image>(CAMERA_TOPIC, 10, std::bind(&EmergencyMode::camera_callback, this, std::placeholders::_1));
    if(!ODOM_TOPIC.empty())
        odom_subscriber = node_->create_subscription<nav_msgs::msg::Odometry>(ODOM_TOPIC, 10, std::bind(&EmergencyMode::odom_callback, this, std::placeholders::_1));
    if(!LOCALIZATION_TOPIC.empty())
        localization_subscriber = node_->create_subscription<nav_msgs::msg::Odometry>(LOCALIZATION_TOPIC, 10, std::bind(&EmergencyMode::localization_callback, this, std::placeholders::_1));
    
    emergency_publisher_ = node_->create_publisher<std_msgs::msg::Bool>(
        EMERGENCY_PUBLISHER_TOPIC,
        rclcpp::QoS(1).reliable().transient_local());
}

bool EmergencyMode::isEmergencyTriggered() {
    auto now = node_->now();

    if(!LIDAR_TOPIC.empty() && (last_lidar.nanoseconds()==0 || (now-last_lidar).seconds() > TIMEOUT)) return true;
    if(!GNSS_TOPIC.empty() && (last_gnss.nanoseconds()==0 || (now-last_gnss).seconds() > TIMEOUT)) return true;
    if(!IMU_TOPIC.empty() && (last_imu.nanoseconds()==0 || (now-last_imu).seconds() > TIMEOUT)) return true;
    if(!CAMERA_TOPIC.empty() && (last_camera.nanoseconds()==0 || (now-last_camera).seconds() > TIMEOUT)) return true;
    if(!ODOM_TOPIC.empty() && (last_odom.nanoseconds()==0 || (now-last_odom).seconds() > TIMEOUT)) return true;
    if(!LOCALIZATION_TOPIC.empty() && (last_localized.nanoseconds()==0 || (now-last_localized).seconds() > TIMEOUT)) return true;

    return false;
}

void EmergencyMode::requestReset() {
    reset_requested_ = true;
}

unsigned int EmergencyMode::execute() {
    if(checkState()) {
        if (reset_requested_) {
            reset_requested_ = false;
            RCLCPP_INFO(node_->get_logger(), "Emergency cleared by operator. Switching to PAUSE mode.");
            return MODE_PAUSE;
        } else {
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "Emergency topics healthy, waiting for manual reset...");
            return MODE_EMERGENCY;
        }
    } else {
        reset_requested_ = false;
        return MODE_EMERGENCY;
    }
}

void EmergencyMode::publishEmergencyStop(bool active) {
    auto msg = std_msgs::msg::Bool();
    msg.data = active;
    emergency_publisher_->publish(msg);
}

bool EmergencyMode::checkState() {
    auto now = node_->now();
    bool state = true;

    if(!LIDAR_TOPIC.empty() && (last_lidar.nanoseconds()==0 || (now-last_lidar).seconds() > TIMEOUT)) {
        RCLCPP_ERROR(node_->get_logger(), "EMERGENCY: No Lidar Data");
        state = false;
    }
    if(!GNSS_TOPIC.empty() && (last_gnss.nanoseconds()==0 || (now-last_gnss).seconds() > TIMEOUT)) {
        RCLCPP_ERROR(node_->get_logger(), "EMERGENCY: No GNSS Data");
        state = false;
    }
    if(!IMU_TOPIC.empty() && (last_imu.nanoseconds()==0 || (now-last_imu).seconds() > TIMEOUT)) {
        RCLCPP_ERROR(node_->get_logger(), "EMERGENCY: No IMU Data");
        state = false;
    }
    if(!CAMERA_TOPIC.empty() && (last_camera.nanoseconds()==0 || (now-last_camera).seconds() > TIMEOUT)) {
        RCLCPP_ERROR(node_->get_logger(), "EMERGENCY: No Camera Data");
        state = false;
    }
    if(!ODOM_TOPIC.empty() && (last_odom.nanoseconds()==0 || (now-last_odom).seconds() > TIMEOUT)) {
        RCLCPP_ERROR(node_->get_logger(), "EMERGENCY: No Odometry Data");
        state = false;
    }
    if(!LOCALIZATION_TOPIC.empty() && (last_localized.nanoseconds()==0 || (now-last_localized).seconds() > TIMEOUT)) {
        RCLCPP_ERROR(node_->get_logger(), "EMERGENCY: Localization Failed");
        state = false;
    }
    return state;
}

void EmergencyMode::lidar_callback(sensor_msgs::msg::LaserScan::SharedPtr msg) {
    (void)msg;
    last_lidar = node_->now();
}
void EmergencyMode::gnss_callback(sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    (void)msg;
    last_gnss = node_->now();
}
void EmergencyMode::imu_callback(sensor_msgs::msg::Imu::SharedPtr msg) {
    (void)msg;
    last_imu = node_->now();
}
void EmergencyMode::camera_callback(sensor_msgs::msg::Image::SharedPtr msg) {
    (void)msg;
    last_camera = node_->now();
}
void EmergencyMode::odom_callback(nav_msgs::msg::Odometry::SharedPtr msg) {
    (void)msg;
    last_odom = node_->now();
}
void EmergencyMode::localization_callback(nav_msgs::msg::Odometry::SharedPtr msg) {
    (void)msg;
    last_localized = node_->now();
}
