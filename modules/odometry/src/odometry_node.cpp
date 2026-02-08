#include <odometry_node.hpp>

using namespace std::chrono_literals;

OdometryNode::OdometryNode() : Node("odometry_node"), odom(now()){

    speed_subscriber = create_subscription<std_msgs::msg::Float32>(SPEED_TOPIC, 10, std::bind(&OdometryNode::speed_callback, this, std::placeholders::_1));
    steering_subscriber = create_subscription<std_msgs::msg::Float32>(STEERING_TOPIC, 10, std::bind(&OdometryNode::steering_callback, this, std::placeholders::_1));
    
    odom_publisher = create_publisher<nav_msgs::msg::Odometry>(ODOM_TOPIC, 10);
    publish_timer = create_wall_timer(500ms, std::bind(&OdometryNode::publish_odometry, this));
}

void OdometryNode::speed_callback(std_msgs::msg::Float32 msg) {
    speed = msg.data;
    last_speed = now();
}

void OdometryNode::steering_callback(std_msgs::msg::Float32 msg) {
    steering = msg.data;
    last_steering = now();
}

void OdometryNode::publish_odometry() {
    auto now_ = now();
    if(last_speed.nanoseconds()==0 || (now_-last_speed).seconds() > TIMEOUT) return;
    if(last_steering.nanoseconds()==0 || (now_-last_steering).seconds() > TIMEOUT) return;

    odom.update_odometry(speed, steering, now_);

    tf2::Quaternion q;
    q.setRPY(0, 0, odom.th); // Create quaternion from yaw
    geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q); // Convert to msg

    nav_msgs::msg::Odometry odom_;
    odom_.header.stamp = odom.stamp;
    odom_.header.frame_id = "odom";

    // set the position
    odom_.pose.pose.position.x = odom.x;
    odom_.pose.pose.position.y = odom.y;
    odom_.pose.pose.position.z = 0.0;
    odom_.pose.pose.orientation = odom_quat;

    // set the velocity
    odom_.child_frame_id = "base_link";
    odom_.twist.twist.linear.x = speed;
    odom_.twist.twist.angular.z = steering;

    odom_publisher->publish(odom_);

}