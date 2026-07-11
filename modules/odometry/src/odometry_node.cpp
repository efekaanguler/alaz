#include <odometry_node.hpp>

using namespace std::chrono_literals;

OdometryNode::OdometryNode() : Node("odometry_node"), odom(now()){

    speed_subscriber = create_subscription<std_msgs::msg::Float32>(SPEED_TOPIC, 10, std::bind(&OdometryNode::speed_callback, this, std::placeholders::_1));
    steering_subscriber = create_subscription<std_msgs::msg::Float32>(STEERING_TOPIC, 10, std::bind(&OdometryNode::steering_callback, this, std::placeholders::_1));
    velocity_report_subscriber = create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(VELOCITY_REPORT_TOPIC, 10, std::bind(&OdometryNode::velocity_report_callback, this, std::placeholders::_1));
    steering_report_subscriber = create_subscription<autoware_vehicle_msgs::msg::SteeringReport>(STEERING_REPORT_TOPIC, 10, std::bind(&OdometryNode::steering_report_callback, this, std::placeholders::_1));
    throttle_subscriber = create_subscription<std_msgs::msg::Float32>(THROTTLE_TOPIC, 10, std::bind(&OdometryNode::throttle_callback, this, std::placeholders::_1));

    odom_publisher = create_publisher<nav_msgs::msg::Odometry>(ODOM_TOPIC, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    publish_timer = create_wall_timer(50ms, std::bind(&OdometryNode::publish_odometry, this));
}

float OdometryNode::simulate_speed() {
    float MOTOR_GAIN=0.008, MOTOR_BIAS=0.1;
    float estimated_speed = MOTOR_GAIN * throttle + MOTOR_BIAS;

    if(estimated_speed < 0.0) estimated_speed=0.0;
    if(estimated_speed > 1.5) estimated_speed=1.5;
    return estimated_speed;
}

void OdometryNode::speed_callback(std_msgs::msg::Float32 msg) {
    speed = msg.data;
    if(speed == 0 && throttle > 0) {
        speed = simulate_speed();
    }
    last_speed = now();
}

void OdometryNode::steering_callback(std_msgs::msg::Float32 msg) {
    steering = msg.data;
    last_steering = now();
}

void OdometryNode::velocity_report_callback(autoware_vehicle_msgs::msg::VelocityReport msg) {
    speed = msg.longitudinal_velocity;
    last_speed = now();
}

void OdometryNode::steering_report_callback(autoware_vehicle_msgs::msg::SteeringReport msg) {
    steering = msg.steering_tire_angle;
    last_steering = now();
}

void OdometryNode::throttle_callback(std_msgs::msg::Float32 msg) {
    throttle = msg.data;
}

void OdometryNode::publish_odometry() {
    auto now_ = now();
    if(last_speed.nanoseconds()==0 || (now_-last_speed).seconds() > TIMEOUT) return;
    if(last_steering.nanoseconds()==0 || (now_-last_steering).seconds() > TIMEOUT) return;

    float vx = speed;  // Removed division by 36, assume m/s input
    float vth = convert_steering_angle_to_angular_velocity(vx, steering);
    odom.update_odometry(vx, vth, now_);

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
    odom_.twist.twist.linear.x = vx;
    odom_.twist.twist.angular.z = vth;

    odom_.pose.covariance[0]  = 0.05;
    odom_.pose.covariance[7]  = 0.05;
    odom_.pose.covariance[14] = 0.05;
    odom_.pose.covariance[21] = 0.01;
    odom_.pose.covariance[28] = 0.01;
    odom_.pose.covariance[35] = 0.05;

    odom_.twist.covariance[0]  = 0.02;
    odom_.twist.covariance[7]  = 0.02;
    odom_.twist.covariance[14] = 0.02;
    odom_.twist.covariance[21] = 0.01;
    odom_.twist.covariance[28] = 0.01;
    odom_.twist.covariance[35] = 0.02;

    odom_publisher->publish(odom_);

    // Broadcast TF
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = odom.stamp;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";
    t.transform.translation.x = odom.x;
    t.transform.translation.y = odom.y;
    t.transform.translation.z = 0.0;
    t.transform.rotation = odom_quat;
    tf_broadcaster_->sendTransform(t);
}
