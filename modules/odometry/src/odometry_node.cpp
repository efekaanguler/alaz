#include <odometry_node.hpp>

#include <stdexcept>

using namespace std::chrono_literals;

OdometryNode::OdometryNode()
: Node("odometry_node"), odom(now())
{
  wheel_base = declare_parameter<float>("wheel_base", wheel_base);
  TIMEOUT = declare_parameter<float>("timeout_sec", TIMEOUT);
  publish_rate_hz = declare_parameter<double>("publish_rate_hz", publish_rate_hz);
  publish_tf = declare_parameter<bool>("publish_tf", publish_tf);

  if (wheel_base <= 0.0F) {
    throw std::invalid_argument("wheel_base must be positive");
  }
  if (publish_rate_hz <= 0.0) {
    throw std::invalid_argument("publish_rate_hz must be positive");
  }

  const auto status_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  velocity_report_subscriber = create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(
    VEHICLE_VELOCITY_TOPIC, status_qos,
    std::bind(&OdometryNode::velocity_report_callback, this, std::placeholders::_1));
  steering_report_subscriber = create_subscription<autoware_vehicle_msgs::msg::SteeringReport>(
    VEHICLE_STEERING_TOPIC, status_qos,
    std::bind(&OdometryNode::steering_report_callback, this, std::placeholders::_1));

  odom_publisher = create_publisher<nav_msgs::msg::Odometry>(ODOM_TOPIC, 10);
  tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  const auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_hz);
  publish_timer = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
    std::bind(&OdometryNode::publish_odometry, this));
}

void OdometryNode::velocity_report_callback(autoware_vehicle_msgs::msg::VelocityReport msg)
{
  speed = msg.longitudinal_velocity;
  last_speed = now();
}

void OdometryNode::steering_report_callback(autoware_vehicle_msgs::msg::SteeringReport msg)
{
  steering = msg.steering_tire_angle;
  last_steering = now();
}

void OdometryNode::publish_odometry()
{
  auto now_ = now();
  if (last_speed.nanoseconds() == 0 || (now_ - last_speed).seconds() > TIMEOUT) {return;}
  if (last_steering.nanoseconds() == 0 || (now_ - last_steering).seconds() > TIMEOUT) {return;}

  float vx = speed;
  float vth = convert_steering_angle_to_angular_velocity(vx, steering);
  odom.update_odometry(vx, vth, now_);

  tf2::Quaternion q;
  q.setRPY(0, 0, odom.th);   // Create quaternion from yaw
  geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);   // Convert to msg

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
  odom_.pose.covariance[0] = 0.25;
  odom_.pose.covariance[7] = 0.25;
  odom_.pose.covariance[35] = 0.1;
  odom_.twist.covariance[0] = 0.04;
  odom_.twist.covariance[35] = 0.1;

  odom_publisher->publish(odom_);
  if (publish_tf) {
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = odom_.header.stamp;
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";
    odom_tf.transform.translation.x = odom.x;
    odom_tf.transform.translation.y = odom.y;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation = odom_quat;
    tf_broadcaster->sendTransform(odom_tf);
  }
  RCLCPP_DEBUG(this->get_logger(), "Odometry published");
}
