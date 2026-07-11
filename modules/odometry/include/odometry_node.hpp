#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


struct Odometry
{
  double x;
  double y;
  double th;
  rclcpp::Time stamp;

  Odometry(const rclcpp::Time& time)
  {
    x = 0.0;
    y = 0.0;
    th = 0.0;
    stamp = time;
  }

  void update_odometry(const double vx, const double vth, const rclcpp::Time& cur_time)
  {
    if (stamp.seconds() == 0 && stamp.nanoseconds() == 0)
    {
      stamp = cur_time;
    }
    double dt = (cur_time - stamp).seconds();
    double delta_x = (vx * cos(th)) * dt;
    double delta_y = (vx * sin(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;
    stamp = cur_time;
  }
};

class OdometryNode : public rclcpp::Node {

    public:
    OdometryNode();

    private:
    std::string SPEED_TOPIC = "/vehicle_speed";
    std::string STEERING_TOPIC = "/steering_angle";
    std::string VELOCITY_REPORT_TOPIC = "/vehicle/status/velocity_status";
    std::string STEERING_REPORT_TOPIC = "/vehicle/status/steering_status";
    std::string ODOM_TOPIC = "/odom";
    std::string THROTTLE_TOPIC = "/throttle";

    float TIMEOUT=1.0;

    float speed = 0.0;
    float steering = 0.0;
    float throttle=0.0;
    rclcpp::Time last_speed;
    rclcpp::Time last_steering;
    Odometry odom;

    void publish_odometry();
    void speed_callback(std_msgs::msg::Float32 msg);
    void steering_callback(std_msgs::msg::Float32 msg);
    void velocity_report_callback(autoware_vehicle_msgs::msg::VelocityReport msg);
    void steering_report_callback(autoware_vehicle_msgs::msg::SteeringReport msg);
    void throttle_callback(std_msgs::msg::Float32 msg);
    float convert_steering_angle_to_angular_velocity(float cur_vel_mps, float cur_angle_rad) {

      const float wheel_base = 1.05;
      return tan(cur_angle_rad) * cur_vel_mps / wheel_base;

    }

    float simulate_speed();

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_subscriber;
    rclcpp::Subscription<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr
      velocity_report_subscriber;
    rclcpp::Subscription<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr
      steering_report_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr throttle_subscriber;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr publish_timer;
    
};
