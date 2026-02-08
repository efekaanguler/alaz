#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <string>


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
    std::string SPEED_TOPIC = "/speed";
    std::string STEERING_TOPIC = "/steering";
    std::string ODOM_TOPIC = "/odom";

    float TIMEOUT=1.0;

    float speed = 0.0;
    float steering = 0.0;
    rclcpp::Time last_speed;
    rclcpp::Time last_steering;
    Odometry odom;

    void publish_odometry();
    void speed_callback(std_msgs::msg::Float32 msg);
    void steering_callback(std_msgs::msg::Float32 msg);

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_subscriber;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;

    rclcpp::TimerBase::SharedPtr publish_timer;
    
};
