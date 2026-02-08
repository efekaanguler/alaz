#include <odometry_node.hpp>

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdometryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
