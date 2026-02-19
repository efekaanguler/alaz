cmake_minimum_required(VERSION 3.8)
project(can_bridge)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Python modüllerini kur
install(
  DIRECTORY ros2_can_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# Python node'u çalıştırılabilir olarak kur
install(
  PROGRAMS
    ros2_can_bridge/bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# launch dosyalarını kur
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
