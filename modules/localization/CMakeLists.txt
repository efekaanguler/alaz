cmake_minimum_required(VERSION 3.8)
project(localization)

find_package(ament_cmake REQUIRED)

# Install launch and config files so ros2 launch can find them.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)


# Python node'larını install et
install(PROGRAMS
  initial_pose_pub.py
  odom_to_twist_cov.py
  map_projector_info_pub.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
