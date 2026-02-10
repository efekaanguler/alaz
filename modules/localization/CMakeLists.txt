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

ament_package()
