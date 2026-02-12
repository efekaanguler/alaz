cmake_minimum_required(VERSION 3.14)
project(perception_module)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install README
install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
