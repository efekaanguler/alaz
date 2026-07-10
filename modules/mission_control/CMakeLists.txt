cmake_minimum_required(VERSION 3.8)
project(mission_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
# ADD THIS LINE:
find_package(autoware_control_msgs REQUIRED) 
find_package(autoware_vehicle_msgs REQUIRED)

# Executable declaration
add_executable(mission_control_launch
  src/mission_control_launch.cpp
  src/mode_start.cpp
  src/mode_run.cpp
  src/mode_pause.cpp
  src/mode_park.cpp
  src/mode_emergency.cpp
)

# Include directories and target dependencies
target_include_directories(mission_control_launch PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mission_control_launch
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  autoware_planning_msgs
  autoware_adapi_v1_msgs
  autoware_control_msgs # ADD THIS LINE HERE AS WELL
  autoware_vehicle_msgs
)

# Install rules
install(TARGETS mission_control_launch
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()