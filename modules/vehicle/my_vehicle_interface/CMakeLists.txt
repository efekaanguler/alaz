cmake_minimum_required(VERSION 3.14)
project(my_vehicle_interface)

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Create the library
ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  src/vehicle_interface_node.cpp
  src/can_utils.cpp
)

# Create executable
ament_auto_add_executable(${PROJECT_NAME}_node
  src/main.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

# Register as component
rclcpp_components_register_node(${PROJECT_NAME}_lib
  PLUGIN "my_vehicle_interface::VehicleInterfaceNode"
  EXECUTABLE vehicle_interface_node
)

# Install launch files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_safety_policy test/test_safety_policy.cpp)
  target_include_directories(test_safety_policy PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
