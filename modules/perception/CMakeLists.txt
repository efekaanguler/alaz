cmake_minimum_required(VERSION 3.8)
project(perception)

find_package(ament_cmake REQUIRED)

# Install scripts
install(PROGRAMS
  scripts/webcam_detect_standalone.py
  scripts/mjpeg_to_ros.py
  scripts/visualize_detections.py
  scripts/dummy_lidar_publisher.py
  scripts/dummy_camera_publisher.py
  scripts/docker_pipeline_start.sh
  scripts/test_detection_e2e.sh
  scripts/test_planner_integration.sh
  scripts/test_perception_pipeline.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Install models
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
