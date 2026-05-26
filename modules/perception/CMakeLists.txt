cmake_minimum_required(VERSION 3.8)
project(perception)

find_package(ament_cmake REQUIRED)

# Install scripts
install(PROGRAMS
  test_scripts/webcam_detect_standalone.py
  scripts/mjpeg_to_ros.py
  scripts/visualize_detections.py
  test_scripts/dummy_lidar_publisher.py
  test_scripts/dummy_camera_publisher.py
  scripts/camera_info_fallback_publisher.py
  scripts/scan_static_tf_fallback.py
  scripts/pointcloud_to_occupancy_grid.py
  scripts/detection2d_to_tier4_rois.py
  scripts/mac_docker_start.sh
  scripts/nuc_docker_perception.sh
  test_scripts/test_nuc_perception_runtime.sh
  test_scripts/test_detection_e2e.sh
  test_scripts/test_planner_integration.sh
  test_scripts/test_perception_pipeline.sh
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

# Install config files (NUC fallback calibration, etc.)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
