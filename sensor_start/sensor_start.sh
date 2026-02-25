#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /sllidar/install/setup.bash

echo "Sensör düğümleri başlatılıyor..."

# LIDAR (Sabit udev linki)
ros2 run sllidar_ros2 sllidar_node --ros-args \
  -p serial_port:=/dev/sllidar \
  -p serial_baudrate:=115200 \
  -p frame_id:=laser &

# CAMERA (Sabit udev linki)
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/logitech_cam \
  -p pixel_format:=YUYV \
  -p image_size:="[1280,720]" \
  -p output_encoding:=yuv422_yuy2 &

wait
