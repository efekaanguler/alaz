#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /sllidar/install/setup.bash

# LIDAR
ros2 run sllidar_ros2 sllidar_node --ros-args \
  -p serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  -p serial_baudrate:=115200 \
  -p frame_id:=laser &

# CAMERA (şu an sende /dev/video1 görünüyor)
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video1 \
  -p pixel_format:=YUYV \
  -p image_size:="[1280,720]" \
  -p output_encoding:=yuv422_yuy2 &

wait
