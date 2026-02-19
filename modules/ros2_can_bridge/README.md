# Example launch and usage instructions for ros2_can_bridge

## Build the package

Assuming you are in the root of your ROS2 workspace (e.g., ~/ros2_ws):

```
colcon build --packages-select ros2_can_bridge
source install/setup.bash
```

## Run the node

Make sure can0 is up and running:

```
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

Then run the node:

```
ros2 run ros2_can_bridge ros2_can_bridge_node
```

## Topics

- Subscribe to control:
  - `throttle_cmd` (`std_msgs/msg/Int32`): 0-100
  - `brake_cmd` (`std_msgs/msg/Int32`): 0-100
  - `steering_cmd` (`std_msgs/msg/Float32`): -1.0 to 1.0 (left/right)

- Feedback published:
  - `steering_angle` (`std_msgs/msg/Float32`): Current steering angle (float)
  - `vehicle_speed` (`std_msgs/msg/Float32`): Current speed (float)

## Example publish commands

```
ros2 topic pub /throttle_cmd std_msgs/msg/Int32 "data: 50"
ros2 topic pub /brake_cmd std_msgs/msg/Int32 "data: 0"
ros2 topic pub /steering_cmd std_msgs/msg/Float32 "data: 0.2"
```

## Example feedback subscription

```
ros2 topic echo /steering_angle
ros2 topic echo /vehicle_speed
```
