# Alaz Vehicle Interface

ROS 2 Humble vehicle interface for the team's custom competition car. It connects Autoware control and status messages to the car CAN bus through `ros2_socketcan`.

The repository currently contains Autoware launch package version `0.49.0` and `ros2_socketcan` version `1.3.0`.

## Status

The software path is integrated, but the car is still under construction. The CAN IDs, byte layouts, steering scale, acceleration mappings, and vehicle dimensions are **provisional** until checked against the final ECUs and measured car.

Do not perform powered vehicle tests based only on the software tests in this repository.

## Production Data Path

```text
Autoware control commands
  -> my_vehicle_interface
  -> /to_can_bus (can_msgs/Frame)
  -> ros2_socketcan
  -> can0

can0
  -> ros2_socketcan
  -> /from_can_bus (can_msgs/Frame)
  -> my_vehicle_interface
  -> Autoware vehicle status
```

`global_bringup` starts this path through:

```text
my_vehicle_launch/vehicle.launch.xml
  -> autoware_global_parameter_loader
  -> my_vehicle_interface/vehicle_interface.launch.xml
     -> ros2_socketcan
     -> my_vehicle_interface_node
```

The legacy scalar `ros2_can_bridge` package is not part of production bringup.

## Safety Behavior

The interface starts in the safety-stop state. It releases that state only when:

1. Mission control publishes `false` on `/mission_control/emergency_stop`.
2. A control command has been received within `command_timeout_sec` (default `0.2` seconds).
3. Mission control has refreshed its emergency state within `emergency_state_timeout_sec` (default `1.5` seconds).

The safety command is transmitted continuously at 25 Hz:

| Actuator | Safety value |
|---|---:|
| Throttle | 0% |
| Brake | 100% |
| Gear | Neutral |
| Steering | Centered |

The safety stop is active when a monitored sensor/localization input fails, before mission control is ready, when mission control stops publishing, when no control command has arrived, or after a command timeout. Sensor emergencies DO NOT recover automatically; they require an explicit reset from the operator via `/mission_control/manual_resume` after all monitored inputs are healthy. Command timeouts recover automatically only after a new command arrives.

The current state is published on `/vehicle/status/safety_stop`. `true` means the safety actuator command is active. The Autoware control-mode report is `NOT_READY` while stopped and `AUTONOMOUS` after release.

See [SAFETY.md](SAFETY.md) for validation and operational constraints.

## Launch

Ensure the SocketCAN interface already exists and is UP. Interface provisioning is hardware-specific and intentionally not performed by ROS launch.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_vehicle_launch vehicle.launch.xml can_interface:=can0
```

To run the interface without starting a CAN transport:

```bash
ros2 launch my_vehicle_interface vehicle_interface.launch.xml launch_can_bridge:=false
```

## ROS Interfaces

### Subscriptions

| Topic | Type | Purpose |
|---|---|---|
| `/control/command/control_cmd` | `autoware_control_msgs/msg/Control` | Steering and longitudinal command |
| `/control/command/gear_cmd` | `autoware_vehicle_msgs/msg/GearCommand` | Requested gear |
| `/control/command/turn_indicators_cmd` | `autoware_vehicle_msgs/msg/TurnIndicatorsCommand` | Indicator request |
| `/control/command/hazard_lights_cmd` | `autoware_vehicle_msgs/msg/HazardLightsCommand` | Hazard request |
| `/mission_control/emergency_stop` | `std_msgs/msg/Bool` | Recoverable mission safety state |
| `/from_can_bus` | `can_msgs/msg/Frame` | ECU feedback from `ros2_socketcan` |

### Publications

| Topic | Type | Purpose |
|---|---|---|
| `/to_can_bus` | `can_msgs/msg/Frame` | Commands to `ros2_socketcan` |
| `/vehicle/status/velocity_status` | `autoware_vehicle_msgs/msg/VelocityReport` | Vehicle speed |
| `/vehicle/status/steering_status` | `autoware_vehicle_msgs/msg/SteeringReport` | Steering feedback |
| `/vehicle/status/gear_status` | `autoware_vehicle_msgs/msg/GearReport` | Gear feedback |
| `/vehicle/status/control_mode` | `autoware_vehicle_msgs/msg/ControlModeReport` | Readiness/autonomous state |
| `/vehicle/status/safety_stop` | `std_msgs/msg/Bool` | Interface safety-stop state |

## Provisional CAN Map

| Direction | CAN ID | Current interpretation |
|---|---:|---|
| Command | `0x220` | Steering, IEEE-754 float |
| Command | `0x330` | Throttle byte 0, gear byte 2 |
| Command | `0x110` | Brake percentage byte 0 |
| Feedback | `0x440` | Speed, big-endian `uint16`, hm/h |
| Feedback | `0x1E5` | Steering sensor, bytes 1-2 |
| Feedback | `0x720` | Steering ECU status |
| Feedback | `0x730` | Motor ECU status |

This table is an assumption for software development, not a verified specification for the final car.

## Build And Test

```bash
colcon build --symlink-install --packages-up-to my_vehicle_launch mission_control
colcon test --packages-select my_vehicle_interface mission_control
colcon test-result --verbose
```

The source-level protocol check can also run without pytest:

```bash
cd modules/vehicle/my_vehicle_interface
python3 test/test_wiki_verification.py
```

## Required Before Vehicle Motion

1. Replace the provisional CAN map with the signed-off ECU specification.
2. Measure and update `my_vehicle_description/config/vehicle_info.param.yaml`.
3. Measure steering endpoints and update `max_steering_angle_rad`.
4. Calibrate throttle and brake mappings at low power.
5. Verify full brake and neutral on startup, emergency, command loss, interface crash, and CAN transport loss.
6. Verify the physical E-stop independently of ROS, the computer, and the software CAN path.
7. Run CAN loopback, hardware-in-the-loop, and closed-course tests with wheels safely restrained first.
