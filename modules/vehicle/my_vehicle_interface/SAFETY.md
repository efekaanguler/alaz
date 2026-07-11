# Vehicle Interface Safety Contract

## Scope

This document describes the software fail-safe behavior implemented by `my_vehicle_interface`. It does not replace a hardware E-stop, safety relay, brake interlock, independent watchdog, or competition safety assessment.

## Safety Inputs

The interface evaluates two independent conditions at 25 Hz:

- Mission state: `/mission_control/emergency_stop` must be `false`.
- Mission heartbeat: the emergency state must be refreshed within 1.5 seconds.
- Command freshness: `/control/command/control_cmd` must exist and be no older than `command_timeout_sec`.

The emergency topic uses reliable, transient-local QoS. The interface initializes the emergency state to active, so a late or absent mission controller cannot release the vehicle.

## Safety Output

If either condition is unsafe, every CAN cycle commands:

```text
steering = 0.0
throttle = 0%
gear = neutral
brake = 100%
```

Emergency has higher priority than all Autoware commands. A previously requested Drive or Reverse gear is not sent while safety stop is active.

## Recovery

- Sensor/localization emergency: mission control publishes `false` after all monitored inputs recover. No manual reset is required by the current project decision.
- Missing/stale command: the interface remains stopped until a fresh Autoware control command arrives.
- Both conditions must be healthy before actuation resumes.

Automatic recovery creates a known risk: intermittent sensors can allow the system to resume after recovery. This policy must be reviewed again when the final competition rules and vehicle safety architecture are available.

## Current Monitored Inputs

Mission control currently watches the configured lidar, camera, odometry, and localization topics. GNSS and IMU topic strings are empty in the current mission-control source, so the planned M8N GNSS is **not yet part of the emergency decision**.

## No-Hardware Validation With VCAN

Create a virtual CAN interface outside ROS:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

Build and launch:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to my_vehicle_launch mission_control
source install/setup.bash
ros2 launch my_vehicle_launch vehicle.launch.xml can_interface:=vcan0
```

Observe the bus and safety status:

```bash
candump vcan0
ros2 topic echo /vehicle/status/safety_stop
```

Before mission readiness and before fresh commands, frames must continuously encode zero throttle, neutral, centered steering, and full brake.

## Fault Tests Required On Final Hardware

1. Start ROS without mission control and verify full brake/neutral.
2. Start mission control with each required sensor missing, one at a time.
3. Publish a valid command, stop the publisher, and verify safe CAN frames within 200 ms.
4. Trigger an emergency while continuously publishing acceleration commands.
5. Recover sensors and verify motion resumes only after a fresh command.
6. Disconnect the CAN adapter and verify the ECU-side independent watchdog state.
7. Kill `my_vehicle_interface` and verify the ECU-side independent watchdog state.
8. Press the physical E-stop and verify it works without ROS or CAN software.

## Unverified Assumptions

- CAN IDs and byte layouts match the final car.
- `100` means maximum brake command.
- Gear value `0` means neutral.
- Steering value `0.0` is the safest centered command.
- ECUs accept simultaneous neutral and full-brake commands.
- Brake actuation remains available during motor/CAN faults.

These assumptions must be replaced by measured, signed-off requirements before powered testing.
