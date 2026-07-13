# xbox_controller

ROS 2 Xbox controller teleoperation package for the Alaz custom vehicle stack.

This package does not send CAN frames and does not contain RDW/SDC kart CAN IDs. It publishes Autoware-compatible command topics consumed by `my_vehicle_interface`, keeping CAN encoding, watchdogs, emergency behavior, and future custom-car adaptation in the vehicle interface layer.

## Published Topics

- `/control/command/control_cmd` (`autoware_control_msgs/msg/Control`)
- `/control/command/gear_cmd` (`autoware_vehicle_msgs/msg/GearCommand`)

## Default Controls

- Hold `A` (`BTN_SOUTH`) for 1.5 seconds: arm controller
- `Back` (`BTN_SELECT`): disarm controller
- Hold `RB` (`BTN_TR`): deadman switch required for motion commands
- Right trigger (`ABS_RZ`): positive acceleration
- Left trigger (`ABS_Z`): braking / negative acceleration
- Left stick X (`ABS_X`): steering
- `Y` (`BTN_WEST`): neutral
- `B` (`BTN_EAST`): drive
- `X` (`BTN_NORTH`): reverse, ignored unless `allow_reverse=true`

When disarmed, disconnected, or deadman is released, the node publishes zero steering, negative acceleration, and neutral gear by default.

## Runtime Dependency

The node lazily imports the Python `inputs` package at runtime:

```bash
python3 -m pip install inputs
```

## Run

```bash
ros2 launch xbox_controller xbox_controller.launch.py
```

Or through global bringup manual mode:

```bash
ros2 launch global_bringup global.launch.py manual:=true
```

Manual mode disables autonomous `planning` and `control` launch entries while enabling `xbox_controller`, avoiding command-topic conflicts.

For custom-car calibration, edit `config/xbox_controller.param.yaml` and tune steering, acceleration, deceleration, velocity, and deadzone parameters before closed-course motion tests.
