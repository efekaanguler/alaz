# my_vehicle_interface

Autoware vehicle interface for the Self Driving Challenge 2026 kart with CAN support using ros2_socketcan.

## Overview

This package bridges Autoware's control commands with the SDC 2026 kart's CAN-controlled actuators:
- **Steering**: Type A control (target angle → servo)
- **Throttle**: Type B control (direct % → electric motor)
- **Brake**: Type B control (direct % → linear actuator)

## Prerequisites

- ROS 2 Humble
- Autoware
- [ros2_socketcan](https://github.com/autowarefoundation/ros2_socketcan)

## Installation

```bash
# Clone to your autoware workspace
cd <your-autoware-dir>/src/vehicle/external
cp -r my_vehicle_interface .

# Build
cd <your-autoware-dir>
colcon build --packages-select my_vehicle_interface
source install/setup.bash
```

## Configuration

**Before using on the real kart**, update the CAN IDs in `config/vehicle_interface.param.yaml` from the competition wiki (A 3.3).

## Usage

### With Real Hardware

```bash
# Terminal 1: Start CAN bridge
ros2 launch ros2_socketcan socket_can_bridge.launch.xml interface:=can0

# Terminal 2: Start vehicle interface
ros2 launch my_vehicle_interface vehicle_interface.launch.xml
```

### Testing with Virtual CAN (Linux)

```bash
# Set up virtual CAN
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# Terminal 1: Start CAN bridge with vcan
ros2 launch ros2_socketcan socket_can_bridge.launch.xml interface:=vcan0

# Terminal 2: Start kart simulator
python3 test/test_can_simulation.py -i vcan0

# Terminal 3: Start vehicle interface
ros2 launch my_vehicle_interface vehicle_interface.launch.xml
```

## Topics

### Subscribed (from Autoware)
| Topic | Message Type |
|-------|--------------|
| `/control/command/control_cmd` | `AckermannControlCommand` |
| `/control/command/gear_cmd` | `GearCommand` |
| `/from_can_bus` | `can_msgs/Frame` |

### Published (to Autoware)
| Topic | Message Type |
|-------|--------------|
| `/vehicle/status/velocity_status` | `VelocityReport` |
| `/vehicle/status/steering_status` | `SteeringReport` |
| `/vehicle/status/gear_status` | `GearReport` |
| `/to_can_bus` | `can_msgs/Frame` |

## Parameters

See `config/vehicle_interface.param.yaml` for all configurable parameters.

## File Structure

```
my_vehicle_interface/
├── include/my_vehicle_interface/
│   ├── vehicle_interface_node.hpp
│   └── can_utils.hpp
├── src/
│   ├── vehicle_interface_node.cpp
│   ├── can_utils.cpp
│   └── main.cpp
├── launch/
│   └── vehicle_interface.launch.xml
├── config/
│   └── vehicle_interface.param.yaml
├── test/
│   └── test_can_simulation.py
├── CMakeLists.txt
├── package.xml
└── README.md
```

## License

Apache-2.0
