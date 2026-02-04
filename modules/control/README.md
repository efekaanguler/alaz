# Control Module

Control module for Autoware autonomous driving system.

## Overview

This module contains the control stack for vehicle control and trajectory tracking.

## Components

- **Trajectory Follower Node**: Main trajectory tracking controller
- **MPC Lateral Controller**: Model Predictive Control for lateral vehicle dynamics
- **Adaptive Cruise Control**: Longitudinal velocity control

## Configuration

See `config/` directory for parameter files.

## Launch

To launch the control module:

```bash
ros2 launch control control.launch.py
```

## References

- [Autoware Documentation](https://autowarefoundation.org)
