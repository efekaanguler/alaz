# Planning Module

Planning module for Autoware autonomous driving system.

## Overview

This module contains the planning stack for path planning and motion planning components.

## Components

- **Behavior Path Planner**: High-level path planning with behavior selection
- **Motion Velocity Smoother**: Velocity profile generation and smoothing
- **Trajectory Follower**: Low-level trajectory tracking

## Configuration

See `config/` directory for parameter files.

## Launch

To launch the planning module:

```bash
ros2 launch planning planning.launch.py
```

## References

- [Autoware Documentation](https://autowarefoundation.org)
