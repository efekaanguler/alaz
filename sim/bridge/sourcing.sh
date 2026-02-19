#!/bin/bash

# Find and source ROS Bridge setup.bash

for home_dir in /home/* /root; do
    setup_path="$home_dir/Workspace/ros-bridge/install/setup.bash"
    if [ -f "$setup_path" ]; then
        source "$setup_path"
        echo "✓ Sourced: $setup_path"
        return 0 2>/dev/null || exit 0
    fi
done

echo "✗ Could not find ros-bridge setup.bash"
return 1 2>/dev/null || exit 1