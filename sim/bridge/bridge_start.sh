ros2 launch carla_ros_bridge carla_ros_bridge.launch.py \
    host:=localhost \
    port:=2000 \
    timeout:=10 \
    synchronous_mode:=true \
    fixed_delta_seconds:=0.05 \
    ego_vehicle_role_name:='["ego_vehicle"]'