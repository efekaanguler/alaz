# rdw_vehicle_launch

This package provides the core vehicle launch files for the Alaz autonomous vehicle platform.
It integrates the vehicle interface and parameter loader for Autoware.

## Launch

This package is automatically launched by `global_bringup`.
It executes `my_vehicle_launch/vehicle.launch.xml` which sets up the vehicle interface and Autoware global parameter loaders.
