# TARS

Mudflat rescue robot software. ROS 2 Humble, 4-wheel skid-steer, GPS/IMU EKF
localization, Nav2, and a height-adjustable axle.

## Platform

- Wheels: 250 mm diameter, overdrive 2.5
- Track width: 0.4788 m
- Drive: Dynamixel velocity mode, left IDs 2/3, right IDs 12/13
- Height: front IDs 1/4, rear IDs 11/14
- Sensors: u-blox GPS (NTRIP RTK), iahrs IMU, RealSense D435, ground probe

NTRIP credentials are not stored here. Pass them at launch.

## Layout

- `src/eclipse_pkg` — nodes, launch, Nav2/EKF config, waterline keepout
- `src/tars_recovery_behaviors` — recovery behavior-tree plugins
- `src/tars_tide_layer` — tide keepout costmap layer
- `src/eclipse_pkg_msgs` — custom messages
- `arduino/probe_sensor` — probe firmware
- `datasets/tidflt` — mudflat polygons for tide keepout
- `scripts/` — Jetson bringup and waterline bake tools

## Build

```bash
source /opt/ros/humble/setup.bash
cd /path/to/eclipse-summit
colcon build --symlink-install
source install/setup.bash
```

```bash
ros2 launch eclipse_pkg tars_autonomy.launch.py
```
