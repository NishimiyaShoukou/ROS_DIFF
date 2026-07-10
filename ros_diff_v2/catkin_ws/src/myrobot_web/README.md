# myrobot_web

`myrobot_web` provides a local, tablet-friendly control page for ROS_DIFF v2. The Raspberry Pi serves the page and connects it to ROS through rosbridge; Android, iPadOS, and desktop devices only need a modern browser on the same LAN.

## Features

- Touch joystick with dead-man stop behavior.
- Mutually exclusive chassis, mapping, and navigation modes.
- Live occupancy grid, robot pose, and global path display.
- Map pan, wheel/pinch zoom, and fit-to-view.
- Drag-to-set AMCL initial pose and navigation goal.
- One-tap map saving, navigation cancel, and emergency stop.
- Battery and ROS node status display.
- Manual velocity priority over navigation velocity through a small command multiplexer.

## Install

Install rosbridge on the Raspberry Pi, then build the workspace:

```bash
sudo apt update
sudo apt install ros-noetic-rosbridge-server

cd ~/ROS_DIFF/ros_diff_v2/catkin_ws
catkin_make
source devel/setup.bash
```

When copying packages into another workspace, copy both `myrobot` and `myrobot_web`.

## Run

Stop any existing `bringup`, mapping, or navigation roslaunch process first, then run:

```bash
source ~/ROS_DIFF/ros_diff_v2/catkin_ws/devel/setup.bash
roslaunch myrobot_web mobile_control.launch
```

Open the following address on a tablet connected to the same Wi-Fi:

```text
http://<raspberry-pi-ip>:8080
```

The default launch starts the chassis and lidar in `idle` mode. The page can then switch to mapping or navigation. Saving a map updates `myrobot/map/map.yaml` and `myrobot/map/map.pgm`, which are also the default navigation map.

## Launch Parameters

| Parameter | Default | Purpose |
| --- | --- | --- |
| `http_port` | `8080` | Tablet web page port. |
| `websocket_port` | `9090` | rosbridge WebSocket port. |
| `auto_start_mode` | `idle` | Initial mode: `idle`, `mapping`, `navigation`, or `none`. |
| `serial_port` | `/dev/chassis` | STM32 serial device. |
| `lidar_port` | `/dev/ydlidar` | YDLIDAR serial device. |
| `map_file` | `myrobot/map/map.yaml` | Map loaded by navigation. |
| `map_prefix` | `myrobot/map/map` | Files written by map saver. |
| `use_imu_yaw` | `false` | Enable base-controller IMU yaw fusion. |
| `use_ekf` | `false` | Enable robot_localization EKF. |

If a non-default WebSocket port is used, append it to the page URL, for example `http://<ip>:8080/?ws=9091`.

## Safety

Keep the web interface on a trusted local network. Do not expose ports `8080`, `9090`, or the ROS master directly to the public internet. The page sends zero velocity on pointer release, loss of focus, and connection loss; the STM32 bridge command timeout remains the final stop safeguard.

The manager refuses to start a second robot mode when it detects externally launched chassis, mapping, or navigation nodes. Stop the external roslaunch terminal before switching modes from the page.
