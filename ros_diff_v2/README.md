# ROS_DIFF v2

v2 是当前可运行的 2WD ROS 小车版本，硬件目标为 Raspberry Pi 4B 2GB、Ubuntu 20.04 Server、ROS Noetic、STM32F103 和 YDLIDAR-X2。

当前版本：`2.1.0`

## What Is Included

- `base_controller`：连接 STM32 底盘，订阅 `/cmd_vel`，发布里程计、TF、IMU、电池和底盘状态。
- `bringup.launch`：启动 URDF、底盘控制器和可选 YDLIDAR-X2。
- `mapping_v2.launch`：基于 `/scan` 与里程计启动 gmapping。
- `navigation_v2.launch`：启动 map_server、AMCL 和 move_base。
- `state_estimation.launch`：可选 `robot_localization` EKF 配置。

## Directory Layout

| 路径 | 说明 |
| --- | --- |
| `catkin_ws/` | ROS Noetic 工作空间，主要包为 `myrobot`。 |
| `stm32f1/` | STM32F103 下位机工程，包含电机、编码器、IMU、电池检测和串口协议。 |
| `stm32f1/TOOLS/pid_tuner/` | 浏览器版 PID/底盘串口调试工具。 |
| `myrobot/TOOLS/urdf_editor/` | URDF 参数可视化编辑工具，便于后续移植新底盘。 |
| `docs/` | v2 部署、硬件、协议、建图导航和排错文档。 |

## ROS Topics

| Topic | 类型 | 说明 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | ROS 速度命令输入。 |
| `/car_state` | `myrobot/speed` | 底盘速度状态。 |
| `/wheel/odom` | `nav_msgs/Odometry` | 纯双轮编码器里程计，适合标定轮径和轮距。 |
| `/odom` | `nav_msgs/Odometry` | 默认发布给导航使用的里程计。 |
| `/imu/data` | `sensor_msgs/Imu` | STM32 上传的 MPU6050 yaw 数据。 |
| `/battery_state` | `sensor_msgs/BatteryState` | STM32 ADC 估算电池状态。 |
| `/scan` | `sensor_msgs/LaserScan` | YDLIDAR-X2 激光数据。 |

## Build

如果直接在仓库内构建：

```bash
cd ~/ROS_DIFF/ros_diff_v2/catkin_ws
catkin_make
source devel/setup.bash
```

如果你更习惯把包放进 `~/catkin_ws`：

```bash
mkdir -p ~/catkin_ws/src
cp -r ~/ROS_DIFF/ros_diff_v2/catkin_ws/src/myrobot ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

注意：YDLIDAR SDK 不是 ROS 包；还需要把 `ydlidar_ros_driver` 放进同一个 catkin 工作空间。具体见 [docs/raspberry_pi_setup.md](docs/raspberry_pi_setup.md)。

## Bringup

只测试 STM32 底盘：

```bash
roslaunch myrobot bringup.launch enable_lidar:=false use_imu_yaw:=false
```

底盘 + 雷达：

```bash
roslaunch myrobot bringup.launch enable_lidar:=true use_imu_yaw:=false
```

进入建图前可检查：

```bash
rostopic hz /car_state
rostopic echo -n 1 /car_state
rostopic hz /scan
rosnode info /base_controller
```

`/base_controller` 应启动并订阅 `/cmd_vel`。

## Mapping

树莓派上运行：

```bash
roslaunch myrobot mapping_v2.launch use_imu_yaw:=false use_ekf:=false rviz:=false
```

另开终端遥控：

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _speed:=0.08 _turn:=0.35 _repeat_rate:=10.0 _key_timeout:=0.5
```

保存地图：

```bash
mkdir -p ~/maps
rosrun map_server map_saver -f ~/maps/home
```

## Navigation

```bash
roslaunch myrobot navigation_v2.launch map_file:=/home/ubuntu/maps/home.yaml use_imu_yaw:=false use_ekf:=false rviz:=false
```

在远程 PC 的 RViz 里设置初始位姿，然后发布 `2D Nav Goal`。

## Important Notes

- v2 ROS 上位机和 STM32 固件必须一起更新，CRC 协议不兼容无 CRC 帧。
- 电机和编码器方向建议先用 STM32/PID 调试工具确认，再接入 ROS。
- 当前电机、TB6612 通道和编码器引脚分配见 [docs/hardware_v2.md](docs/hardware_v2.md)。
- Raspberry Pi 4B 2GB 不建议本机跑 RViz，推荐远程电脑显示。
