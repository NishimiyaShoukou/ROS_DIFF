# 基于树莓派 4B 和 STM32 的 ROS 差速机器人

Raspberry Pi 4B + STM32 differential-drive ROS robot

<p align="center">
  <img src="ros_diff_v1/photo/myrobot.png" alt="ROS differential-drive robot prototype" width="720">
</p>

<p align="center">
  <sub>v1 实物原型图。v2 延续 2WD 差速底盘思路，并更新为 ROS Noetic、STM32F1 和 YDLIDAR-X2。</sub>
</p>

`ROS_DIFF` 是本仓库名称，其中 `DIFF` 表示 differential drive，即双轮差速驱动。项目记录了一台基于 Raspberry Pi 4B 和 STM32 的 ROS 小车从早期版本到 v2 的重构过程，包含 ROS 上位机、STM32 下位机、URDF、建图导航配置和实车调试文档。

当前主线版本：`v2.1.0`

## Project Status

| 模块 | 状态 |
| --- | --- |
| 底盘通信 | STM32 与树莓派通过自定义二进制串口协议通信，v2.1 使用 CRC-8/MAXIM。 |
| 里程计 | ROS 侧基于双轮累计编码器计数计算 `/wheel/odom` 和 `/odom`。 |
| 雷达 | YDLIDAR-X2 已发布 `/scan`，实测可用于 gmapping。 |
| 建图 | `mapping_v2.launch` 已在室内环境跑通。 |
| 导航 | `navigation_v2.launch` 已跑通 AMCL + move_base，可向目标点移动。 |
| 待标定 | 有效轮距、IMU yaw、导航速度和局部代价地图仍需按场地继续微调。 |

## Hardware Overview

| 项目 | 当前配置 |
| --- | --- |
| 底盘 | 2WD 差速底盘，前万向轮 |
| 上位机 | Raspberry Pi 4B 2GB，Ubuntu 20.04 Server，ROS Noetic |
| 下位机 | STM32F103，HAL/Keil 工程 |
| 雷达 | YDLIDAR-X2，默认设备 `/dev/ydlidar` |
| 底盘串口 | 默认设备 `/dev/chassis`，115200 8N1 |
| 驱动轮 | 直径 48 mm，轮宽 19 mm |
| 有效轮距 | 初值 129 mm |
| 有效轮周长 | `0.144764589 m`，由 1 m 直行测试校准 |
| 雷达位姿 | `x=+0.080 m, y=0, z=+0.130 m, yaw=pi` |
| IMU | MPU6050，当前建议建图导航先使用 `use_imu_yaw:=false` |

完整尺寸、引脚和坐标说明见 [ros_diff_v2/docs/hardware_v2.md](ros_diff_v2/docs/hardware_v2.md)。

## Repository Layout

| 路径 | 说明 |
| --- | --- |
| `ros_diff_v1/` | 早期 ROS/STM32F4 版本，作为历史参考保留。 |
| `ros_diff_v2/` | 当前主线版本，面向 Raspberry Pi 4B、ROS Noetic、STM32F1 和 YDLIDAR-X2。 |
| `ros_diff_v2/catkin_ws/` | ROS Noetic 工作空间。 |
| `ros_diff_v2/stm32f1/` | STM32F103 下位机工程。 |
| `ros_diff_v2/docs/` | v2 部署、协议、硬件、建图导航和排错文档。 |
| `VERSION` | 当前公开版本号。 |
| `CHANGELOG.md` | 版本变更记录。 |

## Quick Start

```bash
cd ~
git clone https://github.com/NishimiyaShoukou/ROS_DIFF.git
cd ~/ROS_DIFF/ros_diff_v2/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro noetic -r -y --skip-keys="ydlidar_ros_driver rviz usb_cam"
catkin_make
source devel/setup.bash
```

启动底盘和雷达：

```bash
roslaunch myrobot bringup.launch enable_lidar:=true use_imu_yaw:=false
```

建图：

```bash
roslaunch myrobot mapping_v2.launch use_imu_yaw:=false use_ekf:=false rviz:=false
```

键盘遥控：

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _speed:=0.08 _turn:=0.35 _repeat_rate:=10.0 _key_timeout:=0.5
```

导航：

```bash
roslaunch myrobot navigation_v2.launch map_file:=/home/ubuntu/maps/home.yaml use_imu_yaw:=false use_ekf:=false rviz:=false
```

树莓派 4B 2GB 不建议本机运行 RViz。推荐树莓派只运行 ROS 节点，在 PC 或虚拟机中远程查看 RViz。

## Documentation

- [v2 README](ros_diff_v2/README.md)：v2 目录、常用命令和当前验证状态。
- [硬件与坐标](ros_diff_v2/docs/hardware_v2.md)：尺寸、引脚、雷达/IMU 位姿和电池检测。
- [树莓派部署](ros_diff_v2/docs/raspberry_pi_setup.md)：clone、依赖、YDLIDAR 驱动、udev 和构建。
- [建图与导航](ros_diff_v2/docs/mapping_navigation.md)：bringup、teleop、gmapping、map_saver、AMCL 和 RViz 配置。
- [串口协议](ros_diff_v2/docs/serial_protocol_v2.md)：v2.1 二进制帧、CRC-8/MAXIM、状态位和电池字段。
- [故障排查](ros_diff_v2/docs/troubleshooting.md)：常见 ROS、串口、雷达和 RViz 问题。
- [ROS v2 分析记录](ros_diff_v2/docs/ros_v2_analysis.md)：开发过程中的技术分析和标定思路。

## Versioning

仓库采用简单的版本管理方式：

- `main` 保持当前可运行主线。
- `ros_diff_v1/` 作为历史版本参考。
- `ros_diff_v2/` 作为当前开发和调试主线。
- `v2.x.y` Git tag 标记已经实车验证过的版本。
- 发布时同步更新 `VERSION`、`CHANGELOG.md` 和 ROS `package.xml`。

当前建议优先在 v2 上继续补充实车测试、地图、标定参数和导航调参记录。
