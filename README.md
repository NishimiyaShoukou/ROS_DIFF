# ROS_DIFF

ROS_DIFF 是一个两驱差速 ROS 小车项目。仓库同时保留早期可参考的 v1 和当前正在验证的 v2，目标是让别人 clone 下来后能看懂硬件、编译 ROS、烧录 STM32，并完成遥控、建图和导航调试。

当前主线版本：`v2.1.0`

已验证状态：2026-07-09 在实车上跑通 STM32 底盘串口通信、YDLIDAR-X2 `/scan`、gmapping 建图、AMCL + move_base 导航。导航已经能向目标点移动，不再像 v1 那样原地乱转；定位精度和地图质量还需要继续通过场地、轮距、初始位姿和 IMU 标定微调。

## Repository Layout

| 路径 | 说明 |
| --- | --- |
| `ros_diff_v1/` | 早期 ROS/STM32F4 版本，作为历史参考保留，默认不再改动。 |
| `ros_diff_v2/` | 当前主线版本，面向 Raspberry Pi 4B 2GB + Ubuntu 20.04 Server + ROS Noetic + STM32F1 + YDLIDAR-X2。 |
| `VERSION` | 当前公开版本号。 |
| `CHANGELOG.md` | 版本变更记录。 |
| `.gitignore` | 过滤 catkin、Keil、STM32 编译产物和本地调试副本。 |

## V2 Hardware Snapshot

| 项目 | 当前值 |
| --- | --- |
| 底盘 | 2WD 差速底盘，前万向轮 |
| 主控 | Raspberry Pi 4B 2GB，Ubuntu 20.04 Server，ROS Noetic |
| 下位机 | STM32F103，HAL/Keil 工程 |
| 雷达 | YDLIDAR-X2，串口 `/dev/ydlidar`，frame `laser` |
| 底盘串口 | `/dev/chassis`，115200 8N1 |
| 驱动轮 | 直径 48 mm，轮宽 19 mm |
| 轮距 | 129 mm，按 148 mm 外宽减一个轮宽估算 |
| 有效轮周长 | `0.144764589 m`，根据 1 m 地板直行测试校准 |
| 雷达位姿 | `x=+0.080 m, y=0, z=+0.130 m, yaw=pi`，雷达 0 度朝车尾 |
| IMU | MPU6050 元件面朝下安装，ROS 默认 `invert_imu_yaw: true` |

更完整的接线、坐标和标定说明见 [ros_diff_v2/docs/hardware_v2.md](ros_diff_v2/docs/hardware_v2.md)。

## Quick Start

树莓派端建议先按文档安装依赖和绑定串口：

```bash
cd ~
git clone https://github.com/NishimiyaShoukou/ROS_DIFF.git
cd ~/ROS_DIFF/ros_diff_v2/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro noetic -r -y --skip-keys="ydlidar_ros_driver rviz usb_cam"
catkin_make
source devel/setup.bash
roslaunch myrobot bringup.launch enable_lidar:=true use_imu_yaw:=false
```

建图：

```bash
roslaunch myrobot mapping_v2.launch use_imu_yaw:=false use_ekf:=false rviz:=false
```

遥控：

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _speed:=0.08 _turn:=0.35 _repeat_rate:=10.0 _key_timeout:=0.5
```

导航：

```bash
roslaunch myrobot navigation_v2.launch map_file:=/home/ubuntu/maps/home.yaml use_imu_yaw:=false use_ekf:=false rviz:=false
```

详细步骤见 [ros_diff_v2/docs/raspberry_pi_setup.md](ros_diff_v2/docs/raspberry_pi_setup.md) 和 [ros_diff_v2/docs/mapping_navigation.md](ros_diff_v2/docs/mapping_navigation.md)。

## Documentation

- [v2 README](ros_diff_v2/README.md)：v2 目录说明、常用命令和当前验证状态。
- [硬件与坐标](ros_diff_v2/docs/hardware_v2.md)：尺寸、引脚、雷达/IMU 位姿和电池检测。
- [树莓派部署](ros_diff_v2/docs/raspberry_pi_setup.md)：clone、依赖、YDLIDAR 驱动、udev 和构建。
- [建图与导航](ros_diff_v2/docs/mapping_navigation.md)：bringup、teleop、gmapping、map_saver、AMCL 和 RViz 配置。
- [串口协议](ros_diff_v2/docs/serial_protocol_v2.md)：v2.1 二进制帧、CRC-8/MAXIM、状态位和电池字段。
- [故障排查](ros_diff_v2/docs/troubleshooting.md)：包找不到、base_controller 未启动、键盘无响应、雷达 warning、RViz 崩溃等。
- [ROS v2 分析记录](ros_diff_v2/docs/ros_v2_analysis.md)：开发过程中的技术分析和标定思路。

## Version Management

这个仓库建议保持简单：

- `main`：面向游客和自己调试的稳定主线，确保 README 能对应仓库里的代码。
- `v2.x.y` Git tag：每次实车验证到一个可回退状态后打 tag，例如 `v2.1.0`。
- `VERSION`、`CHANGELOG.md`、ROS `package.xml`：发布时同步更新。
- 短期实验可以使用 `feature/...` 或 `fix/...` 分支，验证后合回 `main`；暂时不需要长期维护 `v1`/`v2` 两条分支，因为目录已经把版本隔离清楚了。

v1 只作为历史快照和参考资料；新功能、修复、文档都优先放在 v2。