# Troubleshooting

## `bringup.launch` 找不到

现象：

```text
RLException: [bringup.launch] is neither a launch file in package [myrobot]
```

处理：

```bash
cd ~/ROS_DIFF/ros_diff_v2/catkin_ws
catkin_make
source devel/setup.bash
rospack find myrobot
```

如果你把 `src` 复制到了 `~/catkin_ws`，就 source 对应工作空间的 `devel/setup.bash`。

## `Resource not found: xacro`

安装缺失依赖：

```bash
sudo apt update
sudo apt install ros-noetic-xacro
```

然后重新 source 工作空间。

## `ydlidar_ros_driver` Not Found

只安装 `YDLidar-SDK` 不够，SDK 不是 ROS 包。还需要：

```bash
cd ~/ROS_DIFF/ros_diff_v2/catkin_ws/src
git clone https://github.com/YDLIDAR/ydlidar_ros_driver.git
cd ..
catkin_make
source devel/setup.bash
rospack find ydlidar_ros_driver
```

## YDLIDAR Checksum Warning

偶发：

```text
Checksum error ...
Real points 497 > fixed points 490
```

如果 `/scan` 频率稳定、RViz 里雷达方向正确、建图可用，少量 warning 通常不影响调试。若 warning 密集且 `/scan` 中断，检查供电、USB 线、雷达串口名和 YDLIDAR 参数。

## Keyboard Teleop Publishes But Robot Does Not Move

先确认底盘节点真的在运行：

```bash
rosnode info /base_controller
```

正常应看到 `/base_controller` 订阅 `/cmd_vel`。如果显示 `unknown node` 或没有订阅，说明只启动了 `roscore`，没有启动底盘节点。运行：

```bash
roslaunch myrobot bringup.launch enable_lidar:=false use_imu_yaw:=false
```

再测试：

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## `No fresh chassis feedback`

含义：ROS 已启动，但没有收到新鲜 STM32 反馈，因此抑制运动命令。

检查：

```bash
ls -l /dev/chassis
rostopic hz /car_state
```

常见原因：

- 串口名绑定错。
- 没有权限访问串口。
- STM32 没烧 v2.1 固件，协议 CRC 或帧长不匹配。
- 串口被 PID 调试网页或其他串口助手占用。

## Wheel Direction Is Strange

如果 PID 调试网页里给左右轮独立转速都异常，不要先改 ROS。先检查 STM32 固件中的电机映射：

- 左轮 MOTOR1：`PA8 + PB14/PB15`。
- 右轮 MOTOR2：`PA9 + PB12/PB13`。
- 左编码器：`PB6/PB7`。
- 右编码器：`PA0/PA1`。

底层测试正常后，再用 ROS YAML 的 `swap_left_right`、`invert_left_command`、`invert_right_command`、`invert_left_feedback`、`invert_right_feedback` 做小范围补偿。

## RViz Crashes

虚拟机或低性能电脑上 RViz 可能因为 OpenGL/OGRE 崩溃。建议：

- 远程电脑运行 RViz，树莓派只跑 ROS 节点。
- 用 `LaserScan` 显示 `/scan`，不要误用点云显示。
- 减少显示项，关闭不必要的 RobotModel/TF trail。
- Fixed Frame 建图用 `map`，只看雷达时也可临时用 `laser` 或 `base_footprint` 排查。

## `teleop_twist_keyboard` Not Found

安装：

```bash
sudo apt update
sudo apt install ros-noetic-teleop-twist-keyboard
```

## `serial_port::available` Compile Error

Ubuntu 20.04 的 Boost 1.71 上 `boost::asio::serial_port` 没有 `available()` 成员。v2 当前代码已经改成平台相关的可读字节查询函数；如果仍然报这个错，说明树莓派上的 `myrobot` 代码不是最新版本，需要重新同步仓库或复制最新 `src/myrobot`。