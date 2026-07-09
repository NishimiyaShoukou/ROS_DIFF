# Mapping And Navigation

本流程按“底盘通信正常 -> 雷达正常 -> 键盘遥控正常 -> 建图 -> 保存地图 -> 导航”的顺序走。

## 1. Bringup

树莓派终端 1：

```bash
cd ~/ROS_DIFF/ros_diff_v2/catkin_ws
source devel/setup.bash
roslaunch myrobot bringup.launch enable_lidar:=true use_imu_yaw:=false
```

检查：

```bash
rostopic hz /car_state
rostopic hz /scan
rosnode info /base_controller
```

`/base_controller` 应该订阅 `/cmd_vel`，否则遥控命令不会进入底盘节点。

## 2. Teleop

树莓派终端 2：

```bash
source ~/ROS_DIFF/ros_diff_v2/catkin_ws/devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _speed:=0.08 _turn:=0.35 _repeat_rate:=10.0 _key_timeout:=0.5
```

键盘方向约定：

- `i`：前进。
- `,`：后退。
- `j`：左转，对应正 `angular.z`。
- `l`：右转。
- `u/o/m/.`：弧线运动。

如果 `j` 的 `/cmd_vel.angular.z` 为正，但小车实际右转，先检查低层左右轮方向和编码器反馈，再决定是否修改 ROS 配置。

## 3. Remote RViz

远程电脑需要能访问树莓派 ROS master。电脑端：

```bash
export ROS_MASTER_URI=http://<raspberry_pi_ip>:11311
export ROS_IP=<pc_ip>
source /opt/ros/noetic/setup.bash
rviz
```

RViz 推荐配置：

- Fixed Frame：建图时用 `map`。
- Add `Map`，Topic `/map`。
- Add `LaserScan`，Topic `/scan`，Size 适当调小。
- Add `TF`。
- Add `RobotModel`。
- Add `Odometry`，Topic `/odom` 或 `/wheel/odom`。

虚拟机里 RViz 如果容易崩溃，优先使用 `LaserScan`，不要把 `/scan` 当点云显示；减少显示项和队列长度。

## 4. Mapping

树莓派终端 1：

```bash
roslaunch myrobot mapping_v2.launch use_imu_yaw:=false use_ekf:=false rviz:=false
```

终端 2 用键盘慢速绕房间移动。建议：

- 速度先低一点，例如 `_speed:=0.08`。
- 转向不要原地狂转，尽量有平移。
- 房间太小、墙面太近、特征太少时，gmapping 容易畸变。
- 建图期间保持雷达高度和线束不要晃。

保存地图：

```bash
mkdir -p ~/maps
rosrun map_server map_saver -f ~/maps/home
```

会生成：

```text
~/maps/home.yaml
~/maps/home.pgm
```

## 5. Navigation

树莓派：

```bash
roslaunch myrobot navigation_v2.launch map_file:=/home/ubuntu/maps/home.yaml use_imu_yaw:=false use_ekf:=false rviz:=false
```

`map_file` 参数应指向实际保存的地图 yaml 文件，例如 `/home/<user>/maps/home.yaml`。

RViz：

1. Fixed Frame 设为 `map`。
2. Add `Map`、`LaserScan`、`TF`、`RobotModel`、`Odometry`。
3. 用 `2D Pose Estimate` 给 AMCL 初始位姿。
4. 确认激光点云和地图墙体大致重合。
5. 用 `2D Nav Goal` 给目标点。

初始位姿不准时，小车会“看起来会走，但走不准”。先把初始位姿对准，再调导航参数。

## 6. Suggested Tuning Order

1. 纯底盘直行 1 m，校准 `wheel_circumference`。
2. 原地转 360 度，校准有效 `wheel_separation`。
3. 确认雷达方向，障碍物在车前时 RViz 也显示在车前。
4. 先不用 IMU 完成一张稳定地图。
5. 再开启 IMU yaw 或 `robot_localization`，观察 `/odom` 是否更稳。
6. 最后调 `DWAPlannerROS` 最大速度、角速度、膨胀层和 footprint。
