# Raspberry Pi Setup

目标平台：Raspberry Pi 4B 2GB，Ubuntu 20.04 Server，ROS Noetic。

## 1. Clone Repository

```bash
cd ~
git clone https://github.com/NishimiyaShoukou/ROS_DIFF.git
```

如果你不熟悉 git，可以把仓库理解成“带历史记录的项目文件夹”。以后更新代码时，在这个目录里执行：

```bash
cd ~/ROS_DIFF
git pull
```

## 2. ROS Dependencies

先确保系统已经安装 ROS Noetic。常用依赖：

```bash
sudo apt update
sudo apt install -y \
  python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential \
  ros-noetic-xacro ros-noetic-robot-state-publisher ros-noetic-tf \
  ros-noetic-gmapping ros-noetic-map-server ros-noetic-amcl ros-noetic-move-base \
  ros-noetic-robot-localization ros-noetic-teleop-twist-keyboard \
  ros-noetic-rosbridge-server
```

第一次使用 rosdep 时：

```bash
sudo rosdep init
rosdep update
```

如果提示已经初始化过，继续 `rosdep update` 即可。

## 3. YDLIDAR Driver

YDLIDAR 分两部分：

- `YDLidar-SDK`：底层 SDK，安装到系统。
- `ydlidar_ros_driver`：ROS 包，必须放到 catkin 工作空间的 `src/` 下。

安装 SDK：

```bash
cd ~
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK
mkdir -p build
cd build
cmake ..
make
sudo make install
```

安装 ROS driver：

```bash
cd ~/ROS_DIFF/ros_diff_v2/catkin_ws/src
git clone https://github.com/YDLIDAR/ydlidar_ros_driver.git
```

只安装 SDK 但没有 `ydlidar_ros_driver` 时，`roslaunch` 会报：

```text
package 'ydlidar_ros_driver' not found
```

## 4. Serial Device Names

项目默认使用：

- STM32：`/dev/chassis`
- YDLIDAR-X2：`/dev/ydlidar`

推荐用 udev 固定设备名。先插上设备查看信息：

```bash
udevadm info -a -n /dev/ttyUSB0
udevadm info -a -n /dev/ttyUSB1
```

再创建规则：

```bash
sudo nano /etc/udev/rules.d/99-ros-diff.rules
```

示例：

```text
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="chassis", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="ydlidar", MODE="0666"
```

实际 `idVendor`/`idProduct` 要以你的设备为准。如果两个串口芯片完全相同，需要用 USB 物理路径或设备序列号进一步区分。

重载规则：

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/chassis /dev/ydlidar
```

## 5. Build

```bash
cd ~/ROS_DIFF/ros_diff_v2/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro noetic -r -y --skip-keys="ydlidar_ros_driver rviz usb_cam"
catkin_make
source devel/setup.bash
```

建议把环境写进 `~/.bashrc`：

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/ROS_DIFF/ros_diff_v2/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

新开终端后检查：

```bash
rospack find myrobot
rospack find myrobot_web
```

两个命令都能输出包路径，说明环境生效。

## 6. First Bringup

先不启用雷达：

```bash
roslaunch myrobot bringup.launch enable_lidar:=false use_imu_yaw:=false
```

看到 `Connected to chassis on /dev/chassis` 后，另开终端检查：

```bash
rostopic hz /car_state
rostopic echo -n 1 /car_state
rosnode info /base_controller
```

再启用雷达：

```bash
roslaunch myrobot bringup.launch enable_lidar:=true use_imu_yaw:=false
rostopic hz /scan
```

Raspberry Pi 4B 2GB 不建议本机跑 RViz，推荐在电脑端远程查看。
