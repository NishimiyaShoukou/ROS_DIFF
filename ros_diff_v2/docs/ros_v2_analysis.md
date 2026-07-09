# ROS_DIFF v2 ROS 上位机分析与适配记录

> 2026-07-09：串口协议已升级为 v2.1，双向帧均使用
> CRC-8/MAXIM 校验。最新帧格式以
> [serial_protocol_v2.md](serial_protocol_v2.md) 为准，本文后续出现的旧帧长度仅保留为历史分析。

> 当前 ROS 节点采用 Boost.Asio 承载自定义二进制协议，不使用 rosserial。
> STM32 负责 100 Hz 电机闭环和 300 ms 安全超时，树莓派负责 50 Hz
> 速度命令、里程计和 ROS 接口。该边界避免 Linux 调度抖动进入电机 PID。

> 旧 URDF 的正 X 指向驱动轮一侧，但实车车头是万向轮一侧。v2 使用
> `base_footprint` 作为驱动轴中心，并旋转旧 `base_link` 180 度，使 ROS
> 正 X 指向万向轮。首次架空测试仍需确认左右轮名称和正负号。

## v2 里程计数据流

1. STM32 的 100 Hz 任务读取左右编码器增量并累计为 int32 计数。
2. STM32 以 50 Hz 上报累计计数、滤波 rpm、MPU6050 yaw 和状态位。
3. ROS 使用累计计数差计算左右轮实际路程，不积分滤波 rpm。
4. `/wheel/odom` 只使用双轮差速模型，适合检查轮径、轮距和方向。
5. 默认 `/odom` 使用轮速角增量与 IMU yaw 增量互补融合。
6. `use_ekf:=true` 时，由 `robot_localization` 融合 `/wheel/odom` 和
   `/imu/data` 并发布 `/odom` 与 `odom -> base_footprint`。

前万向轮是被动支撑，不进入理想差速运动学公式。它造成的偏差主要表现为
地面打滑、载荷不均、转向阻力和“有效轮距”变化，应通过实车标定与协方差
表达，而不是给里程计增加第三个主动轮约束。

## 已确认的底盘几何

坐标原点的 X/Y 位于驱动轮轴线中点，`base_footprint` 的 Z 位于地面；
`base_link` 位于轴心高度 24 mm。

| 项目 | 数值 |
| --- | --- |
| 整车外宽（含轮） | 148 mm |
| 轮宽 | 19 mm |
| 驱动轮中心距初值 | 129 mm |
| 整车长度 | 146 mm |
| 轴线到车尾 | 24 mm |
| 轴线到车头 | 122 mm |
| 驱动轮直径 | 48 mm |
| 理论轮周长 | 150.796 mm |
| 地板实测有效轮周长 | 144.765 mm（1 m 指令实走 0.96 m） |
| 万向轮接地点 | `x=+98 mm` |
| 雷达旋转中心 | `x=+80 mm, y=0, z=轴线上方130 mm` |
| 雷达朝向 | yaw=`pi`，扫描 0 度指向车尾 |
| MPU6050 安装 | 元件面朝下，默认反转 yaw 符号 |

导航 footprint 使用 `x=[-0.024, +0.122]`、
`y=[-0.074, +0.074]`，另加 10 mm 安全 padding。轮周长已按首次地板
直行测试乘以 `0.96` 标定；轮距仍需通过原地旋转实验校准有效值。

## 首次实车验证顺序

1. 架空车轮，发送小幅正 `linear.x`，确认车头向万向轮方向运动。
2. 发送正 `angular.z`，确认从上方观察为逆时针旋转。
3. 若左右通道交换，设置 `swap_left_right`；方向相反则调整对应 invert 参数。
4. 手动转动车轮一整圈，检查累计计数差是否约为
   `encoder_counts_per_wheel_rev`，当前初值为 1040。
5. 暂时设置 `use_imu_yaw: false`，先标定轮周长和有效轮距。
6. 再启用 IMU，确认 `/imu/data` 的正 yaw 与 ROS 逆时针正方向一致。
7. 最后比较 `/wheel/odom` 与 `/odom`，稳定后再尝试 `use_ekf:=true`。

MPU6050 当前按 Z 轴朝下处理，`invert_imu_yaw` 默认设为 `true`。架空测试
正 `angular.z` 时，`/imu/data` yaw 应增加；若实测已经增加，说明固件层
已完成符号修正，应将该参数改回 `false`。

## 当前目录规划

- `ros_diff_v1/`：旧项目只读参考，不在 v2 里继续修改。
- `ros_diff_v2/stm32f1/`：已经重构中的 STM32F1 下位机工程。
- `ros_diff_v2/catkin_ws/`：新的 ROS Noetic 上位机工作空间，源码在 `src/`。

## v2 串口协议结论

STM32 v2 使用 `0x55 0xaa` 帧头。下行命令固定 11 字节，上行反馈固定
25 字节，两者均使用 CRC-8/MAXIM。命令与反馈轮速都是输出轴
`rpm * 100`；反馈还包含左右累计编码器计数、yaw、模式状态、电池 mV、
估算百分比和电池状态。完整字节布局以
[serial_protocol_v2.md](serial_protocol_v2.md) 为准。

STM32 电池检测默认按 PB1/ADC1_IN9、47k/10k 分压和 2S 锂电配置，每
500 ms 更新一次。若使用 3S，烧录前必须把 `BATTERY_CELL_COUNT` 改为 3；
实测电压与万用表对比后可微调 `BATTERY_CALIBRATION_SCALE`。

`goto_1m()` 保留为台架标定功能，但默认不开机执行。它使用左右累计编码器
计数计算距离，并根据两轮行程差修正直线、临近目标减速，同时带 3 秒堵转
和按测试距离计算的总超时。需要自动上电测试时才临时将
`MOVE_AUTO_START_1M_TEST` 设为 1，并确保车辆前方至少有 1.5 m 空地；
正常 ROS 固件必须保持为 0。测试时不要启动 `base_controller`，因为它发送
的安全零速帧也是有效命令，会夺回 REMOTE 控制并终止 AUTO 测试。

## 已完成的 ROS v2 适配

- 建立标准 catkin 结构：`ros_diff_v2/catkin_ws/src`。
- 保留旧 `myrobot` 包名，减少 launch 和 RViz 配置迁移成本。
- 新增 `chassis_serial` 串口类，按 v2 固定帧解析，不再使用旧版 `read_until("\r\n")` 阻塞读取。
- 新 `base_controller` 同时负责：
  - 订阅 `/cmd_vel`
  - 50 Hz 下发左右轮 rpm 命令
  - 接收 STM32 左右轮 rpm 和 yaw
  - 发布 `/odom`
  - 发布 `odom -> base_link` TF
  - 兼容发布 `/car_state`
- launch 拆分：
  - `bringup.launch`：车端底盘 + robot_state_publisher + YDLIDAR-X2
  - `gmapping.launch` / `mapping_v2.launch`：建图
  - `navigation.launch` / `navigation_v2.launch`：AMCL + move_base
  - `remote_rviz.launch`：远程电脑查看 RViz
- v2 launch 默认不启动 RViz，适合树莓派 4B 2GB + Ubuntu Server。
- `config/ydlidar_x2.yaml` 按 YDLIDAR 官方 `ydlidar_ros_driver` 的 X2 launch 默认值整理，项目内只把 frame 统一为 `laser`。

## 旧版主要风险点

- `/dev/chasis` 硬编码且拼写不统一，v2 改为参数 `/dev/chassis`。
- 旧串口对象在全局静态初始化时打开设备，设备不存在会直接崩。
- 旧 `base_controller` 和 `odm` 分成两个节点，中间只传 `vx/w`，定位问题时数据链较绕。
- yaw 没有处理 `-180/+180` 跳变，转过边界时可能产生瞬时大角速度。
- `local_costmap` 旧配置使用 `map` 作为局部代价地图坐标，v2 改为更常见的 `odom`。
- 旧 launch 在树莓派端启动 RViz，会明显占内存和 GPU/CPU，但它更像性能风险，不是里程计乱跳的根因。

## 树莓派端建议启动方式

```bash
cd ~/ros_diff_v2/catkin_ws
catkin_make
source devel/setup.bash
roslaunch myrobot bringup.launch
```

建图：

```bash
roslaunch myrobot gmapping.launch
```

导航：

```bash
roslaunch myrobot navigation.launch map_file:=/path/to/map.yaml
```

远程电脑看 RViz：

```bash
export ROS_MASTER_URI=http://<raspberry_pi_ip>:11311
export ROS_IP=<pc_ip>
roslaunch myrobot remote_rviz.launch
```

## 下一步实车校准顺序

1. 串口设备名固定：建议用 udev 把 STM32 固定为 `/dev/chassis`，YDLIDAR 固定为 `/dev/ydlidar`。
2. 空载架起车轮，运行 `robot_remote.launch enable_lidar:=false`，发送小速度，确认左右轮方向。
3. 如果方向反了，优先改 `config/base_controller.yaml` 里的 `invert_left_command/right_command` 和 `invert_left_feedback/right_feedback`。
4. 实测轮距 `wheel_separation` 和轮周长 `wheel_circumference`。
5. 只跑 `bringup.launch`，用 `rostopic echo /odom` 验证：
   - 直行时 `x` 增加，`y` 基本不飘。
   - 原地左转时 yaw 正向变化。
   - yaw 跨过 180 度时 `/odom.twist.twist.angular.z` 不出现巨大尖峰。
6. 再跑建图，先不用导航目标。
7. 地图稳定后再跑 `navigation.launch`，逐步提高 `DWAPlannerROS` 的速度/角速度限制。

## 待确认硬件参数

- 有效轮距，当前几何初值：`0.129 m`，仍需原地旋转标定。
- 有效轮周长，当前地板标定值：`0.144764589 m`，建议重复 3 至 5 次
  一米测试并用平均距离继续微调。
- YDLIDAR-X2 安装方向：如果 scan 在 RViz 里前后反，优先检查雷达朝向和 `robot.urdf.xacro` 中 `base_laser_joint` 的 yaw。
- STM32 左右电机正方向：ROS 可通过 YAML 反向，后面稳定后也可以固化到下位机。
