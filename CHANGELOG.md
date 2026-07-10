# Changelog

本项目采用语义化版本：`MAJOR.MINOR.PATCH`。v1 作为历史快照保留；当前公开主线为 v2。

## [Unreleased]

### Planned

- 继续标定有效轮距、IMU yaw 和导航参数。
- 根据更多环境测试继续完善导航参数。

## [2.2.0] - 2026-07-10

### Added

- 增加 `myrobot_web` 平板网页控制台，支持触摸遥控、实时地图、机器人位姿、路径显示、建图保存和导航目标操作。
- 增加底盘、建图、导航三种模式的互斥管理服务，以及手动速度优先的 `/cmd_vel` 仲裁。
- 增加网页断连、失焦、松手停车和一键急停保护。

### Changed

- `navigation_v2.launch` 支持重映射 move_base 速度话题，普通启动方式保持兼容。
- 根据实车测试调整 AMCL、DWA 和局部代价地图参数，并降低导航速度与角速度。
- 项目文档增加平板导航实机截图和移动控制部署说明。

### Fixed

- 拆分命令与反馈的左右轮交换参数，修正实车左转时里程计角速度符号相反的问题。
- 默认关闭未完成姿态验证的 IMU yaw 融合。
- 将示例地图图像路径改为相对路径，避免换机器后 map_server 找不到 PGM 文件。

## [2.1.0] - 2026-07-10

### Added

- 增加面向开源读者的 v2 文档：硬件、树莓派部署、建图导航、串口协议和故障排查。
- 增加 STM32 ADC 电池电压采样、低电量状态和 ROS `/battery_state` 发布。
- 保留并修复 `goto_1m()` 作为台架/地板直行标定功能。
- 增加 PID 调试网页，用于独立验证左右轮速度、方向和串口遥测。
- 增加 URDF 参数可视化编辑工具，方便后续迁移底盘尺寸。

### Changed

- v1 归档到 `ros_diff_v1/`，v2 作为当前主线放在 `ros_diff_v2/`。
- ROS 上位机使用 Boost.Asio 自定义串口协议，不依赖 rosserial。
- 串口协议升级为 v2.1，双向帧均使用 CRC-8/MAXIM。
- 里程计改用驱动轮轴线中心作为 `base_footprint`，前万向轮不参与理想差速运动学。
- 根据实测尺寸更新轮距、轮径、雷达位姿和 costmap footprint。
- 根据 1 m 地板直行测试，把有效轮周长统一为 `0.144764589 m`。
- 建图和导航入口默认推荐 `use_imu_yaw:=false`，待 IMU 完成验证后再开启。

### Fixed

- 修正 v2 PCB 上 MOTOR1/MOTOR2 与 TB6612 A/B 通道的映射：左轮 MOTOR1 使用 `PA8 + PB14/PB15`，右轮 MOTOR2 使用 `PA9 + PB12/PB13`。
- 修复 Ubuntu 20.04 / Boost 1.71 下 `serial_port::available()` 不存在导致的编译失败。
- 修复 Noetic launch 中布尔参数求值问题，避免 EKF 未启用时缺失 `odom -> base_footprint`。
- 修复 PID 调试串口命令缓冲过小导致连续命令丢失的问题。
- 修复 `goto_1m()` 可能在循环中重复启动、抢占遥控模式的问题。

### Breaking

- v2.1 ROS 节点和 STM32 固件必须同步更新。旧版无 CRC 下行命令和旧版短上行反馈帧均不兼容。

## [2.0.0] - 2026-07-09

### Added

- 建立新的 v2 工程目录，面向 ROS Noetic、STM32F1 和 YDLIDAR-X2。
- 新增 ROS `base_controller`、`bringup.launch`、`mapping_v2.launch`、`navigation_v2.launch` 和基础导航配置。
- 新增左右轮累计编码器计数上报，ROS 里程计不再依赖滤波后的 rpm 积分。
- 新增可选 `robot_localization` EKF 配置。

### Changed

- 根目录作为统一 Git 仓库，同时管理 v1 历史版本和 v2 当前版本。
- STM32 保持电机闭环和安全超时，树莓派负责 ROS 接口、里程计、建图和导航。

### Breaking

- v2.0.0 起串口帧格式与 v1 不兼容。
