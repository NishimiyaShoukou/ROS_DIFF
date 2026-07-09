# Contributing

ROS_DIFF 目前以 v2 为主线，v1 仅作为历史参考。欢迎提交问题、文档修正、移植记录和代码改进。

## Branches

- `main`：稳定主线。README、文档和代码应保持互相对应。
- `feature/...`：新功能或实验，例如 `feature/imu-calibration`。
- `fix/...`：明确的问题修复，例如 `fix/ydlidar-launch`。

当前不额外维护长期 `v1`/`v2` 分支，因为仓库目录已经分成 `ros_diff_v1/` 和 `ros_diff_v2/`。

## Releases

发布一个可回退版本时，请同步更新：

1. `VERSION`
2. `CHANGELOG.md`
3. `ros_diff_v2/catkin_ws/src/myrobot/package.xml`
4. Git tag，例如 `v2.1.0`

建议 tag 对应功能相对完整、文档同步更新的版本。

## Development Notes

- v2 ROS 上位机和 STM32 固件必须配套测试，串口协议不兼容无 CRC 帧。
- 电机方向问题先用 STM32/PID 调试网页确认，再调整 ROS 参数。
- Raspberry Pi 4B 2GB 上尽量不跑 RViz，推荐远程电脑查看。
- 提交前尽量确保 `catkin_make` 通过，并说明改动影响的硬件或 ROS 模块。
