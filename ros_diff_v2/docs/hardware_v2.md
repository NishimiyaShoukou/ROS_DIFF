# ROS_DIFF v2 Hardware Notes

本文记录 v2 实车当前使用的尺寸、坐标、引脚和需要继续标定的项目。

## Coordinate Frames

ROS 约定：

- `base_footprint`：驱动轮轴线中点在地面的投影，作为 2D 运动学原点。
- `base_link`：车体坐标，正 X 指向小车前方，也就是万向轮方向。
- `laser`：YDLIDAR-X2 坐标系。
- `imu_link`：MPU6050 坐标系。

前万向轮只是被动支撑，不进入理想差速运动学公式。它主要通过摩擦、偏载和地面打滑影响“有效轮距”和轨迹，应通过实车标定补偿。

## Dimensions

| 项目 | 当前值 |
| --- | --- |
| 整车外宽，含轮子 | 148 mm |
| 轮宽 | 19 mm |
| 驱动轮中心距初值 | 129 mm |
| 整车长度 | 146 mm |
| 驱动轮轴线到车尾 | 24 mm |
| 驱动轮轴线到车头 | 122 mm |
| 驱动轮直径 | 48 mm |
| 理论轮周长 | 150.796 mm |
| 地板测试有效轮周长 | 144.765 mm |
| 万向轮接地点 | `x=+98 mm, y=0` |
| 雷达旋转中心 | `x=+80 mm, y=0, z=+130 mm` |
| 雷达朝向 | `yaw=pi`，扫描 0 度朝车尾 |

导航 footprint 当前按车体外形估计，并额外留少量 padding。狭窄场地导航时，footprint 和 costmap inflation 会明显影响能否通过。

## Motor And Encoder Mapping

当前固件默认的电机驱动和编码器引脚分配如下。`MOTOR1`、`MOTOR2` 为本工程中的电机接口命名；如果使用不同 PCB 或电机驱动板，请以自己的原理图为准，并同步修改 STM32 工程中的电机 PWM、方向脚和编码器配置。

| 轮子 | 电机接口 | TB6612 通道 | PWM | 方向脚 | 编码器 |
| --- | --- | --- | --- | --- | --- |
| 左轮 | `MOTOR1` | B channel | `PA8 / TIM1_CH1 / PWMB` | `PB14 / BIN1`, `PB15 / BIN2` | `PB6/PB7 / TIM4` |
| 右轮 | `MOTOR2` | A channel | `PA9 / TIM1_CH2 / PWMA` | `PB12 / AIN2`, `PB13 / AIN1` | `PA0/PA1 / TIM2` |

移植或重新接线后，建议先做低速单轮验证：

1. 不启动 ROS，用 PID 调试网页或串口命令单独给左/右轮小速度。
2. 确认目标 rpm、实际转向和编码器反馈方向一致。
3. 再接 ROS，测试正 `linear.x` 是否前进，正 `angular.z` 是否从上往下看逆时针左转。

ROS YAML 里的 `invert_*` 参数适合做最终方向补偿；底层引脚、PWM 通道和编码器通道仍应优先在 STM32 侧保持清晰一致。

## IMU

当前 MPU6050 元件面朝下安装。ROS 配置中默认：

```yaml
invert_imu_yaw: true
```

因为 IMU 安装和初始化仍需继续确认，建图导航默认建议先使用：

```bash
use_imu_yaw:=false
```

后续验证方法：

1. 架空或低速原地左转。
2. 观察 `/imu/data` yaw 是否按 ROS 正方向增加。
3. 对比 `/wheel/odom` 和 `/odom`，确认 IMU 融合没有让轨迹突然跳变。

## Battery Measurement

STM32 侧电池检测当前使用：

- ADC 引脚：`PB1 / ADC1_IN9`
- 分压：`47k + 10k`
- 默认电池类型：`2S`
- 更新周期：约 500 ms

不要把电池直接接到 PB1，必须经过分压。若改为 3S，需要在固件中把 `BATTERY_CELL_COUNT` 改为 3，并用万用表校准 `BATTERY_CALIBRATION_SCALE`。
