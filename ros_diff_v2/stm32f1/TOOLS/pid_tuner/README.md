# 底盘 PID 调试台

## 打开

推荐使用 Chrome 或 Edge 访问：

```text
http://127.0.0.1:8765/
```

也可以直接双击：

```text
TOOLS\pid_tuner\start_pid_tuner.bat
```

脚本会启动本地网页服务器并自动打开浏览器。调参时保持黑色窗口打开，关闭它即可停止。

默认串口参数为 `115200 8N1`，网页顶部可以切换波特率。

## 串口文本协议

网页发送：

```text
$DBG,1
$PID,index,kp1000,ki1000,kd1000,max_out,max_step
$SPD,left100,right100
$MOT,left_pwm,right_pwm
$IMU,enable
$STOP
$GET
```

`index`：

- `0` 左轮
- `1` 右轮
- `2` 左右同时

例子：

```text
$PID,2,10000,500,0,1000,50
$SPD,500,500
```

表示左右速度环 `Kp=10.0, Ki=0.5, Kd=0.0`，目标速度 `5.00, 5.00`。

开环电机测试：

```text
$MOT,300,0
$MOT,0,300
```

表示跳过速度 PID，直接输出左/右电机 PWM。用于确认电机输出和编码器反馈是否一一对应。

STM32 回传：

```text
$TEL,time_ms,targetL100,targetR100,speedL100,speedR100,pwmL,pwmR,kpL1000,kiL1000,kdL1000,kpR1000,kiR1000,kdR1000
```

IMU 监控打开后，STM32 额外回传：

```text
$IMU,time_ms,ready,error,fail,pitch100,roll100,yaw100,aacx,aacy,aacz,gyrox,gyroy,gyroz,temp100
```

调参数据可以用网页里的 `复制 CSV` 发给我分析。
