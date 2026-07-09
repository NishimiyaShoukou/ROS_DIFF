# Chassis Debug Notes

## Hardware

- Motor: MC310P20_V704_R13 / JGA25-310
- Gear ratio: 1:20
- Rated voltage: 7.4 V
- Rated speed: 450 rpm
- Rated current: <= 0.5 A
- Stall current: <= 2.0 A
- Encoder: AB quadrature magnetic encoder, 13 lines
- Driver: TB6612FNG

## 2026-07-08 Findings

- Encoder/motor side mapping was corrected earlier: physical left wheel maps to TIM4, physical right wheel maps to TIM2.
- The speed conversion code was still using `13 * 30`, but the current motor is 20:1. It has been changed to named constants:
  - encoder lines: 13
  - reduction ratio: 20
- Previous PID datasets before this ratio fix are not directly comparable with new datasets.
- Latest low-speed dataset (`target=20`) looked stable:
  - left average speed: 18.96
  - right average speed: 19.04
  - stop command reached zero cleanly
- A motor runaway event was reported but not captured in telemetry. A likely software risk was open-loop `$MOT` holding the last PWM indefinitely. Open-loop PWM now has a 1000 ms timeout.

## Follow-up

- Re-test open-loop PWM after the ratio fix.
- Re-test closed-loop targets 20, 40, 60 with the corrected speed scale.
- Before ROS navigation, add or verify command timeout for upper-computer `cmd_vel`.
