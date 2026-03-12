# Teensy 4.1 (DEV-16771) 引脚与驱动选型建议

## 1. 板卡结论
- 你选的 `Teensy 4.1 DEV-16771 (i.MX RT1062 Cortex-M7)` 合适。
- `Arduino UNO` 不建议用于本项目主控（通道数、算力、实时余量不足）。

## 2. 官方能力摘要（用于选型边界）
- 主频：600MHz Cortex-M7
- I/O：55
- 串口：8
- PWM：35
- CAN：3（其一支持 CAN FD）
- 关键电气约束：IO 仅 3.3V，非 5V tolerant

参考：
- SparkFun DEV-16771 页面
- PJRC Teensy 4.1 页面与 pinout card

## 3. 建议引脚分配（当前代码默认）
- Jetson 通信：
  - USB CDC: `Serial`（默认）
  - TTL UART: `Serial1`（`RX1/TX1`，可在代码切换）
- 急停输入：
  - `ESTOP_PIN = 22`（`INPUT_PULLUP`，低电平触发）
- 升降台：
  - `LIFT_PWM_PIN = 2`
  - `LIFT_DIR_PIN = 3`
- Stewart 六执行器（占位）：
  - PWM: `4,5,6,7,8,9`
  - DIR: `10,11,12,13,14,15`

说明：
- 以上是“可跑框架”的默认映射，最终需按你买的驱动器接口改。
- 若使用 CAN 执行器，PWM/DIR 引脚会大幅减少。

## 4. 驱动型号推荐（按执行器类型）

## 4.1 若 Stewart/升降是有刷直流线性执行器（带编码器）
- 推荐：
  - RoboClaw 2x15A / 2x30A（自带编码器闭环能力，串口/USB配置成熟）
  - Cytron 系列（如 MDDS/MD30C）+ 外部编码器闭环（需 MCU 实现 PID）
- 不推荐长期方案：
  - BTS7960 大量并联方案（可跑，但工程一致性和可靠性一般）

## 4.2 若 Stewart 是步进电机丝杠方案
- 推荐：
  - TMC5160（高压电流能力更好，适合较大负载）
  - TMC2209（轻载可用，成本低）
- 备注：
  - 步进失步风险需靠限位与原点流程管理

## 4.3 若 Stewart 是 BLDC/伺服方案（高性能）
- 推荐：
  - ODrive 系列（CAN/UART）
  - 工业伺服驱动器（脉冲+方向或总线）
- 备注：
  - 成本高，但性能和稳定性最好

## 5. 你的项目推荐路径（优先可落地）
1. V1 快速闭环：
   - Teensy 4.1 + RoboClaw（默认推荐路径）
2. V2 稳定升级：
   - 迁移到 CAN 伺服/BLDC，降低布线复杂度并提升可靠性

## 6. 与现有代码对应
- 控制代码文件：
  - `jetson_stewart_lift_controller.ino`
- 需要你按驱动器替换的函数：
  - `readLiftPositionM()`
  - `writeLiftCommand()`
  - `readLegLengthM()`
  - `writeLegCommand()`
  - `readEstopPin()`

## 7. RoboClaw 默认总线建议
- 控制器：Teensy `Serial2` <-> RoboClaw packet serial
- 波特率：`115200`
- 地址建议：
  - 升降台：`0x80`
  - Stewart 六执行器：`0x81 ... 0x86`
- 电机通道：
  - 每个 RoboClaw 优先使用 `M1` 作为单执行器通道（简化初版）
