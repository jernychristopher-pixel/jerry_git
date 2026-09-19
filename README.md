# SlamTest

基于 STM32F103 的双轮差速底盘固件。实现速度闭环、IMU 航向保持、PS2 手柄遥控，
并通过二进制串口协议与上位机（树莓派 / PC）对接，作为 SLAM 小车的运动执行层。

## 目录

- [功能特性](#功能特性)
- [硬件平台](#硬件平台)
- [控制架构](#控制架构)
- [编译](#编译)
- [烧录](#烧录)
- [上下位机协议](#上下位机协议)
- [目录结构](#目录结构)
- [开发约定](#开发约定)
- [相关文档](#相关文档)

## 功能特性

- **双轮差速运动学** —— 由 `V`（线速度）、`W`（角速度）逆解出左右轮目标，轮距 115mm、轮径 65mm。
- **左右独立速度闭环** —— 编码器实测脉冲经增量式 PID 得出 PWM 占空比，两轮各自闭环。
- **IMU 航向保持** —— MPU6050 配合 DMP 输出偏航角，直行时自动修正角速度，抑制左右轮速差导致的跑偏。
- **双控制源互斥** —— 手动（手柄）与自动（上位机）共用 `Chassis_Set_Cmd()` 一个下发入口；
  收到任一合法下行帧即切入 AUTO 模式，按手柄 START 键交还手动控制。
- **PS2 手柄** —— 按键 / 摇杆两种操作模式（TRIANGLE 键切换），三挡速度增益。
- **上行反馈帧** —— 37 字节回传底盘速度、左右轮速、IMU 原始数据、航向环状态与电池电压。
- **VOFA+ 在线调参** —— USART3 输出 8 通道文本波形，支持 `#` 命令在线改 PID 与刹车参数。
- **电池电压监测** —— PC1 采样，11:1 分压。
- **OLED 可选** —— 由 `APP_USE_OLED` 单一宏控制，关闭时节省约 2.5KB FLASH 并释放主循环时间。

## 硬件平台

| 项目 | 说明 |
|---|---|
| MCU | STM32F103RCT6（LQFP64，72MHz，256KB FLASH / 48KB RAM） |
| 主控板 | WHEELTEC C10B |
| 电机驱动 | AT8236 |
| 姿态传感器 | MPU6050（软件 I2C） |
| 遥控器 | PS2 手柄 |
| 上位机 | 树莓派 / PC |

### 外设与引脚分配

| 外设 | 引脚 | 用途 |
|---|---|---|
| TIM1 | — | 控制环时基，40Hz（25ms） |
| TIM3 CH1 / CH2 | PA6 / PA7 | 左电机 IN1 / IN2 |
| TIM3 CH3 / CH4 | PB0 / PB1 | 右电机 IN2 / IN1 |
| TIM4 CH1 / CH2 | PB6 / PB7 | 左轮编码器 |
| TIM8 CH1 / CH2 | PC6 / PC7 | 右轮编码器 |
| USART1 | PA9 / PA10 | 上下位机协议，115200 8N1 |
| USART3 | PB10 / PB11 | VOFA+ 波形输出，230400 8N1 |
| ADC1_IN11 | PC1 | 电池电压，11:1 分压 |
| 软件 I2C | PB14 / PB15 | MPU6050（SCL / SDA） |
| PS2 | PB8 / PC9 / PC4 / PC8 | DI / DO / CS / CLK |
| 软件 SPI | PB3 / PB4 / PB5 / PC14 | OLED（DC / RES / SDA / SCL），默认关闭 |

## 控制架构

```text
   PS2 手柄                     上位机（树莓派 / PC）
       │                                │
       │ 手动命令                        │ 11 字节下行帧 @115200
       ▼                                ▼
  ┌──────────────────────────────────────────┐
  │   Chassis_Set_Cmd(V, W, src)             │  唯一命令入口
  │   手动模式 / AUTO 模式互斥                 │
  └────────────────────┬─────────────────────┘
                       │  逆运动学（轮距 115mm）
                       ▼
  ┌──────────────────────────────────────────┐
  │   TIM1 控制环 @ 40Hz (25ms)              │
  │     ├── 航向保持：IMU 偏航角修正 W         │
  │     ├── 左右轮速度 PID                    │
  │     └── PWM 输出                         │
  └────────────────────┬─────────────────────┘
                       ▼
              TIM3 → AT8236 → 直流电机
                       │
              编码器 TIM4 / TIM8
                       │
                       ▼
  ┌──────────────────────────────────────────┐
  │   37 字节上行反馈帧 → USART1              │
  │   8 通道文本波形   → USART3（VOFA+）      │
  └──────────────────────────────────────────┘
```

## 编译

需要 CMake ≥ 3.22、Ninja 与 arm-none-eabi-gcc（GNU Tools for STM32）。

```bash
cmake --preset Debug
cmake --build build/Debug
```

Release 构建把 `Debug` 换成 `Release` 即可。

构建产物（每次编译自动生成）：

| 文件 | 用途 |
|---|---|
| `build/Debug/SlamTest.elf` | 调试用，带符号 |
| `build/Debug/SlamTest.bin` | 无线烧录工具使用 |
| `build/Debug/SlamTest.hex` | ST-Link 烧录 / 离线备份 |

## 烧录

工程支持两种烧录方式。**切换时下表两处文件必须同步修改，只改一个会导致跑飞。**

| 模式 | `STM32F103XX_FLASH.ld` | `system_stm32f1xx.c` |
|---|---|---|
| 有线 ST-Link 直烧 | `ORIGIN = 0x8000000, LENGTH = 256K` | `VECT_TAB_OFFSET = 0x00000000U` |
| 无线（BootLoader + APP） | `ORIGIN = 0x8002000, LENGTH = 248K` | 启用 `USER_VECT_TAB_ADDRESS`，`VECT_TAB_OFFSET = 0x00002000U` |

> **注意**：`system_stm32f1xx.c` 中 `VECT_TAB_OFFSET` 出现两次 —— SRAM 分支（默认注释掉，不生效）
> 与 FLASH 分支（真正生效）。改偏移量必须改 **FLASH 分支**。

- 无线模式下 BootLoader 固定占用 `0x08000000` 起 8KB，**不要整片擦除**，否则 BootLoader 会被抹掉。
- 有线模式直接用 ST-Link 烧 `SlamTest.hex`；无线模式用 WHEELTEC 无线烧录工具烧 `SlamTest.bin`。

## 上下位机协议

完整定义见 [`docs/chassis_protocol_spec.md`](docs/chassis_protocol_spec.md)，**该文件是协议的唯一权威来源**。
代码与文档冲突时以文档为准并修改代码。

坐标系与 ROS REP-103 一致：`+vx` 为车头方向，`+wz` 为逆时针（左转）。
线路单位固定为线速度 `mm/s`、角速度 `mrad/s`、转角 `0.01度`、电压 `mV`，整数一律大端有符号。

### 下行控制帧（上位机 → 底盘，11 字节）

| 偏移 | 长度 | 字段 |
|---|---|---|
| 0 | 1 | `0x7B` |
| 3 | 2 | `X` 线速度 mm/s，前进为正 |
| 5 | 2 | `Y` 线速度 mm/s，差速车必须填 0 |
| 7 | 2 | `Z` 角速度 mrad/s，左转为正 |
| 9 | 1 | BCC = 前 9 字节异或 |
| 10 | 1 | `0x7D` |

上位机必须以 20 ~ 50 Hz 持续发送；AUTO 模式下 200ms 收不到新包即自动减速停车。

### 上行反馈帧（底盘 → 上位机，37 字节）

| 偏移 | 长度 | 字段 | 单位 |
|---|---|---|---|
| 0 | 1 | `0x7B` | |
| 1 | 1 | `flag_stop` | 0=电机使能 |
| 2 | 2 | `vx_mm_s` | mm/s，前进为正 |
| 6 | 2 | `wz_mrad_s` | mrad/s，左转为正 |
| 8 ~ 18 | 12 | `acc_*` / `gyro_*` | MPU6050 原始值 |
| 20 | 2 | `battery_mv` | mV，0=未接 |
| 22 | 2 | `yaw_001deg` | 0.01度 |
| 24 | 2 | `yaw_rate_mrad_s` | mrad/s |
| 26 | 2 | `yaw_hold_err_001deg` | 0.01度 |
| 28 | 2 | `yaw_hold_out_mrad_s` | mrad/s |
| 30 | 1 | `status` | 位域 |
| 31 | 2 | `vl_mm_s` | 左轮实测 mm/s |
| 33 | 2 | `vr_mm_s` | 右轮实测 mm/s |
| 35 | 1 | BCC = 前 35 字节异或 | |
| 36 | 1 | `0x7D` | |

`status` 位定义：bit0 = IMU 可用，bit1 = 上位机（AUTO）控制中，bit2-3 = 挡位索引（0~2）。

**改帧布局前必须同步 6 处**：`protocol.h`、`protocol.c`、`main.c`、`chassis_link_test.py`、
`check.py` / `pi_check.py`、`slam-car-hmi/bridge/chassis_driver.py`，改完必须跑自检：

```bash
python tools/verify_protocol.py
```

## 目录结构

```text
SlamTest/
├── Core/                  CubeMX 生成的 HAL 初始化、中断与外设配置
│   ├── Inc/
│   └── Src/
├── Lib/                   自研业务代码
│   ├── Inc/
│   └── Src/               kinematics / pid / motor / imu / yaw_hold /
│                          protocol / vofa / ps2 / oled / adc ...
├── Drivers/               STM32F1xx HAL 驱动与 CMSIS
├── cmake/                 工具链文件 gcc-arm-none-eabi.cmake
├── docs/
│   └── chassis_protocol_spec.md    协议规范（权威来源）
├── tools/
│   └── verify_protocol.py          协议一致性自检
├── _at8236_example/       AT8236 电机驱动参考例程
├── _sch_png/              C10B 主板原理图
└── vofa_docs/             VOFA+ 使用说明
```

## 开发约定

- **单一权威来源**：协议以 `docs/chassis_protocol_spec.md` 为准；物理常量（轮距、轮径、编码器线数）
  只允许定义在 `Lib/Src/kinematics.c` 一处，上位机不得复制。
- **方向只翻一次**：对外坐标系与内部电极性的转换只发生在 `kinematics.c` 的
  `CHASSIS_V_SIGN` / `CHASSIS_W_SIGN` 两个宏，其它位置不得再翻符号。
- **单位换算只在一层**：固件侧集中在 `protocol.c`，上位机侧集中在解析脚本的 `decode()`。
- **改协议必跑自检**：`python tools/verify_protocol.py`，它会把固件与各上位机脚本的帧长、
  校验偏移全部对一遍，并按字节序造帧验证往返一致。

## 相关文档

| 文档 | 内容 |
|---|---|
| [`docs/chassis_protocol_spec.md`](docs/chassis_protocol_spec.md) | 上下位机协议规范，唯一权威来源 |
| [`AGENTS.md`](AGENTS.md) | 工程约束：烧录模式切换、协议同步清单、编译开关 |
| [`vofa_docs/`](vofa_docs) | VOFA+ 上位机使用说明 |
