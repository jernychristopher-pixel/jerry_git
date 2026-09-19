# SlamTest 项目注意事项

## 烧录布局切换（有线/无线）

工程有两种烧录模式，切换时**两处文件要同步改**，不能只改一个。

### A. 有线 ST-Link 直烧（无 BootLoader）
- `STM32F103XX_FLASH.ld`
  - `FLASH (rx) : ORIGIN = 0x8000000, LENGTH = 256K`
- `Core/Src/system_stm32f1xx.c`
  - `VECT_TAB_OFFSET = 0x00000000U`（写在 FLASH 分支）

### B. 无线烧录（BootLoader + APP）
- BootLoader 固定烧在 `0x08000000`（占 8KB）
- `STM32F103XX_FLASH.ld`
  - `FLASH (rx) : ORIGIN = 0x8002000, LENGTH = 248K`
- `Core/Src/system_stm32f1xx.c`
  - 启用 `USER_VECT_TAB_ADDRESS`
  - `VECT_TAB_OFFSET = 0x00002000U`（写在 FLASH 分支）

## 关键坑：VECT_TAB_OFFSET 出现两次

`Core/Src/system_stm32f1xx.c` 里 `VECT_TAB_OFFSET` 有两个定义：
- `#if defined(VECT_TAB_SRAM)` 分支 → SRAM 分支，默认注释掉，**不生效**
- `#else` 分支 → FLASH 分支，**真正生效**

改偏移量时必须改 `#else`（FLASH）分支，绝不能改 SRAM 分支。
历史错误：把 `0x2000` 写进了 SRAM 分支，导致生效的 FLASH 分支一直是 `0`，VTOR 一直是 `0x08000000`。

## 其他提醒
- ST-Link 整片擦除会擦掉 BootLoader；无线模式下不要整片擦除。
- 有线直烧模式直接用 ST-Link 烧 `SlamTest.hex` 即可。
- 无线模式用 WHEELTEC 无线烧录工具烧 `SlamTest.bin`。

## 上下位机协议（改帧布局前必读）

`docs/chassis_protocol_spec.md` 是**唯一权威来源**，改任何帧字段前先读它。

上行反馈帧是 **37 字节**。改帧长或布局必须同步 6 个位置，缺一不可：

| # | 文件 |
|---|---|
| 1 | `Lib/Inc/protocol.h`（布局注释 + struct） |
| 2 | `Lib/Src/protocol.c`（`TX_LEN` / `RX_LEN` + 组帧） |
| 3 | `Core/Src/main.c`（填新字段） |
| 4 | `chassis_link_test.py`（桌面 vmware share 与 slam-car-hmi/bridge 各一份） |
| 5 | `check.py` / `pi_check.py`（`Desktop\脚本`） |
| 6 | `slam-car-hmi/bridge/chassis_driver.py`（ROS2 节点） |

改完**必须**跑自检，它会把上面这些文件全部对一遍并造帧验证：

```
python tools/verify_protocol.py
```

历史教训：24 -> 33 字节那次只改了固件，四个上位机脚本全部按 24 字节解析，
静默失效、不报错。详见规范 §10。

## OLED 显示开关

`Lib/Inc/app_config.h` 里的 `APP_USE_OLED` 是唯一开关（默认 `0` = 关）：

- `0`：`Lib/Src/oled.c` 整体编译为空文件，`oled.h` 里所有 `OLED_xxx()` 退化成空操作，
  `Core/Src/main.c` 里那段周期刷屏块被 `#if APP_USE_OLED` 排除。
  省约 2.5KB FLASH（50840 -> 48280B），主循环不再被软件 SPI 占用十几毫秒。
- `1`：正常驱动 OLED（软件 SPI：PB3=DC PB4=RES PB5=SDA PC14=SCL）。

只改这一个宏即可，不需要动 `CMakeLists.txt`。屏幕物理拆除/装回时同步改。
开启后 OLED 上原本的 IMU 信息（YAW / Ry / Wc / E）已改由 USART1 上行帧下发，见 `Lib/Inc/protocol.h`。
