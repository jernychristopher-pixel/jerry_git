/**
  ******************************************************************************
  * @file    app_config.h
  * @brief   应用级编译开关 (集中放这里, 避免散落在各文件里)
  ******************************************************************************
  */

#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

/**
  * OLED 显示屏开关 (SSD1306 128x64, 软件SPI: PB3=DC PB4=RES PB5=SDA PC14=SCL)
  *   0 = 不编译 OLED 驱动, 不占用主循环时间, 不刷新屏幕 (当前屏幕已物理拆除)
  *   1 = 正常驱动 OLED 显示
  * 说明: 这里是唯一开关。置 0 时 oled.c 编译为空文件,
  *       所有 OLED_xxx() 调用在 oled.h 里退化成空操作, 调用方代码无需改动。
  */
#define APP_USE_OLED   0

/**
  * M4-4 协议模式开关
  *   0 = 原 SlamTest 协议 (0x7B/0x7D + BCC, 11 字节下行 / 37 字节上行)
  *   1 = M4-4 协议 (0xAA55 + len + id + seq + payload + CRC16)
  * 两种协议共用 USART1, 不能同时开启。
  * M4-4 现场编译时把它改成 1; 日常跑车保持 0。
  */
#ifndef APP_USE_M44_PROTOCOL
#define APP_USE_M44_PROTOCOL 0
#endif

#endif /* __APP_CONFIG_H */
