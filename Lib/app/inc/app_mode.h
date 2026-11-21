#ifndef __APP_MODE_H
#define __APP_MODE_H

#include <stdint.h>

/*
 * 运行时工作模式: 一套固件同时支持 M4-2 / M4-3 / M4-4
 *
 * 以前 APP_MODE_M42_STREAM 是编译期开关, 交一份固件只能跑一种模式。
 * 现在改成运行期变量, 上电默认由 APP_BOOT_MODE 决定, 之后可以在线切:
 *
 *   1. 收到二进制帧头 AA 55          -> 切到 PROTOCOL (M4-3 / M4-4)
 *   2. 收到 ASCII 行 "#M42 1" + 换行 -> 切到 M42 采集 (M4-2)
 *   3. 收到 ASCII 行 "#M42 0" + 换行 -> 切回 PROTOCOL
 *
 * 两种流不会互相误判: CSV 全是可打印 ASCII, 不可能出现 0xAA;
 * 二进制帧里连续出现 "#M42 1\n" 这 7 个字节的概率可以忽略,
 * 而且切错了只要重新上电就回到默认模式。
 *
 * 切进 M42 要重新做 2 s 零偏估计, 不能在中断里做, 所以中断只改
 * g_app_mode, 真正干活的是主循环里的 AppMode_Service()。
 */

#define APP_MODE_PROTOCOL   0u
#define APP_MODE_M42        1u

extern volatile uint8_t g_app_mode;

/* 上电时调一次, 之后主循环每轮调 AppMode_Service() */
void    AppMode_Init(uint8_t boot_mode);
void    AppMode_Service(void);

/* USART1 接收中断里逐字节调用 */
void    AppMode_FeedByte(uint8_t byte);

static inline uint8_t AppMode_Get(void)
{
    return g_app_mode;
}

#endif /* __APP_MODE_H */
