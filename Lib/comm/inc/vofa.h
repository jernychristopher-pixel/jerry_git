#ifndef __VOFA_H
#define __VOFA_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

void Vofa_Start_Receive_IT(void);
void Ftoa(char *dst, float v, int dec);
void Vofa_Send(float ch0, float ch1, float ch2, float ch3,
               float ch4, float ch5, float ch6, float ch7);

/* USART1 单协议: 环形缓冲 -> 状态机 -> 速度/PID 指令 */
void Vofa_Protocol_Poll(void);

/* 每 50ms 发一帧 0x02 状态 */
void Vofa_Protocol_SendStatus(int16_t left_rpm, int16_t right_rpm,
                              int16_t pitch_001deg, int16_t yaw_001deg,
                              uint16_t battery_mv,
                              int16_t yaw_rate_mrad_s,
                              int16_t yaw_hold_out_mrad_s,
                              uint8_t status_bits);

/* 每 20ms 发一帧 0x10 遥测, 给 M4-3 采集 target/actual/pwm */
void Vofa_Protocol_SendTelemetry(int16_t target_rpm_L, int16_t actual_rpm_L,
                                 int16_t pwm_L,
                                 int16_t target_rpm_R, int16_t actual_rpm_R,
                                 int16_t pwm_R);

uint16_t Vofa_GetRxErr(void);
void Vofa_Rx_Reattach(void);

/* 模式切换时清解析器: 丢掉环形缓冲里的半截帧/残留文本 */
void Vofa_Reset_Parser(void);

#endif
