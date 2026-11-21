#ifndef __VOFA_H
#define __VOFA_H

#include "stm32f1xx_hal.h"
#include "usart.h"
#include <string.h>

#define RX_BUF_SIZE 32

void Vofa_Start_Receive_IT(void);
void Ftoa(char *dst, float v, int dec);
void Vofa_Send(float ch0, float ch1, float ch2, float ch3, float ch4, float ch5, float ch6, float ch7);
void Vofa_Service(void);

/* M4-3 采集模式: #V 1 后 USART1 暂停二进制帧, 改发 CSV 文本。
 * 列: t,target_rpm_L,actual_rpm_L,pwm_L,target_rpm_R,actual_rpm_R,pwm_R */
uint8_t Vofa_Csv_Enabled(void);
void Vofa_Send_Csv(float t, float target_l, float actual_l, float pwm_l,
                   float target_r, float actual_r, float pwm_r);

uint16_t Vofa_GetRxErr(void);
void Vofa_Rx_Reattach(void);
#endif
