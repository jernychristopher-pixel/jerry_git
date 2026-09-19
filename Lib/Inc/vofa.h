#ifndef __VOFA_H
#define __VOFA_H

#include "stm32f1xx_hal.h"
#include "usart.h"
#include <string.h>

#define RX_BUF_SIZE 32

extern int gear;
extern float back_scale;

extern float joy1_X;
extern float joy1_Y;
extern float joy2_X;
extern float joy2_Y;

void Vofa_Start_Receive_IT(void);
void Ftoa(char *dst, float v, int dec);
void Vofa_Send(float ch0, float ch1, float ch2, float ch3, float ch4, float ch5, float ch6, float ch7);
void Vofa_Service(void);


uint16_t Vofa_GetRxErr(void);
void Vofa_Rx_Reattach(void);
#endif
