#include "protocol.h"
#include "kinematics.h"
#include "usart.h"
#include "stm32f1xx_hal.h"

#define RX_LEN 11U   /* 下行 0x7B + 2预留 + X(2) + Y(2) + Z(2) + BCC + 0x7D */
#define TX_LEN 37U   /* 上行, 布局见 protocol.h */

static uint8_t  rx[RX_LEN];
static uint8_t  rx_idx = 0;

static uint8_t bcc(const uint8_t *buf, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum ^= buf[i];
    }
    return sum;
}

/*
 * 逐字节喂入, 状态机:
 *   见到0x7B重新对齐帧头; 收满11字节后校验帧尾与BCC;
 *   合法帧立即切入自动模式并下发V/W。
 */
void Protocol_Push_Byte(uint8_t ch) {
    if (ch == 0x7B) {
        rx_idx = 0;
        rx[rx_idx++] = ch;
        return;
    }
    if (rx_idx == 0) {
        return;
    }
    if (rx_idx < RX_LEN) {
        rx[rx_idx++] = ch;
    }
    if (rx_idx == RX_LEN) {
        if (rx[0] == 0x7B && rx[10] == 0x7D && rx[9] == bcc(rx, 9)) {
            int16_t vx = (int16_t)((rx[3] << 8) | rx[4]);  /* mm/s */
            int16_t wz = (int16_t)((rx[7] << 8) | rx[8]);  /* rad/s * 1000 */

            float V = vx / 1000.0f;   /* mm/s -> m/s */
            float W = wz / 1000.0f;   /* mrad/s -> rad/s */

            Chassis_Set_Mode(CHASSIS_MODE_AUTO);
            Chassis_Set_Cmd(V, W, CHASSIS_SRC_AUTO);
        }
        rx_idx = 0;
    }
}

/* 帧间空闲判定: USART1 上收到 '#' 且此刻空闲时, 该字节可安全地当作文本命令起点 */
uint8_t Protocol_Rx_Idle(void)
{
    return (rx_idx == 0U) ? 1U : 0U;
}

int16_t Protocol_ClampI16(float v)
{
    /* IMU 异常/丢帧时数值可能离谱, 组帧前统一限幅, 防止 int16 溢出翻符号 */
    if (v >  32767.0f) return  32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)v;
}

void Protocol_Send_Feedback(const Protocol_Feedback_t *fb) {
    uint8_t frame[TX_LEN];

    frame[0]  = 0x7B;
    frame[1]  = fb->flag_stop;
    frame[2]  = (uint8_t)(fb->vx_mm_s >> 8);
    frame[3]  = (uint8_t)(fb->vx_mm_s);
    frame[4]  = (uint8_t)(fb->vy_mm_s >> 8);
    frame[5]  = (uint8_t)(fb->vy_mm_s);
    frame[6]  = (uint8_t)(fb->wz_mrad_s >> 8);
    frame[7]  = (uint8_t)(fb->wz_mrad_s);
    frame[8]  = (uint8_t)(fb->acc_x_raw >> 8);
    frame[9]  = (uint8_t)(fb->acc_x_raw);
    frame[10] = (uint8_t)(fb->acc_y_raw >> 8);
    frame[11] = (uint8_t)(fb->acc_y_raw);
    frame[12] = (uint8_t)(fb->acc_z_raw >> 8);
    frame[13] = (uint8_t)(fb->acc_z_raw);
    frame[14] = (uint8_t)(fb->gyro_x_raw >> 8);
    frame[15] = (uint8_t)(fb->gyro_x_raw);
    frame[16] = (uint8_t)(fb->gyro_y_raw >> 8);
    frame[17] = (uint8_t)(fb->gyro_y_raw);
    frame[18] = (uint8_t)(fb->gyro_z_raw >> 8);
    frame[19] = (uint8_t)(fb->gyro_z_raw);
    frame[20] = (uint8_t)(fb->battery_mv >> 8);
    frame[21] = (uint8_t)(fb->battery_mv);
    frame[22] = (uint8_t)(fb->yaw_001deg >> 8);
    frame[23] = (uint8_t)(fb->yaw_001deg);
    frame[24] = (uint8_t)(fb->yaw_rate_mrad_s >> 8);
    frame[25] = (uint8_t)(fb->yaw_rate_mrad_s);
    frame[26] = (uint8_t)(fb->yaw_hold_err_001deg >> 8);
    frame[27] = (uint8_t)(fb->yaw_hold_err_001deg);
    frame[28] = (uint8_t)(fb->yaw_hold_out_mrad_s >> 8);
    frame[29] = (uint8_t)(fb->yaw_hold_out_mrad_s);
    frame[30] = fb->status;
    frame[31] = (uint8_t)(fb->vl_mm_s >> 8);
    frame[32] = (uint8_t)(fb->vl_mm_s);
    frame[33] = (uint8_t)(fb->vr_mm_s >> 8);
    frame[34] = (uint8_t)(fb->vr_mm_s);
    frame[35] = bcc(frame, 35);
    frame[36] = 0x7D;

    HAL_UART_Transmit(&huart1, frame, sizeof(frame), 10);
}
