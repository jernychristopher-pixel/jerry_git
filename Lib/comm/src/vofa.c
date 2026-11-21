#include "vofa.h"
#include "protocol.h"
#include "pid.h"
#include "kinematics.h"
#include "usart.h"
#include "app_mode.h"

#include <stdio.h>
#include <string.h>

/* USART1: 树莓派二进制协议; USART3: VOFA 文本波形 + 无线烧录 reset 复位 */
static uint8_t uart1_rx_byte = 0;
static uint8_t uart3_rx_byte = 0;
static Protocol_Context g_proto;

static void Proto_Write(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 10);
}

static void Proto_OnVelocity(int16_t left_rpm, int16_t right_rpm)
{
    /* 协议约定轮速前进为正; 转速环内部极性相反, 只在这里翻一次 */
    Chassis_Set_Wheel_Rpm(CHASSIS_V_SIGN * (float)left_rpm,
                          CHASSIS_V_SIGN * (float)right_rpm);
}

static void Proto_OnPreset(uint8_t preset)
{
    PID_Apply_Preset(&SpeedPID_L, preset);
    PID_Apply_Preset(&SpeedPID_R, preset);
}

/* 线上单位 0.001, 顺序: 左 Kp/Ki/Kd, 右 Kp/Ki/Kd */
static void Proto_OnGains(const int16_t gains[6])
{
    SpeedPID_L.Kp = (float)gains[0] / 1000.0f;
    SpeedPID_L.Ki = (float)gains[1] / 1000.0f;
    SpeedPID_L.Kd = (float)gains[2] / 1000.0f;
    SpeedPID_R.Kp = (float)gains[3] / 1000.0f;
    SpeedPID_R.Ki = (float)gains[4] / 1000.0f;
    SpeedPID_R.Kd = (float)gains[5] / 1000.0f;
}

/* 开启两路串口接收中断（必须调用！） */
void Vofa_Start_Receive_IT(void)
{
    Protocol_Init(&g_proto, Proto_Write, Proto_OnVelocity,
                  Proto_OnPreset, Proto_OnGains);
    HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
    HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
}

/* 无线烧录: 收到上位机 "reset" 后软件复位, 回到 BootLoader */
static void _System_Reset_(uint8_t ch)
{
    static uint8_t res_buf[5];
    static uint8_t res_count = 0;

    res_buf[res_count] = ch;
    if (ch == 'r' || res_count > 0) {
        res_count++;
    } else {
        res_count = 0;
    }

    if (res_count == 5) {
        res_count = 0;
        if (res_buf[0] == 'r' && res_buf[1] == 'e' &&
            res_buf[2] == 's' && res_buf[3] == 'e' &&
            res_buf[4] == 't') {
            NVIC_SystemReset();   /* 复位后执行 BootLoader, 进入无线烧录 */
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        /*
         * 先过模式检测: M4-2 采集模式下 USART1 发的是 CSV 文本,
         * 这时只能靠 AA 55 帧头切回二进制协议。
         * 检测本身在 AppMode_FeedByte 里把 g_app_mode 改好,
         * 所以这里再读一次模式, 决定这个字节要不要进解析器。
         */
        AppMode_FeedByte(uart1_rx_byte);
        if (AppMode_Get() == APP_MODE_PROTOCOL) {
            Protocol_RxByte_ISR(&g_proto, uart1_rx_byte);
        }
        HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
    } else if (huart == &huart3) {
        /* USART3 只保留无线烧录的 reset 检测 */
        _System_Reset_(uart3_rx_byte);
        HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
    }
}

void Vofa_Protocol_Poll(void)
{
    Protocol_Poll(&g_proto, HAL_GetTick());
}

void Vofa_Reset_Parser(void)
{
    Protocol_Reset_Parser(&g_proto);
}

void Vofa_Protocol_SendStatus(int16_t left_rpm, int16_t right_rpm,
                              int16_t pitch_001deg, int16_t yaw_001deg,
                              uint16_t battery_mv,
                              int16_t yaw_rate_mrad_s,
                              int16_t yaw_hold_out_mrad_s,
                              uint8_t status_bits)
{
    static uint32_t next_tx = 0;
    uint32_t now = HAL_GetTick();

    if ((int32_t)(now - next_tx) >= 0) {
        Protocol_Status st;
        st.left_rpm = left_rpm;
        st.right_rpm = right_rpm;
        st.pitch_001deg = pitch_001deg;
        st.yaw_001deg = yaw_001deg;
        st.battery_mv = battery_mv;
        st.yaw_rate_mrad_s = yaw_rate_mrad_s;
        st.yaw_hold_out_mrad_s = yaw_hold_out_mrad_s;
        st.status_bits = status_bits;
        Protocol_SendStatus(&g_proto, &st);

        next_tx += 50u;
        /* 主循环 20ms 一拍, 如果已经落后超过一整拍, 不补发, 重新对齐 */
        if ((int32_t)(now - next_tx) >= 0) {
            next_tx = now + 50u;
        }
    }
}

void Vofa_Protocol_SendTelemetry(int16_t target_rpm_L, int16_t actual_rpm_L,
                                 int16_t pwm_L,
                                 int16_t target_rpm_R, int16_t actual_rpm_R,
                                 int16_t pwm_R)
{
    Protocol_Telemetry tm;
    tm.target_rpm_L = target_rpm_L;
    tm.actual_rpm_L = actual_rpm_L;
    tm.pwm_L = pwm_L;
    tm.target_rpm_R = target_rpm_R;
    tm.actual_rpm_R = actual_rpm_R;
    tm.pwm_R = pwm_R;
    Protocol_SendTelemetry(&g_proto, &tm);
}

/*
 * HAL 的坑: 串口一旦出现 ORE/FE/NE, HAL 会执行 UART_EndRxTransfer() 关掉接收中断,
 * 然后等 HAL_UART_ErrorCallback(). 默认是空实现 —— 没人重新武装接收, RX 就永久静默
 * (TX 照发, 上位机看着一切正常, 车却再也不听指令). 对 SLAM 链路是致命的, 必须兜底.
 */
static volatile uint16_t rx_err_cnt = 0U;

uint16_t Vofa_GetRxErr(void)
{
    return rx_err_cnt;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    rx_err_cnt++;
    if (huart == &huart1) {
        /* 波特率错/线上干扰时 HAL 会报 FE/NE/ORE, 计入坏帧 */
        Protocol_OnUartError(&g_proto,
                             (huart->ErrorCode & HAL_UART_ERROR_ORE) ? 1u : 0u);
    }
    __HAL_UART_CLEAR_PEFLAG(huart);
    if (huart->Instance == USART1) {
        HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
    } else if (huart->Instance == USART3) {
        HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
    }
}

/*
 * 主循环兜底: 万一 HAL 内部状态卡住(重新武装返回 BUSY), 直接看硬件寄存器 ——
 * RXNEIE 被关掉就说明没人在收数据了, 强制复位状态再挂一次.
 */
void Vofa_Rx_Reattach(void)
{
    if ((huart1.Instance->CR1 & USART_CR1_RXNEIE) == 0U) {
        huart1.RxState = HAL_UART_STATE_READY;
        HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
    }
    if ((huart3.Instance->CR1 & USART_CR1_RXNEIE) == 0U) {
        huart3.RxState = HAL_UART_STATE_READY;
        HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
    }
}

static void ip_to_str(char *dst, long v)
{
    char buf[12];
    int n = 0;
    if (v == 0) { dst[0] = '0'; dst[1] = '\0'; return; }
    while (v > 0) { buf[n++] = (char)('0' + (v % 10)); v /= 10; }
    for (int i = 0; i < n; i++) dst[i] = buf[n - 1 - i];
    dst[n] = '\0';
}

void Ftoa(char *dst, float v, int dec)
{
    int pos = 0;
    if (v < 0.0f) { dst[pos++] = '-'; v = -v; }
    float scale = 1.0f;
    for (int i = 0; i < dec; i++) scale *= 10.0f;
    long scaled = (long)(v * scale + 0.5f);
    long scale_l = (long)scale;
    ip_to_str(dst + pos, scaled / scale_l);
    pos += (int)strlen(dst + pos);
    if (dec > 0) {
        dst[pos++] = '.';
        long div = scale_l / 10;
        for (int i = 0; i < dec; i++) {
            dst[pos++] = (char)('0' + ((scaled / div) % 10));
            div /= 10;
        }
    }
    dst[pos] = '\0';
}

// ---- FireWater 文本帧发送: ch0-3左右轮目标/实际脉冲, ch4偏航角, ch5航向误差 ----
void Vofa_Send(float ch0, float ch1, float ch2, float ch3,
               float ch4, float ch5, float ch6, float ch7)
{
    char a[16], b[16], c[16], d[16], e[16], f[16], g[16], h[16];
    char line[128];
    Ftoa(a, ch0, 3);
    Ftoa(b, ch1, 3);
    Ftoa(c, ch2, 3);
    Ftoa(d, ch3, 3);
    Ftoa(e, ch4, 2);
    Ftoa(f, ch5, 2);
    Ftoa(g, ch6, 2);
    Ftoa(h, ch7, 3);
    int n = snprintf(line, sizeof(line), "%s,%s,%s,%s,%s,%s,%s,%s\n",
                     a, b, c, d, e, f, g, h);
    if (n > 0 && n < (int)sizeof(line)) {
        HAL_UART_Transmit(&huart3, (uint8_t *)line, (uint16_t)n, 10);
    }
}
