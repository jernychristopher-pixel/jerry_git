#include "vofa.h"
#include "protocol.h"
#include "yaw_hold.h"


#include "pid.h"
#include "kinematics.h"
#include <stdio.h>

#if APP_USE_M44_PROTOCOL
#include "m44_protocol.h"
#endif

/* USART1: 树莓派二进制协议; USART3: VOFA文本 + 无线烧录reset复位 */
static uint8_t uart1_rx_byte = 0;
static uint8_t uart3_rx_byte = 0;
static uint8_t text_buf[RX_BUF_SIZE] = {0};
static uint8_t text_idx = 0;
/* 当前文本命令来自哪个串口(供 #Q 回读), 以及回读请求标志 */
static UART_HandleTypeDef *text_src = NULL;
static UART_HandleTypeDef *cfg_dump_port = NULL;
static volatile uint8_t cfg_dump_req = 0;
static volatile uint8_t csv_mode = 0;

#if APP_USE_M44_PROTOCOL
static M44_Context g_m44;

static void M44_Write(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 10);
}

static void M44_OnVelocity(int16_t left_rpm, int16_t right_rpm)
{
    Chassis_Set_Wheel_Rpm((float)left_rpm, (float)right_rpm);
}
#endif

/* 开启两路串口接收中断（必须调用！） */
void Vofa_Start_Receive_IT(void)
{
#if APP_USE_M44_PROTOCOL
    M44_Init(&g_m44, M44_Write, M44_OnVelocity, NULL);
#endif
    HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
    HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
}

static float parse_float(const char *s)
{
    float sign = 1.0f, val = 0.0f, frac = 0.0f, div = 1.0f;
    while (*s == ' ' || *s == '\t') s++;          /* 允许 #A 1.0 这类带空格写法 */
    if (*s == '-') { sign = -1.0f; s++; }
    else if (*s == '+') s++;
    while (*s >= '0' && *s <= '9') { val = val * 10.0f + (float)(*s - '0'); s++; }
    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9') { frac = frac * 10.0f + (float)(*s - '0'); div *= 10.0f; s++; }
    }
    return sign * (val + frac / div);
}

static long parse_int(const char *s)
{
    long sign = 1, val = 0;
    while (*s == ' ' || *s == '\t') s++;
    if (*s == '-') { sign = -1; s++; }
    else if (*s == '+') s++;
    while (*s >= '0' && *s <= '9') { val = val * 10 + (long)(*s - '0'); s++; }
    return sign * val;
}

static void Parse_UART_Frame(char *buf)
{
    if (buf[0] != '#') return; // 无效帧

    char cmd = buf[1];
    float val = parse_float(buf + 2);

    switch (cmd) {
        case 'A': SpeedPID_L.Kp = val; break;
        case 'B': SpeedPID_L.Ki = val; break;
        case 'C': SpeedPID_L.Kd = val; break;
        case 'D': SpeedPID_R.Kp = val; break;
        case 'E': SpeedPID_R.Ki = val; break;
        case 'F': SpeedPID_R.Kd = val; break;
        case 'I': YawHold_SetKp(val); break;
        case 'L': YawHold_SetKi(val); break;
        case 'M': YawHold_SetKd(val); break;
        case 'N': { long n = parse_int(buf + 2); YawHold_Enable(n != 0); break; }
        /* M4-3: #S 0/1/2 切 P/PI/PID; #T <rpm> 直接给左右轮同速目标;
         * #X 退出轮速整定模式, 回到上位机的 V/W 控制。 */
        case 'S': {
            long preset = parse_int(buf + 2);
            if (preset >= 0 && preset <= 2) {
                PID_Apply_Preset(&SpeedPID_L, (unsigned)preset);
                PID_Apply_Preset(&SpeedPID_R, (unsigned)preset);
            }
            break;
        }
        case 'T': {
            float rpm = parse_float(buf + 2);
            Chassis_Set_Wheel_Rpm(rpm, rpm);
            break;
        }
        case 'X': Chassis_Exit_Wheel_Rpm(); break;
        /* #V 1 开启 CSV 文本输出(暂停 USART1 二进制帧), #V 0 关闭 */
        case 'V': { long v = parse_int(buf + 2); csv_mode = (v != 0); break; }
        case 'R': PID_Init(&SpeedPID_L); PID_Init(&SpeedPID_R); break;
        case 'Q': cfg_dump_req = 1; cfg_dump_port = text_src; break;
        default: break;
    }
}

/* 无线烧录: 收到上位机 "reset" 后软件复位, 回到 BootLoader */
static void _System_Reset_(uint8_t ch) {
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

/* USART3 字节入口: 先喂复位检测, 再按 # 文本帧解析 */
static void Vofa_Push_Byte(uint8_t ch) {
    _System_Reset_(ch);

    if (ch == '\n') {
        text_buf[text_idx] = '\0';
        text_src = &huart3;
        Parse_UART_Frame((char*)text_buf);
        text_idx = 0;
        memset(text_buf, 0, RX_BUF_SIZE);
    } else if (ch == '#') {
        text_idx = 0;
        text_buf[text_idx] = ch;
        text_idx++;
    } else {
        if (text_idx < RX_BUF_SIZE - 1) {
            text_buf[text_idx] = ch;
            text_idx++;
        }
    }
}

#if !APP_USE_M44_PROTOCOL
/*
 * USART1 文本命令入口 (树莓派 Type-C)。
 * 帧间空闲且首字节为井号时, 按 VOFA 调参命令解析; 其余字节原样交给二进制控制帧。
 * 不接蓝牙也不接 USB-TTL 时, 树莓派往串口写命令即可在线改参数。
 * 二进制解析器在空闲状态下本来就忽略非 0x7B 字节, 所以借用井号不会打乱控制帧。
 */
static uint8_t t1_buf[RX_BUF_SIZE];
static uint8_t t1_idx = 0;
static uint8_t t1_active = 0;

static void Vofa_Push_Byte_USART1(uint8_t ch)
{
    if (!t1_active) {
        if (ch == 0x23 && Protocol_Rx_Idle()) {
            t1_active = 1;
            t1_idx = 0;
            t1_buf[t1_idx++] = ch;
        } else {
            Protocol_Push_Byte(ch);
        }
        return;
    }

    if (ch == 0x0A) {
        t1_buf[t1_idx] = 0;
        text_src = &huart1;
        Parse_UART_Frame((char*)t1_buf);
        t1_active = 0;
        t1_idx = 0;
    } else if (t1_idx < RX_BUF_SIZE - 1) {
        t1_buf[t1_idx++] = ch;
    } else {
        t1_active = 0;
        t1_idx = 0;
    }
}
#endif /* !APP_USE_M44_PROTOCOL */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
#if APP_USE_M44_PROTOCOL
        /* M4-4: USART1 只跑 0xAA55 + CRC16 协议 */
        M44_RxByte_ISR(&g_m44, uart1_rx_byte);
#else
        /* USART1: 树莓派 Wheeltec 二进制控制帧, 兼收 '#' 在线调参命令 */
        Vofa_Push_Byte_USART1(uart1_rx_byte);
#endif
        HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
    } else if (huart == &huart3) {
        /* USART3: VOFA 文本调参 + 无线烧录 reset 复位 */
        Vofa_Push_Byte(uart3_rx_byte);
        HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
    }
}

#if APP_USE_M44_PROTOCOL
void Vofa_M44_Poll(void)
{
    M44_Poll(&g_m44, HAL_GetTick());
}

void Vofa_M44_Service(int16_t left_rpm, int16_t right_rpm,
                      int16_t pitch_001deg, int16_t yaw_001deg,
                      uint8_t status_bits)
{
    static uint32_t next_tx = 0;
    uint32_t now = HAL_GetTick();

    if ((int32_t)(now - next_tx) >= 0) {
        M44_SendStatus(&g_m44, left_rpm, right_rpm,
                       pitch_001deg, yaw_001deg, status_bits);
        next_tx += 50u;
        /* 主循环 20ms 一拍, 如果已经落后超过一整拍, 不补发, 重新对齐 */
        if ((int32_t)(now - next_tx) >= 0) {
            next_tx = now + 50u;
        }
    }
}
#endif

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
#if APP_USE_M44_PROTOCOL
    if (huart == &huart1) {
        /* 波特率错/线上干扰时 HAL 会报 FE/NE/ORE, 计入 M4-4 坏帧 */
        M44_OnUartError(&g_m44,
                        (huart->ErrorCode & HAL_UART_ERROR_ORE) ? 1u : 0u);
    }
#endif
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
/* #Q 回读: 由主循环发出, 不占中断, 用来确认链路通断和参数现状 */
void Vofa_Service(void)
{
    if (!cfg_dump_req) return;
    cfg_dump_req = 0;

    char lkp[16], lki[16], lkd[16], rkp[16], rki[16], rkd[16];
    Ftoa(lkp, SpeedPID_L.Kp, 2);
    Ftoa(lki, SpeedPID_L.Ki, 2);
    Ftoa(lkd, SpeedPID_L.Kd, 2);
    Ftoa(rkp, SpeedPID_R.Kp, 2);
    Ftoa(rki, SpeedPID_R.Ki, 2);
    Ftoa(rkd, SpeedPID_R.Kd, 2);

    char line[128];
    int n = snprintf(line, sizeof(line),
                     "CFG Lkp=%s Lki=%s Lkd=%s Rkp=%s Rki=%s Rkd=%s",
                     lkp, lki, lkd, rkp, rki, rkd);
    if (n <= 0 || n >= (int)sizeof(line)) return;

    UART_HandleTypeDef *port = (cfg_dump_port != NULL) ? cfg_dump_port : &huart3;
    HAL_UART_Transmit(port, (uint8_t*)line, (uint16_t)n, 50);
    uint8_t nl = 10;
    HAL_UART_Transmit(port, &nl, 1, 50);
}

// ---- FireWater 文本帧发送: ch0-3左右轮目标/实际脉冲, ch4偏航角, ch5航向误差 ----
void Vofa_Send(float ch0, float ch1, float ch2, float ch3, float ch4, float ch5, float ch6, float ch7) {
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
    int n = snprintf(line, sizeof(line), "%s,%s,%s,%s,%s,%s,%s,%s\n", a, b, c, d, e, f, g, h);
    if (n > 0 && n < (int)sizeof(line)) {
        HAL_UART_Transmit(&huart3, (uint8_t*)line, (uint16_t)n, 10);
    }
}

uint8_t Vofa_Csv_Enabled(void) {
    return csv_mode;
}

/* 原始 CSV: 一行 7 列, 前四列与题目要求的 t,target,actual,pwm 对应(左轮) */
void Vofa_Send_Csv(float t, float target_l, float actual_l, float pwm_l,
                   float target_r, float actual_r, float pwm_r) {
    char a[16], b[16], c[16], d[16], e[16], f[16], g[16];
    char line[128];
    Ftoa(a, t, 3);
    Ftoa(b, target_l, 2);
    Ftoa(c, actual_l, 2);
    Ftoa(d, pwm_l, 1);
    Ftoa(e, target_r, 2);
    Ftoa(f, actual_r, 2);
    Ftoa(g, pwm_r, 1);
    int n = snprintf(line, sizeof(line), "%s,%s,%s,%s,%s,%s,%s\n", a, b, c, d, e, f, g);
    if (n > 0 && n < (int)sizeof(line)) {
        HAL_UART_Transmit(&huart1, (uint8_t*)line, (uint16_t)n, 20);
    }
}
