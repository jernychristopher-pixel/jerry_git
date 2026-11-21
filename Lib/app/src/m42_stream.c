#include "m42_stream.h"

#include <math.h>
#include <string.h>

#include "kinematics.h"
#include "mpu6050.h"
#include "usart.h"
#include "stm32f1xx_hal.h"
#include "app_mode.h"

#define M42_DT_NOMINAL_S       0.010f
#define M42_TAU_S              0.50f   /* 互补滤波时间常数 */
#define M42_ACCEL_TAU_S        0.03f   /* 加速度计一阶低通 */
#define M42_BIAS_SAMPLES       200U    /* 2 s @ 100 Hz */
#define M42_BIAS_PERIOD_MS     10U
#define M42_ACCEL_LSB_PER_G    16384.0f
#define M42_ENCODER_CPR        1456.0f
#define M42_RAD_TO_DEG         57.29577951308232f
#define M42_TX_CAP             2048U
#define M42_WINDOW_STEPS       100U    /* 100 x 10 ms = 1 s 测速窗口 */

/*
 * WHEELTEC C10B 贴装方向: DMP 方向矩阵为 diag(-1, -1, +1),
 * 即芯片 X/Y 轴与车身 X/Y 反向。这里统一换算到题目坐标系:
 *   X 车头前方, Y 车身右侧, Z 垂直向上。
 */
#define M42_AX_SIGN            (-1.0f)
#define M42_AY_SIGN            (-1.0f)
#define M42_AZ_SIGN            ( 1.0f)
/*
 * 题目约定 pitch 抬头为正: pitch_acc = atan2(-ax, az)。
 * 右手系绕 +Y 的正旋转让车头向下, 所以积分项用 -gyro_y。
 * gyro_y_board = -gyro_y_raw, 两次取负抵消, 实际用 +raw gyro_y。
 */
#define M42_GYRO_PITCH_SIGN    (+1.0f)

static float    g_gyro_sens = 65.5f;
static float    g_bias_dps;
static uint8_t  g_filter_ready;
static float    g_acc_lpf[3];
static float    g_pitch_acc;
static float    g_pitch_fused;
static float    g_pitch_gyro;
static uint32_t g_t0_ms;
static uint32_t g_last_ms;

static int32_t  g_count_l;
static int32_t  g_count_r;
static float    g_rpm_l;
static float    g_rpm_r;
static int32_t  g_win_l;
static int32_t  g_win_r;
static uint16_t g_win_n;

static volatile uint32_t g_i2c_err;
static volatile uint32_t g_drop_lines;

static uint8_t  g_tx_buf[M42_TX_CAP];
/* head 在主循环写、中断读, tail/inflight 反过来, 必须 volatile,
 * 否则开优化后编译器会把它们缓存在寄存器里, 队列就错位了。 */
static volatile uint16_t g_tx_head;
static volatile uint16_t g_tx_tail;
static volatile uint16_t g_tx_inflight;
static volatile uint8_t g_tx_busy;

/* ---- 无浮点 printf 的定点格式化 ---- */

static char *put_u32(char *p, uint32_t value)
{
    char tmp[10];
    uint8_t n = 0;
    do {
        tmp[n++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);
    while (n > 0U) {
        *p++ = tmp[--n];
    }
    return p;
}

static char *put_i32(char *p, int32_t value)
{
    if (value < 0) {
        *p++ = '-';
        return put_u32(p, (uint32_t)(-(value + 1)) + 1U);
    }
    return put_u32(p, (uint32_t)value);
}

/* decimals: 1 -> 0.0, 2 -> 0.00, 3 -> 0.000 */
static char *put_fixed(char *p, float value, uint8_t decimals)
{
    int32_t scale = (decimals == 1U) ? 10 : (decimals == 2U) ? 100 : 1000;
    float scaled = value * (float)scale;
    int32_t q = (int32_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
    int32_t whole;
    int32_t frac;
    int32_t div;

    if (q < 0) {
        *p++ = '-';
        q = -q;
    }
    whole = q / scale;
    frac = q % scale;
    p = put_u32(p, (uint32_t)whole);
    *p++ = '.';
    for (div = scale / 10; div > 0; div /= 10) {
        *p++ = (char)('0' + (frac / div) % 10);
    }
    return p;
}

/* ---- 串口发送队列: 只从尾部追加, 发送完成后再回收 ---- */

static void tx_compact(void)
{
    if (g_tx_busy || g_tx_tail == 0U) {
        return;
    }
    if (g_tx_head > g_tx_tail) {
        uint16_t n = (uint16_t)(g_tx_head - g_tx_tail);
        memmove(g_tx_buf, &g_tx_buf[g_tx_tail], n);
        g_tx_head = n;
        g_tx_tail = 0;
    } else {
        g_tx_head = 0;
        g_tx_tail = 0;
    }
}

static void tx_kick(void)
{
    uint16_t n;

    if (g_tx_busy) {
        return;
    }
    n = (uint16_t)(g_tx_head - g_tx_tail);
    if (n == 0U) {
        return;
    }
    g_tx_inflight = n;
    g_tx_busy = 1;
    if (HAL_UART_Transmit_IT(&huart1, &g_tx_buf[g_tx_tail], n) != HAL_OK) {
        g_tx_busy = 0;
        g_tx_inflight = 0;
    }
}

static void tx_push(const char *line, uint16_t len)
{
    if ((uint32_t)g_tx_head + len > M42_TX_CAP) {
        tx_compact();
    }
    if ((uint32_t)g_tx_head + len > M42_TX_CAP) {
        g_drop_lines++;   /* 链路长期发不出去时丢整行, 不破坏正在发送的数据 */
        return;
    }
    memcpy(&g_tx_buf[g_tx_head], line, len);
    g_tx_head = (uint16_t)(g_tx_head + len);
    tx_kick();
}

void M42_TxComplete(void)
{
    g_tx_tail = (uint16_t)(g_tx_tail + g_tx_inflight);
    g_tx_inflight = 0;
    g_tx_busy = 0;
    if (g_tx_tail == g_tx_head) {
        g_tx_head = 0;
        g_tx_tail = 0;
    }
    tx_kick();
}

/*
 * 切回协议模式时调用: 采集流必须在下一个字节之前彻底停住,
 * 否则残留的 CSV 文本会插进二进制帧里, 主机侧看到的就是坏帧。
 *
 * 正在发送的那一段只能靠 HAL 中止, 光清指针会让 TxComplete 里的
 * tail += inflight 把 tail 推到 head 前面, 队列就再也对不上了。
 */
void M42_Stop(void)
{
    if (g_tx_busy) {
        HAL_UART_AbortTransmit(&huart1);
        g_tx_busy     = 0;
        g_tx_inflight = 0;
    }
    g_tx_head = 0;
    g_tx_tail = 0;
}

/* HAL 的发送完成回调只服务 M4-2 文本流 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        /* 已经切到协议模式时队列已被 M42_Stop 清空, 不再续发 */
        if (g_app_mode == APP_MODE_M42) {
            M42_TxComplete();
        }
    }
}

static void emit_bias_line(float bias_dps)
{
    char line[40];
    char *p = line;

    memcpy(p, "# gyro_y_bias_dps=", 18);
    p += 18;
    p = put_fixed(p, bias_dps, 3);
    *p++ = '\r';
    *p++ = '\n';
    tx_push(line, (uint16_t)(p - line));
}

void M42_Init(void)
{
    int16_t raw[7];
    float sum = 0.0f;
    uint16_t n = 0;
    uint32_t deadline;

    g_gyro_sens = MPU6050_GetGyroSens();
    g_t0_ms = HAL_GetTick();
    g_last_ms = g_t0_ms;

    /* 上电 2 s 静止窗口: 估计 gyro_y 残余零偏 (硬件偏移校准之后的残差) */
    deadline = HAL_GetTick() + M42_BIAS_SAMPLES * M42_BIAS_PERIOD_MS;
    while ((int32_t)(HAL_GetTick() - deadline) < 0) {
        /*
         * 主站如果在这 2 s 内发了 AA 55 帧头, 说明它要跑 M4-3/M4-4,
         * 中断里已经把模式改掉了。这时候立刻放弃标定, 别把对面上位机
         * 卡在 2 s 的等零偏里。
         */
        if (g_app_mode != APP_MODE_M42) {
            return;
        }
        if (MPU6050_ReadRaw14(raw)) {
            sum += (float)raw[5] / g_gyro_sens;
            n++;
        }
        HAL_Delay(M42_BIAS_PERIOD_MS);
    }
    g_bias_dps = (n > 0U) ? (sum / (float)n) : 0.0f;
    emit_bias_line(g_bias_dps);

    /* 用第一帧真实姿态初始化滤波, 避免从 0 度慢慢爬 */
    if (MPU6050_ReadRaw14(raw)) {
        g_acc_lpf[0] = M42_AX_SIGN * (float)raw[0] / M42_ACCEL_LSB_PER_G;
        g_acc_lpf[1] = M42_AY_SIGN * (float)raw[1] / M42_ACCEL_LSB_PER_G;
        g_acc_lpf[2] = M42_AZ_SIGN * (float)raw[2] / M42_ACCEL_LSB_PER_G;
        g_pitch_acc = atan2f(-g_acc_lpf[0], g_acc_lpf[2]) * M42_RAD_TO_DEG;
        g_pitch_fused = g_pitch_acc;
        g_pitch_gyro = g_pitch_acc;
        g_filter_ready = 1;
    }
    g_last_ms = HAL_GetTick();
}

void M42_Step(void)
{
    int16_t raw[7];
    int32_t d_l = 0;
    int32_t d_r = 0;
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - g_last_ms) / 1000.0f;
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float pitch_acc;
    float pitch_rate;
    float step_deg;
    float alpha;
    float beta;
    uint32_t t_ms;
    char line[192];
    char *p = line;

    if (dt < 0.001f || dt > 0.050f) {
        dt = M42_DT_NOMINAL_S;
    }
    g_last_ms = now;

    if (!MPU6050_ReadRaw14(raw)) {
        g_i2c_err++;
        return;
    }

    ax = M42_AX_SIGN * (float)raw[0] / M42_ACCEL_LSB_PER_G;
    ay = M42_AY_SIGN * (float)raw[1] / M42_ACCEL_LSB_PER_G;
    az = M42_AZ_SIGN * (float)raw[2] / M42_ACCEL_LSB_PER_G;
    gx = M42_AX_SIGN * (float)raw[4] / g_gyro_sens;
    gy = M42_AY_SIGN * (float)raw[5] / g_gyro_sens;
    gz = M42_AZ_SIGN * (float)raw[6] / g_gyro_sens;

    alpha = dt / (M42_ACCEL_TAU_S + dt);
    g_acc_lpf[0] += alpha * (ax - g_acc_lpf[0]);
    g_acc_lpf[1] += alpha * (ay - g_acc_lpf[1]);
    g_acc_lpf[2] += alpha * (az - g_acc_lpf[2]);
    pitch_acc = atan2f(-g_acc_lpf[0], g_acc_lpf[2]) * M42_RAD_TO_DEG;
    g_pitch_acc = pitch_acc;

    pitch_rate = M42_GYRO_PITCH_SIGN * (gy - g_bias_dps);
    step_deg = pitch_rate * dt;

    if (!g_filter_ready) {
        g_pitch_fused = pitch_acc;
        g_pitch_gyro = pitch_acc;
        g_filter_ready = 1;
    }
    g_pitch_gyro += step_deg;
    beta = M42_TAU_S / (M42_TAU_S + dt);
    g_pitch_fused = beta * (g_pitch_fused + step_deg)
                  + (1.0f - beta) * pitch_acc;

    /* 1 s 窗口测速: 控制环累计值 -> 窗口和 -> 脉冲/圈 -> r/min */
    Chassis_Read_Encoder_Total(&d_l, &d_r, 1);
    g_win_l += d_l;
    g_win_r += d_r;
    g_win_n++;
    if (g_win_n >= M42_WINDOW_STEPS) {
        g_count_l = g_win_l;
        g_count_r = g_win_r;
        g_rpm_l = (float)g_count_l * 60.0f / M42_ENCODER_CPR;
        g_rpm_r = (float)g_count_r * 60.0f / M42_ENCODER_CPR;
        g_win_l = 0;
        g_win_r = 0;
        g_win_n = 0;
    }

    t_ms = now - g_t0_ms;
    p = put_u32(p, t_ms / 1000U);
    *p++ = '.';
    *p++ = (char)('0' + (t_ms % 1000U) / 100U);
    *p++ = (char)('0' + ((t_ms % 1000U) / 10U) % 10U);
    *p++ = ',';
    p = put_i32(p, g_count_l);
    *p++ = ',';
    p = put_i32(p, g_count_r);
    *p++ = ',';
    p = put_fixed(p, g_rpm_l, 1);
    *p++ = ',';
    p = put_fixed(p, g_rpm_r, 1);
    *p++ = ',';
    p = put_fixed(p, ax, 3);
    *p++ = ',';
    p = put_fixed(p, ay, 3);
    *p++ = ',';
    p = put_fixed(p, az, 3);
    *p++ = ',';
    p = put_fixed(p, gx, 1);
    *p++ = ',';
    p = put_fixed(p, gy, 1);
    *p++ = ',';
    p = put_fixed(p, gz, 1);
    *p++ = ',';
    p = put_fixed(p, g_pitch_fused, 2);
    *p++ = ',';
    p = put_fixed(p, g_pitch_gyro, 2);
    *p++ = '\r';
    *p++ = '\n';
    tx_push(line, (uint16_t)(p - line));
}

uint32_t M42_GetI2cErrorCount(void)
{
    return g_i2c_err;
}

uint32_t M42_GetDropLineCount(void)
{
    return g_drop_lines;
}
