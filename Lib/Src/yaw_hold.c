#include "yaw_hold.h"
#include "imu.h"

#include <math.h>

/* 若修正方向与打滑方向一致(越修越偏), 改成 -1.0f */
#define YAW_HOLD_SIGN      (1.0f)

#define YAW_KP_DEFAULT     (1.5f)
#define YAW_KI_DEFAULT     (0.6f)
#define YAW_KD_DEFAULT     (0.1f)
#define YAW_ERR_I_MAX      (0.60f)    /* 积分限幅 rad, 约±34度 */
#define YAW_RATE_DEADZONE  (0.00175f) /* 角速率死区 0.1度/s, 只作用于P/D项 */
#define YAW_OUT_MAX        (1.5f)     /* 修正量限幅 rad/s */
#define YAW_STOP_PULSES    (3.0f)     /* 两侧脉冲和低于此值视为真静止(约0.017m/s) */
#define YAW_W_TURN_EPS     (0.02f)    /* 主动转向判定阈值 rad/s */

#define CTRL_DT  (0.025f)             /* 与运动学控制周期一致 */
#define DEG2RAD  (0.01745329252f)

static uint8_t hold_enabled = 1;
static float Kp = YAW_KP_DEFAULT;
static float Ki = YAW_KI_DEFAULT;
static float Kd = YAW_KD_DEFAULT;
static float err_i = 0.0f;            /* 角速率误差积分, 等效累计角偏差 */
static float err_rate_prev = 0.0f;
static float last_err_deg = 0.0f;
static float last_out = 0.0f;         /* 最近一次返回的W修正量 */

void YawHold_Enable(uint8_t en) {
    hold_enabled = en;
    if (!en) { err_i = 0.0f; err_rate_prev = 0.0f; last_out = 0.0f; }
}

void YawHold_SetKp(float kp) { Kp = kp; }
void YawHold_SetKi(float ki) { Ki = ki; }
void YawHold_SetKd(float kd) { Kd = kd; }

float YawHold_GetError(void) {
    return last_err_deg;
}

float YawHold_GetYaw(void) {
    return IMU_GetYaw();
}

float YawHold_GetLastOut(void) {
    return last_out;
}

/**
 * @brief 每25ms控制节拍调用: 返回应叠加到W上的修正量
 * 直行时锁死角速率(目标=0): 积分连续累积不放泄, 收敛后保持一个
 * 恒定差速把车"钳"在直线上; 死区只作用在P/D项上, 避免死区边缘
 * 的修正-衰减-再修正周期循环(表现为毛刺尖峰和来回漂).
 */
float YawHold_Process(float V, float W, float act_pulses) {
    (void)V;   /* 命令速度不参与停止判定: 刹车滑行中命令已为0但车仍在动, 需继续钳制 */
    if (!hold_enabled || !IMU_RateOk()) {
        err_i = 0.0f;
        err_rate_prev = 0.0f;
        last_err_deg = 0.0f;
        last_out = 0.0f;
        return 0.0f;
    }

    float rate = IMU_GetYawRate();   /* rad/s, 已扣零偏 */

    /* 真正静止或主动转向: 不干预并清积分; 刹车滑行期间(轮子还在转)保持钳制,
       这样停车瞬间车头的左右摆也能被实时压住 */
    if (act_pulses < YAW_STOP_PULSES ||
        (W > YAW_W_TURN_EPS || W < -YAW_W_TURN_EPS)) {
        err_i = 0.0f;
        err_rate_prev = 0.0f;
        last_err_deg = 0.0f;
        last_out = 0.0f;
        return 0.0f;
    }

    /* 直行: 目标角速率=0, 误差=实际旋转速率的反号, 连续积分 */
    float err_rate = -rate;
    err_i += err_rate * CTRL_DT;
    if (err_i >  YAW_ERR_I_MAX) err_i =  YAW_ERR_I_MAX;
    if (err_i < -YAW_ERR_I_MAX) err_i = -YAW_ERR_I_MAX;

    float p = 0.0f;
    if (err_rate > YAW_RATE_DEADZONE || err_rate < -YAW_RATE_DEADZONE)
        p = Kp * err_rate;           /* 死区只压P, 防止陀螺噪声抖舵 */

    float out = YAW_HOLD_SIGN * (p + Ki * err_i + Kd * (err_rate - err_rate_prev));
    err_rate_prev = err_rate;

    if (out >  YAW_OUT_MAX) out =  YAW_OUT_MAX;
    if (out < -YAW_OUT_MAX) out = -YAW_OUT_MAX;

    /* 输出饱和时同向停止积分(防卷绕), 松开后自动恢复 */
    if (out >= YAW_OUT_MAX && err_rate > 0.0f)
        err_i -= err_rate * CTRL_DT;
    if (out <= -YAW_OUT_MAX && err_rate < 0.0f)
        err_i -= err_rate * CTRL_DT;

    last_err_deg = err_i / DEG2RAD;
    last_out = out;
    return out;
}