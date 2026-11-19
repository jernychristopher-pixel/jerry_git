#include "kinematics.h"
#include "yaw_hold.h"
#include "pid.h"
#include "motor.h"
#include "tim.h"
#include <math.h>

// ===== 物理常量已上移到 kinematics.h, 全工程唯一来源, 这里不再重复定义 =====
#define CTRL_PERIOD      0.025f   /* TIM1 控制周期 25ms, 与 CubeMX 配置保持一致 */
#define EMA_ALPHA        0.4f     /* 显示用低通系数, 越大越跟手 */

/* 自动模式断线保护: 超过 200ms(8个25ms节拍)未收到新指令 → 目标归零 */
#define AUTO_TIMEOUT_TICKS  8U
/* 目标死区: 太小的速度直接当 0, 抑制静止时的爬行和嗡鸣 */
#define DEADBAND_V   0.02f   /* m/s */
#define DEADBAND_W   0.04f   /* rad/s */

// 运动学状态全局变量
Robot_State_t robot_state;

/* 命令邮箱: 主循环写, TIM1 25ms中断读; 写入时关中断保证成对更新 */
static volatile float    cmd_V = 0.0f;
static volatile float    cmd_W = 0.0f;
static volatile uint8_t  chassis_mode = CHASSIS_MODE_MANUAL;
static volatile uint32_t auto_ticks = 0;

void Chassis_Set_Mode(Chassis_Mode_t mode) {
    chassis_mode = (uint8_t)mode;
}

Chassis_Mode_t Chassis_Get_Mode(void) {
    return (Chassis_Mode_t)chassis_mode;
}

/**
 * @brief 统一速度命令入口: 手柄与SLAM(串口)都必须走这里下发 V/W
 *        命令来源与当前模式不匹配时直接拒绝, 防止双方抢写目标速度
 */
void Chassis_Set_Cmd(float V, float W, Chassis_Src_t src) {
    if (src == CHASSIS_SRC_AUTO) {
        if (chassis_mode != CHASSIS_MODE_AUTO) return;
        auto_ticks = 0;          /* 喂狗: 自动命令仍然有效 */
    } else {
        if (chassis_mode != CHASSIS_MODE_MANUAL) return;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    cmd_V = CHASSIS_V_SIGN * V;   /* 对外坐标系 -> 内部电机坐标系 */
    cmd_W = CHASSIS_W_SIGN * W;
    if (primask == 0) __enable_irq();
}

/**
 * @brief 运动学逆解：将线速度V(m/s)和角速度W(rad/s)转换为目标轮速脉冲
 */
void Inverse_Kinematics(float V, float W) {
    // 1. 算出左右轮的目标线速度 (m/s)
    float v_left  = V - (W * WHEEL_TRACK / 2.0f);
    float v_right = V + (W * WHEEL_TRACK / 2.0f);

    // 2. 将 m/s 转换为 25ms周期内的目标脉冲数
    robot_state.target_pulse_L = (v_left  * CTRL_PERIOD / WHEEL_PERIMETER) * ENCODER_CPR;
    robot_state.target_pulse_R = (v_right * CTRL_PERIOD / WHEEL_PERIMETER) * ENCODER_CPR;
}

/* 单个轮速脉冲数 -> 轮速 mm/s */
static float pulse_to_mm_s(float pulse) {
    return pulse * WHEEL_PERIMETER / ENCODER_CPR / CTRL_PERIOD * 1000.0f;
}

/* 差速正解: 线速度 = (左+右)/2, 单位 mm/s */
float Chassis_GetVx_MmS(void) {
    float vl = pulse_to_mm_s(robot_state.actual_pulse_L);
    float vr = pulse_to_mm_s(robot_state.actual_pulse_R);
    return CHASSIS_V_SIGN * (vl + vr) * 0.5f;      /* 内部 -> 对外 */
}

/* 差速正解: 角速度 = (右-左)/轮距, 返回值 rad/s * 1000 */
float Chassis_GetWz_MradS(void) {
    float vl = pulse_to_mm_s(robot_state.actual_pulse_L);
    float vr = pulse_to_mm_s(robot_state.actual_pulse_R);
    return CHASSIS_W_SIGN * (vr - vl) / WHEEL_TRACK;   /* mm/s / m, 数值即 mrad/s */
}

/* 左右轮实测线速度, mm/s, 前进为正。
 * 与 Chassis_GetVx_MmS / Chassis_GetWz_MradS 同源 (actual_pulse + CHASSIS_V_SIGN),
 * 保证恒有 vx = (vl + vr) / 2 —— 上位机据此显示左右轮速, 不必再自带轮距常数。 */
void Chassis_GetWheels_MmS(float *vl_mm_s, float *vr_mm_s) {
    *vl_mm_s = CHASSIS_V_SIGN * pulse_to_mm_s(robot_state.actual_pulse_L);
    *vr_mm_s = CHASSIS_V_SIGN * pulse_to_mm_s(robot_state.actual_pulse_R);
}

/**
 * @brief 速度环主体: 目标速度 → 运动学逆解 → 位置式 PID → PWM
 *
 * 设计原则: 速度控制只保留一个位置式 PID。
 *   - 不做加减速斜坡: 目标怎么变, PID 就怎么跟, 参数含义单一;
 *   - 不写单独的刹车状态机: 目标为 0 时误差 = -实际速度,
 *     PID 输出自然反向, 制动由 PID 自己完成;
 *   - 手动/自动的区分只发生在命令入口, 到这里已经统一成 V/W。
 *
 * 调用位置: TIM1 更新中断, 每 25ms 一次。
 */
void Chassis_Control_Loop(void)
{
    /* 1. 取命令。自动模式超过 200ms 没收到新指令就归零, 防止上位机掉线后飞车 */
    float tV = cmd_V;
    float tW = cmd_W;
    if (chassis_mode == CHASSIS_MODE_AUTO) {
        if (auto_ticks >= AUTO_TIMEOUT_TICKS) {
            tV = 0.0f;
            tW = 0.0f;
        } else {
            auto_ticks++;
        }
    }

    /* 2. 航向保持: 直行时用 IMU 偏航误差补一个很小的 W。
     *    传入实际轮速, 让车在减速/滑行阶段也继续被钳住方向。 */
    tW += YawHold_Process(tV, tW,
              fabsf(robot_state.raw_pulse_L) + fabsf(robot_state.raw_pulse_R));

    /* 3. 目标死区: 太小的 V/W 直接当 0, 避免电机在零点附近嗡嗡爬行。
     *    分轴判断, 这样"只转向不前进"时 V 归零不会把 W 一起清掉。 */
    if (fabsf(tV) < DEADBAND_V) tV = 0.0f;
    if (fabsf(tW) < DEADBAND_W) tW = 0.0f;

    /* 4. 逆解: 底盘 V/W → 左右轮在 25ms 内应该走多少脉冲 */
    Inverse_Kinematics(tV, tW);

    /* 5. 读编码器: 硬件计数器读完后清零, 得到本周期脉冲数。
     *    右轮取负号是因为电机/编码器安装方向与左轮相反, 只在这里翻一次。 */
    short pulse_L = (short)__HAL_TIM_GET_COUNTER(&htim4);
    short pulse_R = -(short)__HAL_TIM_GET_COUNTER(&htim8);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    __HAL_TIM_SET_COUNTER(&htim8, 0);

    /* EMA 只用于显示与上报的平滑; 速度环反馈仍用原始脉冲, 不额外引入相位滞后 */
    static float filt_L = 0.0f;
    static float filt_R = 0.0f;
    filt_L = EMA_ALPHA * (float)pulse_L + (1.0f - EMA_ALPHA) * filt_L;
    filt_R = EMA_ALPHA * (float)pulse_R + (1.0f - EMA_ALPHA) * filt_R;

    robot_state.raw_pulse_L = (float)pulse_L;
    robot_state.raw_pulse_R = (float)pulse_R;
    robot_state.actual_pulse_L = filt_L;
    robot_state.actual_pulse_R = filt_R;

    /* 6. 位置式 PID: 目标脉冲 vs 实际脉冲。
     *    目标为 0 时误差 = -实际脉冲, 输出反向, 这就是电机制动。 */
    SpeedPID_L.Target = robot_state.target_pulse_L;
    SpeedPID_L.Actual = (float)pulse_L;
    SpeedPID_R.Target = robot_state.target_pulse_R;
    SpeedPID_R.Actual = (float)pulse_R;
    PID_Update(&SpeedPID_L);
    PID_Update(&SpeedPID_R);

    /* 7. PID 输出是 -100..100 的占空比百分比, 交给电机驱动 */
    Set_Motor_PWM((int)SpeedPID_L.Out, (int)SpeedPID_R.Out);
}
