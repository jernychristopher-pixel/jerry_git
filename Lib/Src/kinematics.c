#include "kinematics.h"
#include "yaw_hold.h"
#include "pid.h"
#include "motor.h"
#include "tim.h"
#include <math.h>

// ===== 物理常量已上移到 kinematics.h, 全工程唯一来源, 这里不再重复定义 =====
#define CTRL_PERIOD      0.025f   // 25ms 控制周期
#define EMA_ALPHA        0.4f     // actual-speed EMA coefficient

/* 自动模式断线保护: 超过 200ms(8个25ms节拍)未收到新指令 → 减速停车 */
#define AUTO_TIMEOUT_TICKS  8U
/* 目标速度斜坡: 每个25ms节拍的最大变化量; 减速斜坡更陡, 刹车更果断 */
#define RAMP_DV_UP_MAX    0.10f
#define RAMP_DV_DOWN_MAX  0.30f
#define RAMP_DW_MAX       0.75f
/* 目标过小直接归零, 抑制爬行 */
#define DEADBAND_V   0.02f
#define DEADBAND_W   0.04f
/* 停车刹车: 无积分强力比例+微分制动; 力度可通过VOFA #O 在线调 */
#define BRAKE_KP_DEFAULT  10.0f
#define BRAKE_KD_DEFAULT  0.3f
/* 刹车输出斜率限幅: 每个25ms节拍制动量的最大变化 (%), 越小越柔和 */
#define BRAKE_SLEW_MAX    25.0f
float brake_kp = BRAKE_KP_DEFAULT;
float brake_kd = BRAKE_KD_DEFAULT;

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
 * @brief 25ms 定时器中断回调函数 (系统的发动机)
 * 需要在 TIM1 的中断里调用此函数！
 */
void Chassis_Control_Loop(void) {
    static float curV = 0.0f, curW = 0.0f;   /* 斜坡后的实际指令, 手动/自动共用 */
    static float filt_L = 0.0f;
    static float filt_R = 0.0f;

    // 1. 读取命令: 自动模式超时未喂狗 → 目标归零减速停车
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

    // 2. 航向保持: 直行时叠加IMU偏航修正; 传实际轮速, 刹车滑行期间继续钳制
    tW += YawHold_Process(tV, tW,
              fabsf(robot_state.raw_pulse_L) + fabsf(robot_state.raw_pulse_R));
    float dV = tV - curV;
    float dW = tW - curW;
    if (dV >  RAMP_DV_UP_MAX)   dV =  RAMP_DV_UP_MAX;
    if (dV < -RAMP_DV_DOWN_MAX) dV = -RAMP_DV_DOWN_MAX;
    if (dW >  RAMP_DW_MAX) dW =  RAMP_DW_MAX;
    if (dW < -RAMP_DW_MAX) dW = -RAMP_DW_MAX;
    curV += dV;
    curW += dW;

    if (!(curV > DEADBAND_V || curV < -DEADBAND_V ||
          curW > DEADBAND_W || curW < -DEADBAND_W)) {
        curV = 0.0f;
        curW = 0.0f;
    }

    // 4. 逆解算: 实际指令 → 左右轮目标脉冲
    Inverse_Kinematics(curV, curW);

    // 5. 极速读取编码器脉冲并清零
    short pulse_L = (short)__HAL_TIM_GET_COUNTER(&htim4);
    short pulse_R = -(short)__HAL_TIM_GET_COUNTER(&htim8);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    __HAL_TIM_SET_COUNTER(&htim8, 0);

    // EMA 滤波仅用于 OLED 显示; PID 用原始脉冲,保持调好的动态响应
    filt_L = EMA_ALPHA * (float)pulse_L + (1.0f - EMA_ALPHA) * filt_L;
    filt_R = EMA_ALPHA * (float)pulse_R + (1.0f - EMA_ALPHA) * filt_R;

    robot_state.raw_pulse_L = (float)pulse_L;
    robot_state.raw_pulse_R = (float)pulse_R;
    robot_state.actual_pulse_L = filt_L;
    robot_state.actual_pulse_R = filt_R;

    // 6. 位置式 PID：目标脉冲 vs 实际脉冲
    int pwm_L, pwm_R;
    /* 刹车分支状态: 是否在刹车中, 上一拍刹车输出(斜率限幅用), 上一拍误差 */
    static uint8_t brk_active = 0;
    static float   brk_out_L  = 0.0f, brk_out_R  = 0.0f;
    static float   brk_prev_L = 0.0f, brk_prev_R = 0.0f;
    if (fabsf(robot_state.target_pulse_L) < 0.5f && fabsf(robot_state.target_pulse_R) < 0.5f) {
        /* 刹车模式: 目标接近零, 无积分强力比例+微分制动,
           防止积分卷绕造成停车冲量; 刹车期间航向环的小差速目标并入,
           停稳后自动落入硬件刹车 */
        float err_L = robot_state.target_pulse_L - robot_state.raw_pulse_L;
        float err_R = robot_state.target_pulse_R - robot_state.raw_pulse_R;

        /* 刚进入刹车: 丢弃上一轮微分历史, 输出从零重建,
           避免刹车首拍直接跳到满幅造成顿挫 */
        if (!brk_active) {
            brk_prev_L = err_L;   brk_prev_R = err_R;
            brk_out_L  = 0.0f;    brk_out_R  = 0.0f;
            brk_active = 1;
        }

        float want_L = brake_kp * err_L + brake_kd * (err_L - brk_prev_L);
        float want_R = brake_kp * err_R + brake_kd * (err_R - brk_prev_R);
        brk_prev_L = err_L;   brk_prev_R = err_R;

        /* 输出斜率限幅: 制动量按固定速率建立, 把单拍饱和冲量摊到几拍,
           同时抹平低速时编码器量化造成的台阶 */
        float d_L = want_L - brk_out_L;
        float d_R = want_R - brk_out_R;
        if (d_L >  BRAKE_SLEW_MAX) d_L =  BRAKE_SLEW_MAX;
        if (d_L < -BRAKE_SLEW_MAX) d_L = -BRAKE_SLEW_MAX;
        if (d_R >  BRAKE_SLEW_MAX) d_R =  BRAKE_SLEW_MAX;
        if (d_R < -BRAKE_SLEW_MAX) d_R = -BRAKE_SLEW_MAX;
        brk_out_L += d_L;
        brk_out_R += d_R;

        if (brk_out_L >  100.0f) brk_out_L =  100.0f;
        if (brk_out_L < -100.0f) brk_out_L = -100.0f;
        if (brk_out_R >  100.0f) brk_out_R =  100.0f;
        if (brk_out_R < -100.0f) brk_out_R = -100.0f;

        pwm_L = (int)brk_out_L;
        pwm_R = (int)brk_out_R;
    } else {
        brk_active = 0;
        SpeedPID_L.Target = robot_state.target_pulse_L;
        SpeedPID_L.Actual = (float)pulse_L;
        SpeedPID_R.Target = robot_state.target_pulse_R;
        SpeedPID_R.Actual = (float)pulse_R;
        PID_Update(&SpeedPID_L);
        PID_Update(&SpeedPID_R);
        pwm_L = (int)SpeedPID_L.Out;
        pwm_R = (int)SpeedPID_R.Out;
    }

    // 7. 驱动底盘硬件
    Set_Motor_PWM(pwm_L, pwm_R);
}
