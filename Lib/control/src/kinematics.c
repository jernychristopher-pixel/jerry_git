#include "kinematics.h"
#include "yaw_hold.h"
#include "pid.h"
#include "motor.h"
#include "tim.h"
#include <math.h>

// ===== 物理常量已上移到 kinematics.h, 全工程唯一来源, 这里不再重复定义 =====
#define CTRL_PERIOD      0.010f   /* TIM1 控制周期 10ms, 与 CubeMX 配置保持一致 */
#define EMA_ALPHA        0.4f     /* 显示用低通系数, 越大越跟手 */
/* 速度反馈滑动窗口: 3 拍 = 30ms。
 * 10ms 单拍的量化是 1 个脉冲 = 4.12 r/min, 20 r/min 时纹波就有 ±20%;
 * 3 拍求和后分辨率提到 1.37 r/min, 而更新率仍是 100Hz。
 * 代价是约 10ms 群延迟, 相对电机 ~250ms 的机械时间常数可以接受。
 * 试过 5 拍(50ms): 低速纹波更小, 但加速段窗口里混入旧值, 60 r/min 超调反而变大。 */
#define RPM_WINDOW       3

/* 上位机断线保护: 超过 200ms(20个10ms节拍)未收到新指令 → 目标归零 */
#define AUTO_TIMEOUT_TICKS  20U
/* 目标死区: 太小的速度直接当 0, 抑制静止时的爬行和嗡鸣 */
#define DEADBAND_V   0.02f   /* m/s */
#define DEADBAND_W   0.04f   /* rad/s */

// 运动学状态全局变量
Robot_State_t robot_state;

/* 命令邮箱: 主循环写, TIM1 10ms中断读; 写入时关中断保证成对更新 */
static volatile float    cmd_V = 0.0f;
static volatile float    cmd_W = 0.0f;
static volatile uint32_t auto_ticks = 0;
/* M4-3 轮速整定模式: 目标直接是左右轮 r/min, 不走 V/W 逆解 */
static volatile float    wheel_rpm_target_L = 0.0f;
static volatile float    wheel_rpm_target_R = 0.0f;
static volatile uint8_t  wheel_rpm_enable = 0;

/**
 * @brief 统一速度命令入口: 上位机下行帧和文本命令都走这里
 *        纯上位机版本没有本地手动控制源, 因此不再做模式互斥
 */
void Chassis_Set_Cmd(float V, float W) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    cmd_V = CHASSIS_V_SIGN * V;   /* 对外坐标系 -> 内部电机坐标系 */
    cmd_W = CHASSIS_W_SIGN * W;
    auto_ticks = 0;               /* 喂狗: 收到新的上位机命令 */
    if (primask == 0) __enable_irq();
}

/* 进入轮速整定模式: 左右轮各自给目标 r/min, 正数代表前进 */
void Chassis_Set_Wheel_Rpm(float left_rpm, float right_rpm) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    wheel_rpm_target_L = left_rpm;
    wheel_rpm_target_R = right_rpm;
    wheel_rpm_enable = 1;
    if (primask == 0) __enable_irq();
}

/* 退出轮速整定模式, 回到上位机的 V/W 控制 */
void Chassis_Exit_Wheel_Rpm(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    wheel_rpm_enable = 0;
    wheel_rpm_target_L = 0.0f;
    wheel_rpm_target_R = 0.0f;
    /* 不沿用退出前的旧命令, 等上位机重新下发; 否则 #X 后车会突然动一下 */
    cmd_V = 0.0f;
    cmd_W = 0.0f;
    auto_ticks = AUTO_TIMEOUT_TICKS;
    /* 退出整定时把积分/微分历史一起清掉, 否则 target=0 时
     * 积分项会冻结在上一次的出力上, 电机停着还一直顶着 PWM。 */
    PID_Init(&SpeedPID_L);
    PID_Init(&SpeedPID_R);
    if (primask == 0) __enable_irq();
}

void Chassis_GetWheelRpm(float *left_rpm, float *right_rpm) {
    *left_rpm = robot_state.actual_rpm_L;
    *right_rpm = robot_state.actual_rpm_R;
}

/**
 * @brief 运动学逆解：将线速度V(m/s)和角速度W(rad/s)转换为目标轮速脉冲
 */
void Inverse_Kinematics(float V, float W) {
    // 1. 算出左右轮的目标线速度 (m/s)
    float v_left  = V - (W * WHEEL_TRACK / 2.0f);
    float v_right = V + (W * WHEEL_TRACK / 2.0f);

    /* 2. 轮速 m/s -> r/min: v / 周长 = rev/s, 再 x60 = r/min */
    robot_state.target_rpm_L = v_left  * 60.0f / WHEEL_PERIMETER;
    robot_state.target_rpm_R = v_right * 60.0f / WHEEL_PERIMETER;
}

/* 本周期脉冲数 -> r/min。60 / (CPR * CTRL_PERIOD) = 60 / 14.56 = 4.1209 */
float Pulse_To_Rpm(float pulse) {
    return pulse * 60.0f / (ENCODER_CPR * CTRL_PERIOD);
}

/* r/min -> 本周期脉冲数, 用于显示或协议换算 */
float Rpm_To_Pulse(float rpm) {
    return rpm * ENCODER_CPR * CTRL_PERIOD / 60.0f;
}

/* 轮速 m/s -> r/min */
float Mps_To_Rpm(float v) {
    return v * 60.0f / WHEEL_PERIMETER;
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
 * 调用位置: TIM1 更新中断, 每 10ms 一次。
 */
void Chassis_Control_Loop(void)
{
    if (wheel_rpm_enable) {
        /* M4-3 轮速整定模式: 目标直接是左右轮 r/min, 不走 V/W 逆解 */
        robot_state.target_rpm_L = wheel_rpm_target_L;
        robot_state.target_rpm_R = wheel_rpm_target_R;
    } else {
        /* 1. 取命令。超过 200ms 没收到新指令就归零, 防止上位机掉线后飞车 */
        float tV = cmd_V;
        float tW = cmd_W;
        if (auto_ticks >= AUTO_TIMEOUT_TICKS) {
            tV = 0.0f;
            tW = 0.0f;
        } else {
            auto_ticks++;
        }

        /* 2. 航向保持: 直行时用 IMU 偏航误差补一个很小的 W。
         *    传入实际轮速, 让车在减速/滑行阶段也继续被钳住方向。 */
        tW += YawHold_Process(tV, tW,
                  fabsf(robot_state.raw_pulse_L) + fabsf(robot_state.raw_pulse_R));

        /* 3. 目标死区: 太小的 V/W 直接当 0, 避免电机在零点附近嗡嗡爬行。
         *    分轴判断, 这样"只转向不前进"时 V 归零不会把 W 一起清掉。 */
        if (fabsf(tV) < DEADBAND_V) tV = 0.0f;
        if (fabsf(tW) < DEADBAND_W) tW = 0.0f;

        /* 4. 逆解: 底盘 V/W -> 左右轮目标 r/min */
        Inverse_Kinematics(tV, tW);
    }

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

    /* 6. 脉冲 -> r/min: PID 给定与反馈统一成 r/min, 不再和编码器计数绑定。
     *    先用 RPM_WINDOW(当前为 3)拍滑动窗口求和, 再换算成 r/min, 压低低速量化纹波。 */
    {
        static int16_t  hist_L[RPM_WINDOW], hist_R[RPM_WINDOW];
        static int32_t  sum_L = 0, sum_R = 0;
        static uint8_t  pos = 0;

        sum_L += (int32_t)pulse_L - hist_L[pos];
        sum_R += (int32_t)pulse_R - hist_R[pos];
        hist_L[pos] = pulse_L;
        hist_R[pos] = pulse_R;
        pos = (uint8_t)((pos + 1U) % RPM_WINDOW);

        robot_state.actual_rpm_L = Pulse_To_Rpm((float)sum_L / (float)RPM_WINDOW);
        robot_state.actual_rpm_R = Pulse_To_Rpm((float)sum_R / (float)RPM_WINDOW);
    }

    /* 7. 位置式 PID: 目标为 0 时误差 = -实际转速, 输出反向, 这就是电机制动 */
    SpeedPID_L.Target = robot_state.target_rpm_L;
    SpeedPID_L.Actual = robot_state.actual_rpm_L;
    SpeedPID_R.Target = robot_state.target_rpm_R;
    SpeedPID_R.Actual = robot_state.actual_rpm_R;

    /* 目标 0 且确实停住了: 清掉积分, 防止"停着还输出一个固定 PWM"。
     * 只在实际转速也接近 0 时清, 滑行/刹车阶段仍让积分参与制动。 */
    if (fabsf(SpeedPID_L.Target) < 0.05f && fabsf(SpeedPID_L.Actual) < 1.0f) {
        SpeedPID_L.ErrorInt = 0.0f;
        SpeedPID_L.DFiltered = 0.0f;
    }
    if (fabsf(SpeedPID_R.Target) < 0.05f && fabsf(SpeedPID_R.Actual) < 1.0f) {
        SpeedPID_R.ErrorInt = 0.0f;
        SpeedPID_R.DFiltered = 0.0f;
    }

    PID_Update(&SpeedPID_L);
    PID_Update(&SpeedPID_R);

    /* 8. PID 输出是 -100..100 的占空比百分比, 交给电机驱动 */
    Set_Motor_PWM((int)SpeedPID_L.Out, (int)SpeedPID_R.Out);
}
