#include "pid.h"

/* 积分分离阈值(r/min)。
 * 静摩擦前馈负责把占空比先托到 52% 死区附近, 误差大时不需要积分去顶;
 * 误差进入 5 r/min 以内再投入积分, 避免起步阶段积分冲到 60% 造成超调。 */
#define PID_INT_SEP 5.0f
/* 微分低通系数: 0.3 表示每拍只吸收 30% 的新差值 */
#define PID_D_ALPHA 0.3f
/* 前馈一阶平滑: 10ms 一拍, 0.08 约等于 120ms 时间常数。
 * 上升沿慢一点, 让 0 -> 60 r/min 时占空比不会从 0 直接跳到 70%+;
 * 下降沿快一点, 目标骤降时前馈及时退掉, 交给 PID 制动。 */
#define PID_FF_RISE_ALPHA 0.08f
#define PID_FF_FALL_ALPHA 0.30f

PID_t SpeedPID_L;
PID_t SpeedPID_R;

/**
  * 函    数：PID初始化
  * 参    数：p 指定结构体的地址
  * 返 回 值：无
  */
void PID_Init(PID_t *p)
{
    p->Target = 0;
    p->Actual = 0;
    p->Out = 0;
    p->Error0 = 0;
    p->Error1 = 0;
    p->ErrorInt = 0;
    p->DFiltered = 0;
    p->ActualPrev = 0;
    p->FFOut = 0;
}

/**
  * @brief 位置式 PID 计算
  *
  * 位置式:  Out = FF + Kp*e + Ki*Σe - Kd*d(actual)/dt
 *   这里的 Σe 是"每个控制周期累加一次误差", 周期固定 10ms,
  *   所以 Ki/Kd 的物理含义与采样周期绑定; 改 TIM1 周期时要重新调参。
  *
  * 三个保护:
  *   1. 积分分离: 误差太大时先不积分, 先把比例项拉过去;
  *   2. 输出限幅: Out 限制在 OutMin..OutMax (对应 PWM 百分比);
  *   3. 反算抗饱和: 输出顶到限幅时, 反推一个刚好不饱和的 ErrorInt,
  *      避免积分继续累积、退饱和时猛冲。
  *
  * @param p 指向 PID 结构体
  */
void PID_Update(PID_t *p)
{
    /* 保留上一拍误差用于调试/兼容; D 项现在对 Actual 求导, 不再用它 */
    p->Error1 = p->Error0;
    p->Error0 = p->Target - p->Actual;

    /* Ki=0 时积分项强制归零, 这样调成纯 P 控制器时不会残留历史积分 */
    if (p->Ki != 0) {
        /* 积分分离: 离目标太远时先不积分, 避免到达目标时冲过头 */
        if (p->Error0 < PID_INT_SEP && p->Error0 > -PID_INT_SEP) {
            p->ErrorInt += p->Error0;
        }
    } else {
        p->ErrorInt = 0;
    }

    /* 微分项对"测量值"而不是"误差"求导:
     * 目标阶跃时 d(error) 会瞬间出现一个尖峰, 把输出往前推, 加剧超调;
     * 对 Actual 求导则只反映真实转速变化, 阶跃瞬间不会产生微分冲击。
     * 先做一阶低通: 编码器 10ms 只有 1 个脉冲(4.12 r/min)的分辨率,
     * 原始差分全是台阶, 直接把 Kd 乘上去只会得到尖刺。 */
    float d_actual = p->Actual - p->ActualPrev;
    p->ActualPrev = p->Actual;
    p->DFiltered += PID_D_ALPHA * (d_actual - p->DFiltered);

    /* 静摩擦前馈: 先把占空比推到"刚好能转"的附近, PID 只负责修偏差。
     * 前馈本身必须平滑: 空载电机从 0 直接给 61% 会瞬间冲到 90 r/min 以上。
     * 上升沿用 120ms 一阶低通, 下降沿用 30ms, 兼顾起步平顺和减速及时。 */
    float u_ff_target = 0.0f;
    if (p->Target > 0.05f) {
        u_ff_target = p->FF_Dead + p->FF_Slope * p->Target;
    } else if (p->Target < -0.05f) {
        u_ff_target = -(p->FF_Dead + p->FF_Slope * (-p->Target));
    }
    if (u_ff_target > p->FFOut) {
        p->FFOut += PID_FF_RISE_ALPHA * (u_ff_target - p->FFOut);
    } else {
        p->FFOut += PID_FF_FALL_ALPHA * (u_ff_target - p->FFOut);
    }

    /* 位置式 PID 主体 */
    p->Out = p->FFOut
           + p->Kp * p->Error0
           + p->Ki * p->ErrorInt
           - p->Kd * p->DFiltered;

    /* 反算抗饱和: 输出已经顶到上限/下限时, 把积分反推回"刚好不饱和"的值 */
    if (p->Out > p->OutMax) {
        p->Out = p->OutMax;
        if (p->Ki != 0) {
            p->ErrorInt = (p->OutMax - p->FFOut - p->Kp * p->Error0 + p->Kd * p->DFiltered) / p->Ki;
        }
    }
    if (p->Out < p->OutMin) {
        p->Out = p->OutMin;
        if (p->Ki != 0) {
            p->ErrorInt = (p->OutMin - p->FFOut - p->Kp * p->Error0 + p->Kd * p->DFiltered) / p->Ki;
        }
    }
}

/* M4-3 三组预设参数: 台架整定值(2026-11-19, 空载)。
 * 单位统一为 r/min, 控制周期 10ms。
 * 本车静摩擦死区约 50% 占空比, 所以 Kp 必须到 1.0 才能起步;
 * P 组故意保留这一组, 用来展示"纯 P 稳态误差很大"。 */
/* Kp, Ki, Kd, FF_Dead, FF_Slope */
static const float PID_PRESETS[3][5] = {
    {1.00f, 0.00f, 0.00f, 0.0f, 0.00f},   /* 纯 P: 不加前馈, 暴露稳态误差 */
    {0.30f, 0.02f, 0.00f, 52.0f, 0.15f},  /* PI  */
    {0.30f, 0.02f, 0.10f, 52.0f, 0.15f},  /* PID */
};

void PID_Apply_Preset(PID_t *p, unsigned preset)
{
    if (preset > 2U) preset = 0U;
    p->Kp = PID_PRESETS[preset][0];
    p->Ki = PID_PRESETS[preset][1];
    p->Kd = PID_PRESETS[preset][2];
    p->FF_Dead = PID_PRESETS[preset][3];
    p->FF_Slope = PID_PRESETS[preset][4];
    p->ErrorInt = 0.0f;
    p->Error0 = 0.0f;
    p->Error1 = 0.0f;
    p->DFiltered = 0.0f;
    p->ActualPrev = 0.0f;
    p->FFOut = 0.0f;
    p->Out = 0.0f;
}
