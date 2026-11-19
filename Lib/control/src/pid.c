#include "pid.h"

/* 积分分离阈值(脉冲): |误差| 超过它时暂停积分, 防止大幅变速时积分卷绕。
 * 60 是按满速稳态误差(约 37 脉冲)留出的余量; 设得太小, 高速段积分会被永久跳过。 */
#define PID_INT_SEP 60.0f

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
}

/**
  * @brief 位置式 PID 计算
  *
  * 位置式:  Out = Kp*e + Ki*Σe + Kd*(e - e_prev)
  *   这里的 Σe 是"每个控制周期累加一次误差", 周期固定 25ms,
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
    /* 保存上一拍误差, 供微分项使用 */
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

    /* 位置式 PID 主体 */
    p->Out = p->Kp * p->Error0
           + p->Ki * p->ErrorInt
           + p->Kd * (p->Error0 - p->Error1);

    /* 反算抗饱和: 输出已经顶到上限/下限时, 把积分反推回"刚好不饱和"的值 */
    if (p->Out > p->OutMax) {
        p->Out = p->OutMax;
        if (p->Ki != 0) {
            p->ErrorInt = (p->OutMax - p->Kp * p->Error0 - p->Kd * (p->Error0 - p->Error1)) / p->Ki;
        }
    }
    if (p->Out < p->OutMin) {
        p->Out = p->OutMin;
        if (p->Ki != 0) {
            p->ErrorInt = (p->OutMin - p->Kp * p->Error0 - p->Kd * (p->Error0 - p->Error1)) / p->Ki;
        }
    }
}
