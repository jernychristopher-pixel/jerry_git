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

/* 创建左右轮速度 PID 实例, 在 kinematics.c 中初始化参数, 在 tim.c 中调用 PID_Update() 更新输出 */
PID_t SpeedPID_L;
PID_t SpeedPID_R;

/**
  * 函    数：PID初始化
  * 参    数：p 指定结构体的地址
  * 返 回 值：无
  */

//按照位置式PID的公式: Out = FF + Kp*e_k + Ki*Σe_k - Kd*d(Actual)/dt 进行置0初始化
//10ms周期, 采样周期固定; Ki/Kd 的数值与这个周期绑定, 改周期必须重新调参。
//
//为什么 D 项前面是减号(README 的公式写的是 +Kd*de/dt):
//  本工程对"测量值 Actual"求微分, 不是对"误差 e"求微分。
//  目标不变时 e = Target - Actual, 所以 d(e)/dt = -d(Actual)/dt,
//  于是 -Kd*d(Actual)/dt 与 +Kd*d(e)/dt 在数学上等价。
//  但目标阶跃时两者不一样:
//    例: 目标 0 -> 60 r/min, 第一拍 Actual 仍为 0。
//    用 +Kd*d(e)/dt: e 从 0 跳到 60, 未滤波时 D 瞬间加 0.1*60 = 6% 占空比,
//                    等于在起步瞬间又往前推一把, 会加剧超调(setpoint kick)。
//    用 -Kd*d(Actual)/dt: 第一拍 d(Actual)=0, D 不出力; 之后 Actual 上升,
//                    d(Actual)>0, D 变成阻尼, 抑制继续加速。
//  目标下降时同理: +Kd*d(e)/dt 会在阶跃瞬间产生一个负尖峰, 额外加一脚制动,
//  容易在下冲时冲过头; 对 Actual 求导在阶跃瞬间没有这个尖峰。
//  阶跃结束后目标不变, 两种写法又等价, 所以差别只在"目标切换的那一拍"。
//
void PID_Init(PID_t *p)
{
    p->Target = 0;
    p->Actual = 0;
    p->Out = 0;// 输出,对应u_k
    p->Error0 = 0;// 当前拍误差,对应e_k
    p->Error1 = 0;// 上一拍误差,对应e_{k-1}
    p->ErrorInt = 0;// 积分项,对应∫e dt,任务使用离散版的积分
    p->DFiltered = 0;// 微分项一阶低通后的值,对应de/dt
    p->ActualPrev = 0;// 上一拍实际值,对应x_{k-1}
    p->FFOut = 0;// 前馈输出,对应u_ff
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
        /* 积分分离: 离目标太远时切除积分, 避免到达目标时冲过头。
         *
         * 注意这里必须"清零"而不是"冻结":
         *   冻结会保留之前积下的值。若某一方向持续超调(|e| >= 阈值),
         *   积分在这段时间里既不增也不减, 坏值永远退不回来。
         *   台架实测 ±150 r/min 每 1.5 s 换向时, +150 的稳态会逐轮
         *   148 -> 105 -> 78 r/min, -150 则 -160 -> -203 -> -233 r/min,
         *   PWM 同步漂移; 改成清零后积分每拍都从 0 重建, 不再单向累积。 */
        if (p->Error0 < PID_INT_SEP && p->Error0 > -PID_INT_SEP) {
            p->ErrorInt += p->Error0;
        } else {
            p->ErrorInt = 0.0f;
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
    p->ActualPrev = p->Actual;// 更新上一拍实际值, 用于下一拍微分计算
    /* 一阶 IIR 低通: 新值 = 70% 旧值 + 30% 本拍差分。
     * 时间常数 tau = -dt/ln(1-alpha) = -10ms/ln(0.7) ≈ 28ms
     * (工程上常近似为 dt/alpha ≈ 33ms)。alpha 越小越平滑、相位滞后越大。 */
    p->DFiltered += PID_D_ALPHA * (d_actual - p->DFiltered);

    /* 静摩擦前馈: 先把占空比推到"刚好能转"的附近, PID 只负责修偏差。
     * 目标值: u_ff_target = sign(Target) * (FF_Dead + FF_Slope*|Target|)
     * 平滑:   FFOut_k = FFOut_{k-1} + alpha * (u_ff_target - FFOut_{k-1})
     * 例: Target=60 时 u_ff_target=52+0.15*60=61%。
     *     从 0 起步, 上升 alpha=0.08, 约 30 拍(300ms)到 56%, 50 拍(500ms)到 60%。
     *     若不平滑直接给 61%, 空载实测峰值 93.41 r/min, 超调 55.68%。
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

    //位置式主体公式(readme给出)
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
 *
 * 静摩擦死区实测(WHEELTEC C10B 台架 + 28:1 减速箱, 电池 11.47V, 空载):
 *   目标 20 r/min -> 稳态约 55% 占空比
 *   目标 30 r/min -> 稳态约 57%
 *   目标 60 r/min -> 稳态约 61%
 *   100% 占空比   -> 空载最高约 78 r/min
 *   52% 以下基本不转(减速箱静摩擦 + 电机齿槽 + H 桥压降 + PWM 平均电压共同造成,
 *   不是单纯"电机参数"; 换电池电压、换减速比、装车落地都会变)。
 *
 * 为什么纯 P 稳态误差这么大:
 *   Kp=1.0 时, 要产生 53% 的稳态占空比只需要 e=53 r/min,
 *   所以目标 60 时实际稳态均值只有约 6.6~6.8 r/min(稳态误差约 -53 r/min),
 *   峰峰值纹波约 6.87 r/min。
 *   实测见 M4-3/data/step_P.csv: 稳态 PWM 50.4~57.3%, 转速均值约 6.75 r/min;
 *   对比曲线见 M4-3/docs/plots/step_compare.png。
 *
 * 注意: 52.0/0.15 只对这台台架有效。换 MG513X 等其它电机必须重新测死区、
 * 重新拟合 FF_Dead/FF_Slope, 并同步确认减速比与 ENCODER_CPR, 不能照抄。
 * P 组故意不用前馈, 用来展示"纯 P 在静摩擦下稳态误差很大"。 */
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
