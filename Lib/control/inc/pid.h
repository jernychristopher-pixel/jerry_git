#ifndef __PID_H
#define __PID_H

typedef struct {
    float Target;
    float Actual;
    float Out;

    float Kp;
    float Ki;
    float Kd;

    float Error0;
    float Error1;      /* 上一拍误差, 当前 D 项改用 ActualPrev, 保留用于调试 */
    float ErrorInt;
    float DFiltered;   /* 微分项一阶低通后的值, 抑制编码器量化毛刺 */
    float ActualPrev;  /* 上一拍实测值, 用于对测量值微分, 避免目标阶跃的微分冲击 */

    /* 静摩擦前馈: u_ff = sign(target) * (FF_Dead + FF_Slope*|target|)
     * 本车 52% 以下几乎不转, 20/30/60 r/min 分别约需 55/57/61% 占空比。
     * 纯 P 组故意设为 0, 用来暴露 P 的稳态误差; PI/PID 组打开。 */
    float FF_Dead;
    float FF_Slope;
    float FFOut;       /* 经过平滑后的实际前馈量, 防止目标跳变时占空比瞬间顶满 */

    float OutMax;
    float OutMin;
} PID_t;

extern PID_t SpeedPID_L;
extern PID_t SpeedPID_R;

void PID_Init(PID_t *p);
void PID_Update(PID_t *p);

/* M4-3 三组预设: 0=纯P, 1=PI, 2=PID。切换时清空积分/微分历史。 */
void PID_Apply_Preset(PID_t *p, unsigned preset);

#endif
