#include "motor.h"
#include "tim.h"

// TIM3 PWM满量程 (与CubeMX Period=65535一致)
#define PWM_FULL  65535
#define PWM_SCALE (PWM_FULL / 100)
#define DEADBAND    1     // deadband: brake only within +-1, finer low-speed control
#define MIN_START   0     // 起步偏置已关闭(0): 低速直接用比例驱动, 消除低速顿挫与前后方向不对称

void Set_Motor_PWM(int pwm_left, int pwm_right) {
    static int8_t dir_L = 0, dir_R = 0;
    int duty, pwm;

    // ===== 左轮：PA6=IN1(CH1) / PA7=IN2(CH2) =====
    pwm = pwm_left;
    if (pwm > DEADBAND) {
        if (dir_L == 0 && pwm < MIN_START) {
            // 静止起步：低于起步偏置不输出，避免低速来回跳
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_FULL);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM_FULL);
        } else {
            duty = pwm * PWM_SCALE;
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
            dir_L = 1;
        }
    } else if (pwm < -DEADBAND) {
        if (dir_L == 0 && pwm > -MIN_START) {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_FULL);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM_FULL);
        } else {
            duty = -pwm * PWM_SCALE;
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty);
            dir_L = -1;
        }
    } else {
        // 死区内：刹车 (IN1=IN2=HIGH → 慢衰减制动)
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_FULL);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM_FULL);
        dir_L = 0;
    }

    // ===== 右轮：PB1=IN1(CH4) / PB0=IN2(CH3) =====
    pwm = pwm_right;
    if (pwm > DEADBAND) {
        if (dir_R == 0 && pwm < MIN_START) {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_FULL);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_FULL);
        } else {
            duty = pwm * PWM_SCALE;
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, duty);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
            dir_R = 1;
        }
    } else if (pwm < -DEADBAND) {
        if (dir_R == 0 && pwm > -MIN_START) {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_FULL);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_FULL);
        } else {
            duty = -pwm * PWM_SCALE;
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty);
            dir_R = -1;
        }
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_FULL);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_FULL);
        dir_R = 0;
    }
}
