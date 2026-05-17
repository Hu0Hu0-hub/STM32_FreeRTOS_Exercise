#ifndef INC_PID_H_
#define INC_PID_H_

#include "tim.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>


//pid结构体
typedef struct
{
    float kp;
    float ki;
    float kd;

    float target;       // 运动学解算出来的目标速度
    float actual;       // 编码器读回来的实际速度

    float error;        // 当前误差
    float last_error;   // 上次误差

    float integral;     // 积分项累加
    float max_integral; // 积分限幅

    float output;       // PID计算后输出的 PWM 值
    float max_output;   // 输出限幅
} PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float p, float i, float d);
int PID_Calc(PID_TypeDef *pid, float target_speed, float actual_speed);

#endif /* INC_PID_H_ */
