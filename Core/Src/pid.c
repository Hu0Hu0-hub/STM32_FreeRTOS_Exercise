#include "pid.h"
#include "tim.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

// 初始化 PID 参数
void PID_Init(PID_TypeDef *pid, float p, float i, float d)
{
        pid->kp = p;
        pid->ki = i;
        pid->kd = d;
        pid->error = 0;
        pid->last_error = 0;
        pid->integral = 0;
        pid->max_integral = 2000;
        pid->max_output = 1000;
}


// PID计算
int PID_Calc(PID_TypeDef *pid, float target_speed, float actual_speed)
{
        pid->target = target_speed;
        pid->actual = actual_speed;
        // 死区
        if (pid->target == 0.0f)
        {
                pid->error = 0;
                pid->last_error = 0;
                pid->integral = 0;
                pid->output = 0;
                return 0;
        }
        // 算误差
        pid->error = pid->target - pid->actual;

        //算积分
        pid->integral += pid->error;
        if(pid->integral > pid->max_integral) pid->integral = pid->max_integral;
        if(pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;

        //套公式：P*误差 + I*积分 + D*(当前误差-上次误差)
        pid->output = (pid->kp * pid->error) +
                      (pid->ki * pid->integral) +
                      (pid->kd * (pid->error - pid->last_error));

        // 4. 更新历史误差
        pid->last_error = pid->error;

        // 5. 输出限幅
        if(pid->output > pid->max_output) pid->output = pid->max_output;
        if(pid->output < -pid->max_output) pid->output = -pid->max_output;

        return (int)pid->output;
}
