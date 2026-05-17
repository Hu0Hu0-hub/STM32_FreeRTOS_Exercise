#include "motor.h"
#include "tim.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "pid.h"

Motor_t Chassis_Motors[4] =
{
    // M1: 左前轮 (PE0, PE1, TIM4_CH1, 编码器 TIM2 32位 计数器记忆 pid结构体 实际速度 目标速度)
    {GPIOE, GPIO_PIN_0, GPIOE, GPIO_PIN_1, TIM_CHANNEL_1, &htim2, 0, {0}, 0, 0},
    // M2: 右前轮 (PE2, PE3, TIM4_CH2, 编码器 TIM3 16位)
    {GPIOE, GPIO_PIN_2, GPIOE, GPIO_PIN_3, TIM_CHANNEL_2, &htim3, 0, {0}, 0, 0},
    // M3: 左后轮 (PE4, PE5, TIM4_CH3, 编码器 TIM5 32位)
    {GPIOE, GPIO_PIN_4, GPIOE, GPIO_PIN_5, TIM_CHANNEL_3, &htim5, 0, {0}, 0, 0},
    // M4: 右后轮 (PE6, PE7, TIM4_CH4, 编码器 TIM8 16位)
    {GPIOE, GPIO_PIN_6, GPIOE, GPIO_PIN_7, TIM_CHANNEL_4, &htim8, 0, {0}, 0, 0}
};

//电机速度设置
void Motor_SetSpeed(Motor_t *motor, int speed)
{
	//速度限幅
    if(speed > 1000) speed = 1000;
    if(speed < -1000) speed = -1000;

    if(speed > 0)
    {
        HAL_GPIO_WritePin(motor->port_fwd, motor->pin_fwd, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->port_rev, motor->pin_rev, GPIO_PIN_RESET);
    }
    else if(speed < 0)
    {
        HAL_GPIO_WritePin(motor->port_fwd, motor->pin_fwd, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->port_rev, motor->pin_rev, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(motor->port_fwd, motor->pin_fwd, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->port_rev, motor->pin_rev, GPIO_PIN_RESET);
    }
    __HAL_TIM_SetCompare(&htim4, motor->pwm_channel, abs(speed));
}

int Motor_GetSpeed(Motor_t *motor)
{
    uint32_t current_count = __HAL_TIM_GET_COUNTER(motor->enc_htim);
    int speed = 0;
    //TIM3 TIM8是16位要用short
    if (motor->enc_htim->Instance == TIM3 || motor->enc_htim->Instance == TIM8)
    {
        speed = (short)(current_count - motor->last_enc_count);
    }
    else
    {
        speed = (int)(current_count - motor->last_enc_count);
    }
    motor->last_enc_count = current_count;
    return speed;
}

void Emergency_Stop()
{
    for(int i = 0; i < 4; i++)
    {
        HAL_GPIO_WritePin(Chassis_Motors[i].port_fwd, Chassis_Motors[i].pin_fwd, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Chassis_Motors[i].port_rev, Chassis_Motors[i].pin_rev, GPIO_PIN_RESET);
        __HAL_TIM_SetCompare(&htim4, Chassis_Motors[i].pwm_channel, 0);

        Chassis_Motors[i].pid.target = 0;
        Chassis_Motors[i].pid.error = 0;
        Chassis_Motors[i].pid.integral = 0;
    }
}
