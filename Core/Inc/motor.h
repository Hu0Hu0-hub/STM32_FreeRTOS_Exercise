#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "pid.h"
#include "tim.h"
#include "gpio.h"
#include "main.h"
#include "stdint.h"
#include "stdbool.h"
//手柄数据传输结构体
typedef struct
{
    float vx;          // 左右平移速度 (X轴)
    float vy;          // 前后前进速度 (Y轴)
    float omega;       // 原地旋转角速度 (Z轴)
    uint8_t  is_valid;    // 数据有效标志位 (手柄是否正常连接)
} CtrlCmd_t;

typedef struct
{
    GPIO_TypeDef* port_fwd;   // 正转引脚 Port
    uint16_t      pin_fwd;    // 正转引脚 Pin
    GPIO_TypeDef* port_rev;   // 反转引脚 Port
    uint16_t      pin_rev;    // 反转引脚 Pin
    uint32_t      pwm_channel; // TIM4 pwm的通道
    TIM_HandleTypeDef* enc_htim; // 绑定的编码器定时器 (htim2/3/5/8)
    uint32_t      last_enc_count; // 计数器记忆
    PID_TypeDef   pid;            // 电机专属PID

    int           actual_speed;
    int           target_speed;
} Motor_t;

void Motor_SetSpeed(Motor_t *motor, int speed);
void Emergency_Stop(void);
int Motor_GetSpeed(Motor_t *motor);
#endif /* INC_MOTOR_H_ */
