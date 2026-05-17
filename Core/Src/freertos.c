/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "ps2.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "queue.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
PID_TypeDef PID_M1, PID_M2, PID_M3, PID_M4;
extern Motor_t Chassis_Motors[4];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern UART_HandleTypeDef huart1;
extern JOYSTICK_TypeDef ps2_pad;
extern TIM_HandleTypeDef htim4; // 声明定时器句�?????
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
QueueHandle_t ps2_dataHandle;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId Task_motorHandle;
osThreadId Task_ps2Handle;
//osMessageQId ps2_dataHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask_motor(void const * argument);
void StartTask_ps2(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of ps2_data */
	 /*osMessageQDef(ps2_data, 1, 20);
  ps2_dataHandle = osMessageCreate(osMessageQ(ps2_data), NULL);    */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  ps2_dataHandle = xQueueCreate(1, sizeof(CtrlCmd_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Task_motor */
  osThreadDef(Task_motor, StartTask_motor, osPriorityHigh, 0, 512);
  Task_motorHandle = osThreadCreate(osThread(Task_motor), NULL);

  /* definition and creation of Task_ps2 */
  osThreadDef(Task_ps2, StartTask_ps2, osPriorityNormal, 0, 128);
  Task_ps2Handle = osThreadCreate(osThread(Task_ps2), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask_motor */
/**
* @brief Function implementing the Task_motor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_motor */
void StartTask_motor(void const * argument)
{
  /* USER CODE BEGIN StartTask_motor */
	for(int i = 0; i < 4; i++)
	{
	      PID_Init(&(Chassis_Motors[i].pid), 3.0f, 0.2f, 0.0f);
	}
  CtrlCmd_t recv_cmd;
  /* Infinite loop */
for(;;)
{
	if(xQueueReceive(ps2_dataHandle, &recv_cmd, pdMS_TO_TICKS(200)) == pdPASS)
	{
		//printf("Queue OK! Valid: %d, Vy: %d,Vx: %d\r\n", recv_cmd.is_valid, (int)recv_cmd.vy,(int)recv_cmd.vx);
		if(recv_cmd.is_valid==1)
		{
			float vx = recv_cmd.vx * 7.0f;
			float vy = recv_cmd.vy * 7.0f;
			float omega = recv_cmd.omega * 7.0f;
			//合成每个轮子的目标速度
			Chassis_Motors[0].target_speed = (int)(vy + vx + omega);
			Chassis_Motors[1].target_speed = (int)(vy - vx - omega);
			Chassis_Motors[2].target_speed = (int)(vy + vx - omega);
			Chassis_Motors[3].target_speed = (int)(vy - vx + omega);

			for(int i = 0; i < 4; i++)
			{
			  Chassis_Motors[i].actual_speed = Motor_GetSpeed(&Chassis_Motors[i]);
			  int pwm = PID_Calc(&(Chassis_Motors[i].pid), Chassis_Motors[i].target_speed, Chassis_Motors[i].actual_speed);
			  Motor_SetSpeed(&Chassis_Motors[i], pwm);
			}
		}
		else
		{
			Emergency_Stop();
		}
	}
	else
	{
		Emergency_Stop();
	}
    osDelay(20);
  }
  /* USER CODE END StartTask_motor */
}

/* USER CODE BEGIN Header_StartTask_ps2 */
/**
* @brief Function implementing the Task_ps2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_ps2 */
void StartTask_ps2(void const * argument)
{
  /* USER CODE BEGIN StartTask_ps2 */
	CtrlCmd_t send_cmd = {0.0f, 0.0f, 0.0f, 0};
  /* Infinite loop */
  for(;;)
  {
      PS2_ScanKey(&ps2_pad);
      //printf("Joy: %d ,MODE:%d \r\n", ps2_pad.LJoy_UD,ps2_pad.mode);
      if(ps2_pad.mode==115)
      {
    	//遥杆死区设置
    	send_cmd.vx = (abs(ps2_pad.LJoy_LR - 128) < 10) ? 0 : (ps2_pad.LJoy_LR - 128);
    	send_cmd.vy = (abs(ps2_pad.LJoy_UD - 128) < 10) ? 0 : (128 - ps2_pad.LJoy_UD);
    	send_cmd.omega = (abs(ps2_pad.RJoy_LR - 128) < 10) ? 0 : (ps2_pad.RJoy_LR - 128);
    	send_cmd.is_valid = 1;
      }
      else
      {
    	  send_cmd.is_valid = 0;
    	  send_cmd.vx = 0;
    	  send_cmd.vy = 0;
    	  send_cmd.omega = 0;
      }
    xQueueOverwrite(ps2_dataHandle, &send_cmd);
    osDelay(50);
  }
  /* USER CODE END StartTask_ps2 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
