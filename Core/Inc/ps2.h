#ifndef __PS2_H
#define __PS2_H

#include "main.h"

// 沿用原厂的手柄结构体定义[cite: 13]
typedef struct
{
  uint8_t mode;
  uint8_t btn1;
  uint8_t btn2;
  uint8_t RJoy_LR;
  uint8_t RJoy_UD;
  uint8_t LJoy_LR;
  uint8_t LJoy_UD;
} JOYSTICK_TypeDef;

void PS2_ScanKey(JOYSTICK_TypeDef* JoystickStruct);

#endif
