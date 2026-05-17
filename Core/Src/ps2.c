#include "ps2.h"

// 将原本的位带操作替换为 HAL 库宏定义[cite: 12, 15]
#define DI()     HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0)

#define CMD_H()  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET)
#define CMD_L()  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET)

#define CS_H()   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_SET)
#define CS_L()   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET)

#define CLK_H()  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_SET)
#define CLK_L()  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET)

// 请求获取数据命令[cite: 12]
const uint8_t PS2_cmnd[9] = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t PS2_data[9] = {0};

// F407 高主频(168MHz)下的软件微秒延时 (替代原来的 ax_delay[cite: 18, 19])
static void Delay_us(uint32_t us)
{
    // 简单的空指令循环，大约延时 us 微秒即可应付 SPI 通信
    uint32_t delay = 28 * us;
    while(delay--) {
        __NOP();
    }
}

// PS2 软件模拟 SPI 读写底层[cite: 12]
static uint8_t PS2_ReadWriteData(uint8_t data)
{
    uint8_t ref, res = 0;
    for(ref = 0x01; ref > 0x00; ref <<= 1) {
        CLK_L();
        if(ref & data) CMD_H();
        else           CMD_L();
        Delay_us(16);

        CLK_H();
        if(DI()) res |= ref;
        Delay_us(16);
    }
    CMD_H();
    return res;
}

// 扫描手柄数据[cite: 12]
void PS2_ScanKey(JOYSTICK_TypeDef *JoystickStruct)
{
    uint8_t i;
    CS_L(); // 选中手柄

    for(i=0; i<9; i++) {
        PS2_data[i] = PS2_ReadWriteData(PS2_cmnd[i]);
        Delay_us(16);
    }
    CS_H(); // 释放手柄

    // 数据打包[cite: 12]
    JoystickStruct->mode    = PS2_data[1];
    JoystickStruct->btn1    = ~PS2_data[3];
    JoystickStruct->btn2    = ~PS2_data[4];
    JoystickStruct->RJoy_LR = PS2_data[5];
    JoystickStruct->RJoy_UD = PS2_data[6];
    JoystickStruct->LJoy_LR = PS2_data[7];
    JoystickStruct->LJoy_UD = PS2_data[8];
}
