#ifndef _IIC_H
#define _IIC_H

#include <inttypes.h>
#include "stm32f4xx_hal.h"

#define IIC_WR  0
#define IIC_RD  1

// 可调延时宏（主频168MHz时，约1us/循环）
#ifndef IIC_DELAY_CYCLES
#define IIC_DELAY_CYCLES  20
#endif

// SDA方向控制
void IIC_SDA_IN(void);
void IIC_SDA_OUT(void);

// 基础时序函数
void IIC_Start(void);
void IIC_Stop(void);
void IIC_Send_Byte(uint8_t _ucByte);
uint8_t IIC_Read_Byte(uint8_t ack);
uint8_t IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);

// 设备级读写
uint8_t IIC_CheckDevice(uint8_t _Address);
void IIC_GPIO_Init(void);

#endif