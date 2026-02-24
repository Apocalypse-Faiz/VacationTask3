#include "stm32f4xx_hal.h"
#include "IIC.h"

/* 引脚定义（可自行修改） */
#define GPIO_PORT_IIC     GPIOB
#define RCC_IIC_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE()
#define IIC_SCL_PIN       GPIO_PIN_6
#define IIC_SDA_PIN       GPIO_PIN_7

/* GPIO操作宏 */
#define IIC_SCL_1()  HAL_GPIO_WritePin(GPIO_PORT_IIC, IIC_SCL_PIN, GPIO_PIN_SET)
#define IIC_SCL_0()  HAL_GPIO_WritePin(GPIO_PORT_IIC, IIC_SCL_PIN, GPIO_PIN_RESET)
#define IIC_SDA_1()  HAL_GPIO_WritePin(GPIO_PORT_IIC, IIC_SDA_PIN, GPIO_PIN_SET)
#define IIC_SDA_0()  HAL_GPIO_WritePin(GPIO_PORT_IIC, IIC_SDA_PIN, GPIO_PIN_RESET)
#define IIC_SDA_READ()  HAL_GPIO_ReadPin(GPIO_PORT_IIC, IIC_SDA_PIN)

/* 延时函数 */
static void IIC_Delay(void)
{
    volatile uint32_t i;
    for (i = 0; i < IIC_DELAY_CYCLES; i++);
}

/* SDA方向控制 */
void IIC_SDA_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = IIC_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIO_PORT_IIC, &GPIO_InitStruct);
}

void IIC_SDA_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = IIC_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIO_PORT_IIC, &GPIO_InitStruct);
}

void IIC_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_IIC_ENABLE();

    GPIO_InitStruct.Pin = IIC_SCL_PIN | IIC_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIO_PORT_IIC, &GPIO_InitStruct);

    IIC_Stop(); // 复位总线
}

void IIC_Start(void)
{
    IIC_SDA_OUT();
    IIC_SDA_1();
    IIC_SCL_1();
    IIC_Delay();
    IIC_SDA_0();
    IIC_Delay();
    IIC_SCL_0();
    IIC_Delay();
}

void IIC_Stop(void)
{
    IIC_SDA_OUT();
    IIC_SDA_0();
    IIC_SCL_1();
    IIC_Delay();
    IIC_SDA_1();
    IIC_Delay();
}

void IIC_Send_Byte(uint8_t _ucByte)
{
    uint8_t i;
    IIC_SDA_OUT();
    IIC_SCL_0();
    for (i = 0; i < 8; i++)
    {
        if (_ucByte & 0x80)
            IIC_SDA_1();
        else
            IIC_SDA_0();
        _ucByte <<= 1;
        IIC_Delay();
        IIC_SCL_1();
        IIC_Delay();
        IIC_SCL_0();
        IIC_Delay();
    }
}

uint8_t IIC_Read_Byte(uint8_t ack)
{
    uint8_t i, value = 0;
    IIC_SDA_IN();
    for (i = 0; i < 8; i++)
    {
        value <<= 1;
        IIC_SCL_1();
        IIC_Delay();
        if (IIC_SDA_READ())
            value |= 1;
        IIC_SCL_0();
        IIC_Delay();
    }
    if (ack)
        IIC_Ack();
    else
        IIC_NAck();
    return value;
}

uint8_t IIC_Wait_Ack(void)
{
    uint8_t re;
    uint32_t timeout = 0;

    IIC_SDA_IN();
    IIC_SCL_1();
    IIC_Delay();
    while (IIC_SDA_READ())
    {
        timeout++;
        if (timeout > 250)
        {
            IIC_Stop();
            return 1;
        }
    }
    re = 0;
    IIC_SCL_0();
    IIC_Delay();
    return re;
}

void IIC_Ack(void)
{
    IIC_SDA_OUT();
    IIC_SDA_0();
    IIC_Delay();
    IIC_SCL_1();
    IIC_Delay();
    IIC_SCL_0();
    IIC_Delay();
    IIC_SDA_1();
}

void IIC_NAck(void)
{
    IIC_SDA_OUT();
    IIC_SDA_1();
    IIC_Delay();
    IIC_SCL_1();
    IIC_Delay();
    IIC_SCL_0();
    IIC_Delay();
}

uint8_t IIC_CheckDevice(uint8_t _Address)
{
    uint8_t ucAck;
    IIC_GPIO_Init();
    IIC_Start();
    IIC_Send_Byte(_Address | IIC_WR);
    ucAck = IIC_Wait_Ack();
    IIC_Stop();
    return ucAck;
}