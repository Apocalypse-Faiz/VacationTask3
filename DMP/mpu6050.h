#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

// 错误码定义
#define ERR_MPU_INIT            -1
#define ERR_SET_SENSOR          -2
#define ERR_CONFIG_FIFO         -3
#define ERR_SET_RATE            -4
#define ERR_LOAD_DMP            -5
#define ERR_SET_ORIENT          -6
#define ERR_ENABLE_FEATURE      -7
#define ERR_SET_FIFO_RATE       -8
#define ERR_SELF_TEST           -9
#define ERR_DMP_STATE           -10

// 默认采样率（已由 inv_mpu.h 定义，此处不再重复）
// #define DEFAULT_MPU_HZ  100

#define Q30  1073741824.0f
#define MPU_ADDR 0x68

// 量程宏（与 inv_mpu.h 一致）
#define GYRO_FSR_250DPS  250
#define GYRO_FSR_500DPS  500
#define GYRO_FSR_1000DPS 1000
#define GYRO_FSR_2000DPS 2000

#define ACCEL_FSR_2G  2
#define ACCEL_FSR_4G  4
#define ACCEL_FSR_8G  8
#define ACCEL_FSR_16G 16

#define LPF_5HZ   5
#define LPF_10HZ  10
#define LPF_20HZ  20
#define LPF_42HZ  42
#define LPF_98HZ  98
#define LPF_188HZ 188

// MPU6050 配置结构体
typedef struct {
    uint8_t sensors;          // 使能的传感器 (INV_XYZ_GYRO | INV_XYZ_ACCEL)
    uint8_t fifo_config;      // FIFO 传感器配置
    uint16_t sample_rate;     // 采样率 (Hz)
    uint8_t accel_fsr;        // 加速度计量程 (2/4/8/16)
    uint16_t gyro_fsr;        // 陀螺仪量程 (250/500/1000/2000)
    uint8_t lpf;              // 低通滤波器频率 (Hz)
    int8_t gyro_orientation[9]; // 陀螺仪方向矩阵
} MPU6050_Config_t;

// DMP 配置结构体
typedef struct {
    uint8_t dmp_on;           // DMP 使能标志
    uint16_t dmp_feature;     // DMP 功能掩码 (DMP_FEATURE_xxx)
    uint8_t interrupt_mode;   // 中断模式 (DMP_INT_CONTINUOUS / DMP_INT_GESTURE)
    uint16_t fifo_rate;       // DMP FIFO 输出速率 (Hz)
} DMP_Config_t;

// 数据接收结构体
typedef struct {
    float gyro[3];            // 陀螺仪原始数据 (dps)
    float accel[3];           // 加速度计原始数据 (g)
    float pitch;              // 俯仰角 (度)
    float roll;               // 横滚角 (度)
    float yaw;                // 航向角 (度)
    long quat[4];             // 四元数 (q30格式)
    uint32_t timestamp;       // 时间戳 (ms)
    uint16_t sensors;         // 有效传感器掩码
    uint8_t more;             // 剩余包数
} MPU6050_Data_t;

// 函数声明
int MPU6050_Init(MPU6050_Config_t *mpu_cfg, DMP_Config_t *dmp_cfg);
void MPU6050_SelfTest(uint8_t apply_accel_bias);
int MPU6050_ReadDMP(MPU6050_Data_t *data);
int MPU6050_ReadFIFO(MPU6050_Data_t *data);

// 底层 I2C 接口（供 inv_mpu.c 调用）
uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
uint8_t MPU_Write_Byte(uint8_t reg, uint8_t data);
uint8_t MPU_Read_Byte(uint8_t reg);

extern MPU6050_Config_t mpu6050_cfg;
extern DMP_Config_t dmp_cfg;

#endif