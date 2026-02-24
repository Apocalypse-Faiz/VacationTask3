#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "IIC.h"
#include <math.h>
#include <string.h> 

// 默认配置（可外部修改）
MPU6050_Config_t mpu6050_cfg = {
    .sensors = INV_XYZ_GYRO | INV_XYZ_ACCEL,
    .fifo_config = INV_XYZ_GYRO | INV_XYZ_ACCEL,
    .sample_rate = 100,
    .accel_fsr = ACCEL_FSR_2G,
    .gyro_fsr = GYRO_FSR_2000DPS,
    .lpf = LPF_42HZ,
    .gyro_orientation = {1, 0, 0, 0, 1, 0, 0, 0, 1}
};

DMP_Config_t dmp_cfg = {
    .dmp_on = 1,
    .dmp_feature = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL |
                   DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO,
    .interrupt_mode = DMP_INT_CONTINUOUS,
    .fifo_rate = 200
};

// 底层 I2C 函数实现（供 inv_mpu.c 使用）
uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    uint8_t i;
    IIC_Start();
    IIC_Send_Byte((addr << 1) | 0);
    if (IIC_Wait_Ack()) { IIC_Stop(); return 1; }
    IIC_Send_Byte(reg);
    if (IIC_Wait_Ack()) { IIC_Stop(); return 1; }
    for (i = 0; i < len; i++) {
        IIC_Send_Byte(buf[i]);
        if (IIC_Wait_Ack()) { IIC_Stop(); return 1; }
    }
    IIC_Stop();
    return 0;
}

uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    IIC_Start();
    IIC_Send_Byte((addr << 1) | 0);
    if (IIC_Wait_Ack()) { IIC_Stop(); return 1; }
    IIC_Send_Byte(reg);
    if (IIC_Wait_Ack()) { IIC_Stop(); return 1; }
    IIC_Start();
    IIC_Send_Byte((addr << 1) | 1);
    if (IIC_Wait_Ack()) { IIC_Stop(); return 1; }
    while (len--) {
        *buf++ = IIC_Read_Byte(len ? 1 : 0);
    }
    IIC_Stop();
    return 0;
}

uint8_t MPU_Write_Byte(uint8_t reg, uint8_t data)
{
    return MPU_Write_Len(MPU_ADDR, reg, 1, &data);
}

uint8_t MPU_Read_Byte(uint8_t reg)
{
    uint8_t data;
    MPU_Read_Len(MPU_ADDR, reg, 1, &data);
    return data;
}

// 初始化函数
int MPU6050_Init(MPU6050_Config_t *mpu_cfg, DMP_Config_t *dmp_cfg)
{
    int ret;

    // 初始化 I2C 总线
    IIC_GPIO_Init();

    // 复位 MPU，若失败则重启
    ret = mpu_init();
    if (ret != 0) {
        HAL_Delay(100);
        NVIC_SystemReset();  // 复位 MCU
    }

    // 配置传感器
    ret = mpu_set_sensors(mpu_cfg->sensors);
    if (ret) return ERR_SET_SENSOR;

    ret = mpu_configure_fifo(mpu_cfg->fifo_config);
    if (ret) return ERR_CONFIG_FIFO;

    ret = mpu_set_sample_rate(mpu_cfg->sample_rate);
    if (ret) return ERR_SET_RATE;

    ret = mpu_set_accel_fsr(mpu_cfg->accel_fsr);
    if (ret) return ERR_SET_RATE;

    ret = mpu_set_gyro_fsr(mpu_cfg->gyro_fsr);
    if (ret) return ERR_SET_RATE;

    if (mpu_cfg->lpf) {
        ret = mpu_set_lpf(mpu_cfg->lpf);
        if (ret) return ERR_SET_RATE;
    }

    // 如果不需要 DMP，直接返回
    if (!dmp_cfg->dmp_on) return 0;

    // 加载 DMP 固件
    ret = dmp_load_motion_driver_firmware();
    if (ret) return ERR_LOAD_DMP;

    // 设置方向
    ret = dmp_set_orientation(inv_orientation_matrix_to_scalar(mpu_cfg->gyro_orientation));
    if (ret) return ERR_SET_ORIENT;

    // 使能 DMP 功能
    ret = dmp_enable_feature(dmp_cfg->dmp_feature);
    if (ret) return ERR_ENABLE_FEATURE;

    // 设置 DMP FIFO 速率
    if (dmp_cfg->fifo_rate && dmp_cfg->fifo_rate <= 200) {
        ret = dmp_set_fifo_rate(dmp_cfg->fifo_rate);
        if (ret) return ERR_SET_FIFO_RATE;
    }

    // 设置中断模式
    if (dmp_cfg->interrupt_mode) {
        mpu_set_int_level(1);
        ret = dmp_set_interrupt_mode(dmp_cfg->interrupt_mode);
        if (ret) return ERR_SET_FIFO_RATE;
    }

    // 使能 DMP
    ret = mpu_set_dmp_state(1);
    if (ret) return ERR_DMP_STATE;

    return 0;
}

// 自检并设置偏置
void MPU6050_SelfTest(uint8_t apply_accel_bias)
{
    long gyro[3], accel[3];
    uint8_t i;

    if (mpu_run_self_test(gyro, accel) != 0x03) return; // 期望 gyro+accel 通过

    // 将偏置转换为合适的格式并写入
    for (i = 0; i < 3; i++) {
        gyro[i] = (long)(gyro[i] * 32.8f);      // 转换为 ±1000dps 格式
        accel[i] = (long)(accel[i] * 2048.f);   // 转换为 ±16g 格式
    }

    // mpu_set_gyro_bias_reg(gyro);
    // if (apply_accel_bias) {
    //     mpu_set_accel_bias_6050_reg(accel);
    // }
}

// 读取 DMP 数据
int MPU6050_ReadDMP(MPU6050_Data_t *data)
{
    short gyro_raw[3], accel_raw[3];
    long quat[4];
    unsigned long timestamp;
    short sensors;
    unsigned char more;
    float q0, q1, q2, q3;

    if (dmp_read_fifo(gyro_raw, accel_raw, quat, &timestamp, &sensors, &more))
        return -1;

    // 转换为物理单位
    for (int i = 0; i < 3; i++) {
        data->gyro[i] = gyro_raw[i] / 32768.0f * mpu6050_cfg.gyro_fsr;
        data->accel[i] = accel_raw[i] / 32768.0f * mpu6050_cfg.accel_fsr;
    }

    if (sensors & INV_WXYZ_QUAT) {
        q0 = (float)quat[0] / (float)(1<<30);
        q1 = (float)quat[1] / (float)(1<<30);
        q2 = (float)quat[2] / (float)(1<<30);
        q3 = (float)quat[3] / (float)(1<<30);

        data->pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.29578f;
        data->roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.29578f;
        data->yaw   = atan2(2 * (q0 * q3 + q1 * q2), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.29578f;
    }

    data->timestamp = timestamp;
    data->sensors = sensors;
    data->more = more;
    memcpy(data->quat, quat, sizeof(quat));

    return 0;
}

// 读取普通 FIFO 数据（无 DMP）
int MPU6050_ReadFIFO(MPU6050_Data_t *data)
{
    short gyro_raw[3], accel_raw[3];
    unsigned long timestamp;
    unsigned char sensors, more;

    if (mpu_read_fifo(gyro_raw, accel_raw, &timestamp, &sensors, &more))
        return -1;

    for (int i = 0; i < 3; i++) {
        data->gyro[i] = gyro_raw[i] / 32768.0f * mpu6050_cfg.gyro_fsr;
        data->accel[i] = accel_raw[i] / 32768.0f * mpu6050_cfg.accel_fsr;
    }

    data->timestamp = timestamp;
    data->sensors = sensors;
    data->more = more;

    return 0;
}