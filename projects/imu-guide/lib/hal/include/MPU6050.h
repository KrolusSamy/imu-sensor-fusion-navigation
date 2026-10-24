/**
 * @file    MPU6050.h
 * @author  Krolus Hebiesh
 * @version 1.0
 * @date    2026-09-06
 * @brief   Hardware Abstraction Layer for the MPU6050. Contains full physical register map.
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

// === [ FULL MPU6050 REGISTER MAP ] ===
#define MPU6050_ADDR                0x68

#define REG_SELF_TEST_X             0x0D
#define REG_SELF_TEST_Y             0x0E
#define REG_SELF_TEST_Z             0x0F
#define REG_SELF_TEST_A             0x10
#define REG_SMPLRT_DIV              0x19 
#define REG_CONFIG                  0x1A 
#define REG_GYRO_CONFIG             0x1B 
#define REG_ACCEL_CONFIG            0x1C 
#define REG_FIFO_EN                 0x23
#define REG_I2C_MST_CTRL            0x24
#define REG_INT_PIN_CFG             0x37 
#define REG_INT_ENABLE              0x38 
#define REG_INT_STATUS              0x3A 
#define REG_ACCEL_XOUT_H            0x3B 
#define REG_ACCEL_XOUT_L            0x3C
#define REG_ACCEL_YOUT_H            0x3D
#define REG_ACCEL_YOUT_L            0x3E
#define REG_ACCEL_ZOUT_H            0x3F
#define REG_ACCEL_ZOUT_L            0x40
#define REG_TEMP_OUT_H              0x41
#define REG_TEMP_OUT_L              0x42
#define REG_GYRO_XOUT_H             0x43 
#define REG_GYRO_XOUT_L             0x44
#define REG_GYRO_YOUT_H             0x45
#define REG_GYRO_YOUT_L             0x46
#define REG_GYRO_ZOUT_H             0x47
#define REG_GYRO_ZOUT_L             0x48
#define REG_USER_CTRL               0x6A
#define REG_PWR_MGMT_1              0x6B 
#define REG_PWR_MGMT_2              0x6C
#define REG_FIFO_COUNTH             0x72
#define REG_FIFO_COUNTL             0x73
#define REG_FIFO_R_W                0x74
#define REG_WHO_AM_I                0x75

void MPU6050_Init();
void MPU6050_Clear_Interrupt();
bool MPU6050_Read_Burst(uint8_t* buffer, uint8_t length);

#endif // MPU6050_H