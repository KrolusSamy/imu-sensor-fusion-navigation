/**
 * @file    GY91.h
 * @author  Krolus Hebiesh
 * @version 1.0
 * @date    2026-09-07
 * @brief   Hardware Abstraction Layer for the GY-91 (MPU-9255 IMU + AK8963 Magnetometer).
 */

#ifndef GY91_H
#define GY91_H

#include <stdint.h>

// === [ MPU-9255 REGISTER MAP ] ===
#define MPU9255_ADDR                0x68
#define AK8963_ADDR                 0x0C // Secondary I2C address via bypass

#define REG_GY91_SMPLRT_DIV         0x19 
#define REG_GY91_CONFIG             0x1A 
#define REG_GY91_GYRO_CONFIG        0x1B 
#define REG_GY91_ACCEL_CONFIG       0x1C 
#define REG_GY91_INT_PIN_CFG        0x37 // Controls I2C Bypass Multiplexer
#define REG_GY91_INT_ENABLE         0x38 
#define REG_GY91_INT_STATUS         0x3A 
#define REG_GY91_ACCEL_XOUT_H       0x3B 
#define REG_GY91_GYRO_XOUT_H        0x43 
#define REG_GY91_USER_CTRL          0x6A
#define REG_GY91_PWR_MGMT_1         0x6B 
#define REG_GY91_WHO_AM_I           0x75 // Returns 0x71

// === [ AK8963 MAGNETOMETER REGISTER MAP ] ===
#define AK8963_WHO_AM_I             0x00 // Returns 0x48
#define AK8963_ST1                  0x02 // Data status 1
#define AK8963_HXL                  0x03 // Magnetometer data output X, Y, Z
#define AK8963_CNTL1                0x0A // Control register 1 (Mode setting)

void GY91_Init();
void GY91_Clear_Interrupt();
bool GY91_Read_Burst(uint8_t* buffer, uint8_t length);
bool GY91_Read_Mag_Burst(uint8_t* buffer);

#endif // GY91_H