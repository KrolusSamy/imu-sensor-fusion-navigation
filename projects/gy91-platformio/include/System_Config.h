/**
 * @file    System_Config.h
 * @author  Krolus Hebiesh
 * @brief   Global macros, I2C addresses, and tuning parameters.
 */
#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#define DEBUG_MODE_ACTIVATE 1
#define TELEMETRY_RATE_MS   30

// --- Hardware Addresses ---
#define MPU9250_I2C_ADDR 0x68
#define AK8963_I2C_ADDR  0x0C
#define BMP280_I2C_ADDR  0x76

// --- Math & Tuning ---
#define GYRO_SENSITIVITY 131.0f
#define DT               0.03f

#endif