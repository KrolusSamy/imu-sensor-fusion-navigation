/**
 * @file    GY91.h
 * @author  Krolus Hebiesh
 * @version 1.0
 * @date    2026-09-07
 * @brief   Hardware Abstraction Layer for GY-91 (MPU9250: MPU6500 + AK8963).
 *          Handles 400kHz I2C configuration, hardware DLPF, and burst extractions.
 */

#ifndef GY91_H
#define GY91_H

#include <Arduino.h>
#include <Wire.h>

// === [ HAL LAYER: SILICON REGISTERS ] ===
#define MPU9250_ADDR     0x68
#define AK8963_ADDR      0x0C

#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define INT_PIN_CFG      0x37
#define ACCEL_XOUT_H     0x3B
#define PWR_MGMT_1       0x6B

#define AK8963_ST1       0x02
#define AK8963_XOUT_L    0x03
#define AK8963_CNTL      0x0A

class GY91 {
public:
    void init();
    void readRawAccelGyro(int16_t* accel, int16_t* gyro);
    void readRawMag(int16_t* mag);
private:
    void writeRegister(uint8_t deviceAddr, uint8_t regAddr, uint8_t data);
};

#endif // GY91_H