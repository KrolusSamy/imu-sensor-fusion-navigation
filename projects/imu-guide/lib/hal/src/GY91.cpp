/**
 * @file    GY91.cpp
 * @author  Krolus Hebiesh
 * @brief   Executes MPU-9250 silicon setup, enables AK8963 I2C bypass, and runs burst reads.
 */

#include "GY91.h"
#include <Wire.h>
#include <Arduino.h>

void GY91_Init() {
    // 1. Hard Silicon Reset
    Wire.beginTransmission(MPU9255_ADDR);
    Wire.write(REG_GY91_PWR_MGMT_1);
    Wire.write(0x80); 
    Wire.endTransmission();
    delay(100); 

    // 2. Wake & PLL Lock
    Wire.beginTransmission(MPU9255_ADDR);
    Wire.write(REG_GY91_PWR_MGMT_1);
    Wire.write(0x01); 
    Wire.endTransmission();

    // 3. Digital Low Pass Filter (DLPF)
    Wire.beginTransmission(MPU9255_ADDR);
    Wire.write(REG_GY91_CONFIG);
    Wire.write(0x03); 
    Wire.endTransmission();
    
    // 4. Kinematic Scaling (±4g Accel, ±500°/s Gyro)
    Wire.beginTransmission(MPU9255_ADDR);
    Wire.write(REG_GY91_ACCEL_CONFIG);
    Wire.write(0x08); 
    Wire.endTransmission();

    Wire.beginTransmission(MPU9255_ADDR);
    Wire.write(REG_GY91_GYRO_CONFIG);
    Wire.write(0x08); 
    Wire.endTransmission();

    // 5. Enable I2C Master Bypass so MCU can talk directly to the AK8963 Compass
    Wire.beginTransmission(MPU9255_ADDR);
    Wire.write(REG_GY91_USER_CTRL);
    Wire.write(0x00); // Disable internal I2C Master mode
    Wire.endTransmission();
    delay(10);

    Wire.beginTransmission(MPU9255_ADDR);
    Wire.write(REG_GY91_INT_PIN_CFG);
    Wire.write(0x02); // Set BYPASS_EN bit (Bit 1)
    Wire.endTransmission();
    delay(10);

    // 6. Initialize AK8963 Magnetometer (Continuous Measurement Mode 2: 100Hz)
    Wire.beginTransmission(AK8963_ADDR);
    Wire.write(AK8963_CNTL1);
    Wire.write(0x16); 
    Wire.endTransmission();
    delay(10);
}

void GY91_Clear_Interrupt() {
    Wire.beginTransmission(MPU9255_ADDR);
    Wire.write(REG_GY91_INT_STATUS);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU9255_ADDR, (uint8_t)1);
    Wire.read();
}

bool GY91_Read_Burst(uint8_t* buffer, uint8_t length) {
    Wire.beginTransmission(MPU9255_ADDR);
    Wire.write(REG_GY91_ACCEL_XOUT_H); 
    Wire.endTransmission(false);
    
    Wire.requestFrom((uint8_t)MPU9255_ADDR, length); 
    if (Wire.available() == length) {
        for (uint8_t i = 0; i < length; i++) {
            buffer[i] = Wire.read();
        }
        return true;
    }
    return false;
}

bool GY91_Read_Mag_Burst(uint8_t* buffer) {
    // Read 7 bytes starting from AK8963 data output (X, Y, Z + Status 2)
    Wire.beginTransmission(AK8963_ADDR);
    Wire.write(AK8963_HXL);
    Wire.endTransmission(false);
    
    Wire.requestFrom((uint8_t)AK8963_ADDR, (uint8_t)7);
    if (Wire.available() == 7) {
        for (uint8_t i = 0; i < 7; i++) {
            buffer[i] = Wire.read();
        }
        return true;
    }
    return false;
}