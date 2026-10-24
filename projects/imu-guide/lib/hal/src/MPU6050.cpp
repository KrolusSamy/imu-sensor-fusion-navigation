/**
 * @file    MPU6050.cpp
 * @author  Krolus Hebiesh
 * @brief   Executes physical silicon configurations and raw I2C burst reads.
 */

#include "MPU6050.h"
#include <Wire.h>
#include <Arduino.h>

void MPU6050_Init() {
    // 1. Hard Silicon Reset: Flushes internal registers to default state
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x80); 
    Wire.endTransmission();
    delay(100); 

    // 2. Wake & PLL Lock: Aligns the internal clock to the X-Axis Gyro for maximum stability
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x01); 
    Wire.endTransmission();

    // 3. Digital Low Pass Filter: Sets DLPF_CFG to 3 (44Hz Accel / 42Hz Gyro), internal clock drops to 1kHz
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(REG_CONFIG);
    Wire.write(0x03); 
    Wire.endTransmission();
    
    // 4. Kinematic Scaling: Configures hardware amplifiers for ±4g Accel, ±500°/s Gyro
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(REG_ACCEL_CONFIG);
    Wire.write(0x08); 
    Wire.endTransmission();

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(REG_GYRO_CONFIG);
    Wire.write(0x08); 
    Wire.endTransmission();

    // 5. Hardware Decimation: Throttle output rate at the silicon level (1000Hz / (1 + 19) = 50Hz)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(REG_SMPLRT_DIV);
    Wire.write(0x13); 
    Wire.endTransmission();

    // 6. Interrupt Latching: Forces INT pin to stay HIGH until cleared, preventing 50µs missed edge deadlocks
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(REG_INT_PIN_CFG);
    Wire.write(0x20); 
    Wire.endTransmission();

    // 7. Interrupt Enable: Activates the DATA_RDY_EN bit to fire the physical pin
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(REG_INT_ENABLE);
    Wire.write(0x01); 
    Wire.endTransmission();
}

void MPU6050_Clear_Interrupt() {
    // Acknowledge interrupt to drop INT pin LOW by reading the status register
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(REG_INT_STATUS);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1);
    Wire.read();
}

bool MPU6050_Read_Burst(uint8_t* buffer, uint8_t length) {
    // Single sequential burst read prevents data tearing mid-packet
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(REG_ACCEL_XOUT_H); 
    Wire.endTransmission(false);
    
    Wire.requestFrom((uint8_t)MPU6050_ADDR, length); 
    if (Wire.available() == length) {
        for (uint8_t i = 0; i < length; i++) {
            buffer[i] = Wire.read();
        }
        return true;
    }
    return false;
}