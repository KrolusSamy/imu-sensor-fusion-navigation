#include "HAL_GY91.h"

void HAL_GY91_Init() {
    Wire.begin();
    Wire.setClock(400000);
    
    // 1. Wake MPU9250 & Set Clock
    Wire.beginTransmission(MPU9250_I2C_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1
    Wire.write(0x01); 
    Wire.endTransmission();

    // 2. Open I2C Bypass Multiplexer for Magnetometer
    Wire.beginTransmission(MPU9250_I2C_ADDR);
    Wire.write(0x37); // INT_PIN_CFG
    Wire.write(0x02); 
    Wire.endTransmission();
    delay(10);
}

void HAL_GY91_ReadRaw(IMU_RawData_t *data) {
    // Burst read 14 bytes from accelerometer and gyroscope
    Wire.beginTransmission(MPU9250_I2C_ADDR);
    Wire.write(0x3B); // ACCEL_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU9250_I2C_ADDR, (uint8_t)14);

    data->ax = (Wire.read() << 8) | Wire.read();
    data->ay = (Wire.read() << 8) | Wire.read();
    data->az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // Skip Temperature
    data->gx = (Wire.read() << 8) | Wire.read();
    data->gy = (Wire.read() << 8) | Wire.read();
    data->gz = (Wire.read() << 8) | Wire.read();
}