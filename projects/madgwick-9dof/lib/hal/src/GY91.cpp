#include "GY91.h"

// === [ HAL LAYER: I2C BUS & SILICON INIT ] ===
void GY91::init() {
    Wire.begin();
    // Hardware Throttling: Boost I2C to Fast Mode to minimize bus occupancy time.
    // Set a timeout to prevent infinite blocking if the bus hangs (SDA/SCL short).
    Wire.setClock(400000); 
    Wire.setWireTimeout(3000, true); 

    // 1. Wake up MPU6500 and select Auto PLL clock source for best stability
    writeRegister(MPU9250_ADDR, PWR_MGMT_1, 0x01);
    delay(100);

    // 2. Configure Digital Low Pass Filter (DLPF) to ~41Hz to eliminate high-freq vibration noise
    writeRegister(MPU9250_ADDR, CONFIG, 0x03);

    // 3. Set Sample Rate Divider to 4. Internal Sample Rate = 1kHz / (1 + 4) = 200 Hz
    writeRegister(MPU9250_ADDR, SMPLRT_DIV, 0x04);

    // 4. Set Gyro FSR to +/- 2000 dps, Accel FSR to +/- 8g
    writeRegister(MPU9250_ADDR, GYRO_CONFIG, 0x00);
    writeRegister(MPU9250_ADDR, ACCEL_CONFIG, 0x08);

    // 5. Enable I2C Bypass to allow the main MCU to talk directly to the AK8963 Mag
    writeRegister(MPU9250_ADDR, INT_PIN_CFG, 0x02);
    delay(50);

    // 6. Init AK8963: 16-bit resolution, 100Hz continuous data output mode
    writeRegister(AK8963_ADDR, AK8963_CNTL, 0x16);
    delay(50);
}

// === [ HAL LAYER: I2C BURST EXTRACTION ] ===
void GY91::readRawAccelGyro(int16_t* accel, int16_t* gyro) {
    // Atomic burst read of 14 bytes (Accel X/Y/Z, Temp, Gyro X/Y/Z)
    // Prevents data tearing where high and low bytes belong to different sampling epochs.
    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU9250_ADDR, (uint8_t)14, (uint8_t)true);

    if (Wire.available() == 14) {
        accel[0] = (Wire.read() << 8) | Wire.read(); 
        accel[1] = (Wire.read() << 8) | Wire.read();
        accel[2] = (Wire.read() << 8) | Wire.read();
        Wire.read(); Wire.read(); // Discard Temperature
        gyro[0]  = (Wire.read() << 8) | Wire.read();
        gyro[1]  = (Wire.read() << 8) | Wire.read();
        gyro[2]  = (Wire.read() << 8) | Wire.read();
    }
}

void GY91::readRawMag(int16_t* mag) {
    Wire.beginTransmission(AK8963_ADDR);
    Wire.write(AK8963_ST1); // Read from ST1 to trigger data readiness
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)AK8963_ADDR, (uint8_t)1, (uint8_t)true);
    
    // Check if Data Ready bit is set
    if (Wire.read() & 0x01) {
        Wire.beginTransmission(AK8963_ADDR);
        Wire.write(AK8963_XOUT_L);
        Wire.endTransmission(false);
        // Read 7 bytes: Mag X/Y/Z (Little Endian) + ST2 (mandatory to release data lock)
        Wire.requestFrom((uint8_t)AK8963_ADDR, (uint8_t)7, (uint8_t)true);
        
        if (Wire.available() == 7) {
            mag[0] = Wire.read() | (Wire.read() << 8);
            mag[1] = Wire.read() | (Wire.read() << 8);
            mag[2] = Wire.read() | (Wire.read() << 8);
            Wire.read(); // Discard ST2
        }
    }
}

void GY91::writeRegister(uint8_t deviceAddr, uint8_t regAddr, uint8_t data) {
    Wire.beginTransmission(deviceAddr);
    Wire.write(regAddr);
    Wire.write(data);
    Wire.endTransmission();
}