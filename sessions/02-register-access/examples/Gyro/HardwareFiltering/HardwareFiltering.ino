/**
 * @file    Phase4_01_GyroNoise_Integration.ino
 * @author  Krolus Hebiesh
 * @version 1.0
 * @date    2026-08-30
 * @brief   Extracts raw angular rate (DLPF OFF, ±250dps) and integrates to show drift.
 */

#include <Wire.h>

// === [ 1. SYSTEM CONFIGURATION ] ===
#define DEBUG_MODE_ACTIVATE 1
const uint8_t MPU_INT_PIN = 2; 

// === [ 2. HARDWARE ABSTRACTION LAYER (HAL) ] ===
const uint8_t MPU_ADDR = 0x68;
const uint8_t REG_SMPLRT_DIV = 0x19;   // Register 25
const uint8_t REG_CONFIG = 0x1A;       // Register 26
const uint8_t REG_GYRO_CONFIG = 0x1B;  // Register 27
const uint8_t REG_INT_PIN_CFG = 0x37;  // Register 55
const uint8_t REG_INT_ENABLE = 0x38;   // Register 56
const uint8_t REG_INT_STATUS = 0x3A;   // Register 58
const uint8_t REG_GYRO_XOUT_H = 0x43;  // Register 67: Start of Gyro data
const uint8_t REG_SIGNAL_PATH_RESET = 0x68; // Register 104
const uint8_t REG_PWR_MGMT_1 = 0x6B;   // Register 107

// === [ 3. MIDDLEWARE ] ===
volatile bool data_ready_flag = false;
int16_t raw_gx, raw_gy, raw_gz;

float rate_z = 0.0f;
float angle_z = 0.0f;
unsigned long last_time_micros = 0;

// === [ 4. APPLICATION ] ===
void configure_aerospace_timing() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x80); // DEVICE_RESET
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_SIGNAL_PATH_RESET);
    Wire.write(0x07); // Resets analog and digital signal paths
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x01); // Lock PLL to X-axis gyro
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_CONFIG);
    Wire.write(0x06); // DLPF ENABLED: 5Hz Bandwidth, 18.6ms physical phase delay, 1kHz clock
    Wire.endTransmission();
    
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_GYRO_CONFIG);
    Wire.write(0x00); // Scale = ±250°/s (Sensitivity: 131 LSB/°/s)
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_SMPLRT_DIV);
    Wire.write(0x13); // Divider = 19. Output Rate: 1000Hz / (1+19) = 50Hz
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_INT_PIN_CFG);
    Wire.write(0x20); // LATCH_INT_EN = 1
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_INT_ENABLE);
    Wire.write(0x01); // DATA_RDY_EN = 1
    Wire.endTransmission();
}

void mpu_data_ready_isr() {
    data_ready_flag = true;
}

void fetch_gyroscope() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_INT_STATUS);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)1);
    Wire.read(); // Drop INT pin LOW

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_GYRO_XOUT_H); 
    Wire.endTransmission(false);
    
    Wire.requestFrom(MPU_ADDR, (uint8_t)6); 
    if (Wire.available() == 6) {
        raw_gx = (Wire.read() << 8) | Wire.read();
        raw_gy = (Wire.read() << 8) | Wire.read();
        raw_gz = (Wire.read() << 8) | Wire.read();
    }
}

void setup() {
#if DEBUG_MODE_ACTIVATE
    Serial.begin(115200);
#endif
    Wire.begin(); 
    Wire.setClock(400000); 
    Wire.setWireTimeout(25000, true);
    
    pinMode(MPU_INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), mpu_data_ready_isr, RISING);
    
    configure_aerospace_timing();
    last_time_micros = micros();
}

void loop() {
    if (data_ready_flag) {
        data_ready_flag = false; 
        Wire.clearWireTimeoutFlag();
        
        fetch_gyroscope();

        // High-precision timestep calculation
        unsigned long current_micros = micros();
        float dt = (current_micros - last_time_micros) / 1000000.0f;
        last_time_micros = current_micros;

        // Kinematic Conversion: 131 LSB/°/s mapping
        rate_z = raw_gz / 131.0f;
        
        // Mathematical Integration (Produces the physical drift)
        angle_z += rate_z * dt;
        
#if DEBUG_MODE_ACTIVATE
        // Plot both the raw rate (should hug 0) and the integrating angle (will drift away)
        Serial.print("RateZ:"); Serial.print(rate_z); Serial.print(",");
        Serial.print("AngleZ:"); Serial.println(angle_z);
#endif
    }
}