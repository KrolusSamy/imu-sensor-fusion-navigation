/**
 * @file    Phase3_01_RawNoise.ino
 * @author  Krolus Hebiesh
 * @version 1.0
 * @brief   Extracts raw unfiltered kinematics (DLPF OFF, 260Hz Bandwidth, ±2g).
 */

#include <Wire.h>

// === [ 1. SYSTEM CONFIGURATION ] ===
#define DEBUG_MODE_ACTIVATE 1
const uint8_t MPU_INT_PIN = 2; 

// === [ 2. HARDWARE ABSTRACTION LAYER (HAL) ] ===
const uint8_t MPU_ADDR = 0x68;
const uint8_t REG_SMPLRT_DIV = 0x19;   // Register 25
const uint8_t REG_CONFIG = 0x1A;       // Register 26
const uint8_t REG_ACCEL_CONFIG = 0x1C; // Register 28
const uint8_t REG_INT_PIN_CFG = 0x37;  // Register 55
const uint8_t REG_INT_ENABLE = 0x38;   // Register 56
const uint8_t REG_INT_STATUS = 0x3A;   // Register 58
const uint8_t REG_ACCEL_XOUT_H = 0x3B; // Register 59
const uint8_t REG_SIGNAL_PATH_RESET = 0x68; // Register 104
const uint8_t REG_PWR_MGMT_1 = 0x6B;   // Register 107

// === [ 3. MIDDLEWARE ] ===
volatile bool data_ready_flag = false;
int16_t raw_ax, raw_ay, raw_az;

// === [ 4. APPLICATION ] ===
void configure_aerospace_timing() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x80); // DEVICE_RESET = 1
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_SIGNAL_PATH_RESET);
    Wire.write(0x07); // GYRO_RESET, ACCEL_RESET, TEMP_RESET = 1
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x01); // Wake & PLL lock to X gyro
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_CONFIG);
    Wire.write(0x06); // DLPF ENABLED: 5Hz Bandwidth, internal clock drops to 1kHz
    Wire.endTransmission();
    
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_CONFIG);
    Wire.write(0x18); // Scale = ±16g (Sensitivity scales down to 2048 LSB/g)
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

void fetch_acceleration() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_XOUT_H); 
    Wire.endTransmission(false);
    
    Wire.requestFrom(MPU_ADDR, (uint8_t)6); 
    if (Wire.available() == 6) {
        raw_ax = (Wire.read() << 8) | Wire.read();
        raw_ay = (Wire.read() << 8) | Wire.read();
        raw_az = (Wire.read() << 8) | Wire.read();
    }
}

void clear_interrupt_flag() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_INT_STATUS);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)1);
    Wire.read(); // Reading clears the INT pin
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
}

void loop() {
    if (data_ready_flag) {
        data_ready_flag = false; 
        clear_interrupt_flag();
        fetch_acceleration();
        
#if DEBUG_MODE_ACTIVATE
        Serial.print(raw_ax); Serial.print(",");
        Serial.print(raw_ay); Serial.print(",");
        Serial.println(raw_az);
#endif
    }
}