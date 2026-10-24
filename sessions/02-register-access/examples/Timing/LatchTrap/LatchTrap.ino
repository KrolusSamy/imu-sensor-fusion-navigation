/**
 * @file    Phase5_00_TheLatchTrap.ino
 * @author  Krolus Hebiesh
 * @version 1.0
 * @date    2026-08-30
 * @brief   Intentionally deadlocks the MCU by failing to clear a latched hardware interrupt.
 */

#include <Wire.h>

// === [ 1. SYSTEM CONFIGURATION ] ===
#define DEBUG_MODE_ACTIVATE 1
const uint8_t MPU_INT_PIN = 2; 

// === [ 2. HARDWARE ABSTRACTION LAYER (HAL) ] ===
const uint8_t MPU_ADDR = 0x68;
const uint8_t REG_INT_PIN_CFG = 0x37;  // Register 55: Latch behavior
const uint8_t REG_INT_ENABLE = 0x38;   // Register 56: Interrupt triggers
const uint8_t REG_ACCEL_XOUT_H = 0x3B; // Register 59: Start of Accel data
const uint8_t REG_PWR_MGMT_1 = 0x6B;   // Register 107: Power Management

// === [ 3. MIDDLEWARE ] ===
volatile bool data_ready_flag = false;
int16_t raw_ax, raw_ay, raw_az;

void configure_deadlock_timing() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x01); 
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_INT_PIN_CFG);
    Wire.write(0x20); // LATCH_INT_EN = 1: Forces INT pin HIGH until explicitly cleared
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_INT_ENABLE);
    Wire.write(0x01); // DATA_RDY_EN = 1
    Wire.endTransmission();
}

void mpu_data_ready_isr() {
    data_ready_flag = true;
}

void fetch_deadlocked_data() {
    // DELIBERATE OMISSION: We are NOT reading REG_INT_STATUS (Register 58).
    // The physical INT pin will remain HIGH forever.

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

// === [ 4. APPLICATION ] ===
void setup() {
#if DEBUG_MODE_ACTIVATE
    Serial.begin(115200);
#endif
    Wire.begin(); 
    Wire.setClock(400000); 
    
    pinMode(MPU_INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), mpu_data_ready_isr, RISING);
    
    configure_deadlock_timing();
}

void loop() {
    if (data_ready_flag) {
        data_ready_flag = false; 
        
        fetch_deadlocked_data();
        
#if DEBUG_MODE_ACTIVATE
        Serial.print(raw_ax); Serial.print(",");
        Serial.print(raw_ay); Serial.print(",");
        Serial.println(raw_az);
#endif
    }
}