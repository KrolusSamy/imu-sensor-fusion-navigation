/**
 * @file    SleepTrap.ino
 * @author  Krolus Hebiesh
 * @version 1.0
 * @brief   Demonstrates the default sleep state trap and precisely wakes the PLL.
 */

#include <Wire.h>

// === [ 1. SYSTEM CONFIGURATION ] ===
#define DEBUG_MODE_ACTIVATE 1
#define TELEMETRY_INTERVAL_MS 100

// === [ 2. HARDWARE ABSTRACTION LAYER (HAL) ] ===
const uint8_t MPU_ADDR = 0x68;
const uint8_t REG_PWR_MGMT_1 = 0x6B;   // Register 107
const uint8_t REG_ACCEL_XOUT_H = 0x3B; // Register 59: Start of Accel data

// === [ 3. MIDDLEWARE ] ===
int16_t raw_ax, raw_ay, raw_az;

void force_sleep_state() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x80); // DEVICE_RESET bit forces chip to default Sleep Mode
    Wire.endTransmission();
    delay(100);       // Allow silicon to physically reboot
}

void wake_silicon_and_lock_pll() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x01); // Clears SLEEP bit and locks clock to X-axis gyro PLL
    Wire.endTransmission();
}

void fetch_acceleration() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_XOUT_H); 
    Wire.endTransmission(false);
    
    // Burst read all 6 bytes to prevent data tearing
    Wire.requestFrom(MPU_ADDR, (uint8_t)6); 
    if (Wire.available() == 6) {
        raw_ax = (Wire.read() << 8) | Wire.read();
        raw_ay = (Wire.read() << 8) | Wire.read();
        raw_az = (Wire.read() << 8) | Wire.read();
    }
}

// === [ 4. APPLICATION ] ===
unsigned long previous_millis = 0;

void setup() {
#if DEBUG_MODE_ACTIVATE
    Serial.begin(115200);
#endif
    Wire.begin(); 
    Wire.setClock(400000); 
    Wire.setWireTimeout(25000, true);
    
    force_sleep_state(); // Trap: sleep mode initially
}

void loop() {
    unsigned long current_millis = millis();
    if (current_millis - previous_millis >= TELEMETRY_INTERVAL_MS) {
        previous_millis = current_millis;
        
        // Wake the chip exactly 5 seconds into execution
        if (current_millis > 5000 && current_millis < 5150) {
            wake_silicon_and_lock_pll();
#if DEBUG_MODE_ACTIVATE
            Serial.println("--- SILICON WAKE & PLL LOCK EXECUTED ---");
#endif
        }

        Wire.clearWireTimeoutFlag();
        fetch_acceleration();

#if DEBUG_MODE_ACTIVATE
        // Formatted for the Serial Plotter
        Serial.print(raw_ax); Serial.print(",");
        Serial.print(raw_ay); Serial.print(",");
        Serial.println(raw_az); 
#endif
    }
}