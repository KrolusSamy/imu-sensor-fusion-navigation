/**
 * @file    WHO_AM_I.ino
 * @author  Krolus Hebiesh
 * @version 1.0
 * @brief   Validates physical I2C silicon connectivity by reading the WHO_AM_I register.
 */

#include <Wire.h>

// === [ 1. SYSTEM CONFIGURATION ] ===
#define DEBUG_MODE_ACTIVATE 1     // Set to 0 in production to save CPU cycles
#define TELEMETRY_INTERVAL_MS 500 // Non-blocking telemetry timer

// === [ 2. HARDWARE ABSTRACTION LAYER (HAL) ] ===
const uint8_t MPU_ADDR     = 0x68; // Default I2C address when AD0 is grounded
const uint8_t REG_WHO_AM_I = 0x75; // Register 117: Contains the I2C identity
const uint8_t EXPECTED_ID  = 0x68; // Factory-hardcoded identity payload

// === [ 3. MIDDLEWARE ] ===
uint8_t silicon_id = 0x00;        
bool hardware_verified = false;   

// === [ 4. APPLICATION ] ===
unsigned long previous_millis = 0;

void setup() {
#if DEBUG_MODE_ACTIVATE
    Serial.begin(115200);
#endif

    Wire.begin();
    Wire.setClock(400000);            // 400kHz Fast Mode for deterministic communication
    Wire.setWireTimeout(25000, true); // Prevent mechanical bus lockups from breadboard bouncing

    // Point the I2C multiplexer to the WHO_AM_I register
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_WHO_AM_I); 
    Wire.endTransmission(false);      // Send restart to keep bus active

    // Request exactly 1 byte to extract the silicon identity
    Wire.requestFrom(MPU_ADDR, (uint8_t)1);
    if (Wire.available()) {
        silicon_id = Wire.read(); 
    }

    if (silicon_id == EXPECTED_ID) {
        hardware_verified = true;
    }
}

void loop() {
    unsigned long current_millis = millis();
    if (current_millis - previous_millis >= TELEMETRY_INTERVAL_MS) {
        previous_millis = current_millis;

#if DEBUG_MODE_ACTIVATE
        Wire.clearWireTimeoutFlag();
        if (hardware_verified) {
            Serial.print("SUCCESS: Silicon Verified. WHO_AM_I = 0x");
            Serial.println(silicon_id, HEX);
        } else {
            Serial.print("FATAL HARDWARE FAULT: Expected 0x68, Read 0x");
            Serial.println(silicon_id, HEX);
        }
#endif
    }
}