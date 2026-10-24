/**
 * @file    I2C_SiliconDiagnostic.ino
 * @author  Krolus Hebiesh
 * @version 1.0
 * @date    2026-08-30
 * @brief   Diagnostic tool to verify I2C bus resilience and MPU-6050 physical identity.
 */

#include <Wire.h>

// === [ 1. SYSTEM CONFIGURATION ] ===
#define DEBUG_MODE_ACTIVATE 1
#define DIAGNOSTIC_INTERVAL_MS 200 // 5Hz polling rate allows for physical movement testing

// === [ 2. HARDWARE ABSTRACTION LAYER (HAL) ] ===
const uint8_t MPU_ADDR = 0x68;     // Default I2C address when AD0 is grounded[cite: 2]
const uint8_t REG_WHO_AM_I = 0x75; // Register 117: Factory-hardcoded identity payload[cite: 2]

// === [ 3. MIDDLEWARE ] ===
uint8_t tx_status = 0;
bool timeout_occurred = false;
uint8_t who_am_i_val = 0;

/**
 * @brief Queries the silicon identity and monitors for physical I2C bus lockups.
 */
void execute_bus_diagnostic() {
    // Clear any previous timeout flags before initiating a new hardware transaction
    Wire.clearWireTimeoutFlag();
    
    // Point the internal I2C multiplexer to the WHO_AM_I register[cite: 2]
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_WHO_AM_I);
    // Send a restart condition instead of a stop to keep the bus active
    tx_status = Wire.endTransmission(false); 
    
    // Check if the physical wire bounced and triggered a state machine lockup
    timeout_occurred = Wire.getWireTimeoutFlag();
    
    // Request exactly 1 byte from the silicon
    Wire.requestFrom(MPU_ADDR, (uint8_t)1);
    if (Wire.available() == 1) {
        who_am_i_val = Wire.read();
    } else {
        who_am_i_val = 0x00; // Reset payload on failed read
    }
}

// === [ 4. APPLICATION ] ===
unsigned long previous_millis = 0;

void setup() {
#if DEBUG_MODE_ACTIVATE
    Serial.begin(115200);
    Serial.println("--- I2C & Silicon Diagnostic Started ---");
#endif

    Wire.begin();
    // Force I2C hardware to 400kHz Fast Mode for rigorous throughput
    Wire.setClock(400000); 
    
    // Enforce a 25ms timeout limit. 
    // The 'true' parameter forces the MCU to physically reset the I2C bus on timeout.
    Wire.setWireTimeout(25000, true); 
}

void loop() {
    unsigned long current_millis = millis();
    
    // Strictly non-blocking state machine execution replaces delay()
    if (current_millis - previous_millis >= DIAGNOSTIC_INTERVAL_MS) {
        previous_millis = current_millis;
        
        execute_bus_diagnostic();
        
#if DEBUG_MODE_ACTIVATE
        Serial.print("TX Status: ");
        Serial.print(tx_status);
        Serial.print(" | Timeout: ");
        Serial.print(timeout_occurred ? "YES" : "NO");
        Serial.print(" | WHO_AM_I: 0x");
        Serial.println(who_am_i_val, HEX);
#endif
    }
}