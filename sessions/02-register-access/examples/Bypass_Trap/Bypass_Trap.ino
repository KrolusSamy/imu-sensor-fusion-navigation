/**
 * @file    Bypass_Trap.ino
 * @author  Embedded Systems Firmware Engineer
 * @version 1.2
 * @date    2026-08-29
 * @brief   Demonstrates multi-layered I2C routing by unlocking two nested hardware multiplexers.
 */

#include <Wire.h>

// === [ 1. SYSTEM CONFIGURATION ] ===
#define DEBUG_MODE_ACTIVATE 1
#define SCAN_INTERVAL_MS 5000 

// === [ 2. HARDWARE ABSTRACTION LAYER (HAL) ] ===
// The LSB of the 7-bit address is determined by the logic level on pin AD0.
const uint8_t MAIN_MPU_ADDR = 0x69; // AD0 tied HIGH (VCC)
const uint8_t GY91_MPU_ADDR = 0x68; // GY-91 default AD0 state

const uint8_t REG_USER_CTRL = 0x6A;    // Register 106: Master Lockout
const uint8_t REG_INT_PIN_CFG = 0x37;  // Register 55: Bypass Switch
const uint8_t REG_PWR_MGMT_1 = 0x6B;   // Register 107: Power Control

const uint8_t I2C_BYPASS_EN_BIT = 0x02; // Bit 1 closes internal analog switch

// === [ 3. MIDDLEWARE ] ===
bool bypass_state = false;

/**
 * @brief Wakes a specific silicon chip and locks its PLL.
 */
void wake_silicon(uint8_t target_address) {
    Wire.beginTransmission(target_address);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x01); // Wake up and lock PLL
    Wire.endTransmission();
}

/**
 * @brief Overthrows the dictator I2C master mode on a target chip.
 */
void disable_master_lockout(uint8_t target_address) {
    Wire.beginTransmission(target_address);
    Wire.write(REG_USER_CTRL);
    Wire.write(0x00); // Forces I2C_MST_EN to 0
    Wire.endTransmission();
}

/**
 * @brief Physically bridges the main and auxiliary I2C buses on a target chip.
 */
void toggle_bypass_mux(uint8_t target_address, bool enable) {
    uint8_t payload = enable ? I2C_BYPASS_EN_BIT : 0x00;
    
    Wire.beginTransmission(target_address);
    Wire.write(REG_INT_PIN_CFG);
    // Setting Bit 1 allows direct access to the auxiliary I2C bus.
    Wire.write(payload); 
    Wire.endTransmission();
}

/**
 * @brief Sweeps all 127 I2C addresses and reports active silicon.
 */
void execute_i2c_scan() {
#if DEBUG_MODE_ACTIVATE
    uint8_t devices_found = 0;
    
    for (uint8_t address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            Serial.print("Silicon detected at address: 0x");
            if (address < 16) Serial.print("0");
            Serial.print(address, HEX);
            
            // Contextual labeling for the students
            if (address == 0x69) Serial.println(" (Main MPU-6050)");
            else if (address == 0x68) Serial.println(" (GY-91 MPU)");
            else if (address == 0x76) Serial.println(" (GY-91 Barometer)");
            else if (address == 0x0C) Serial.println(" (GY-91 Magnetometer!)");
            else Serial.println(" (Unknown)");
            
            devices_found++;
        }
    }
    
    if (devices_found == 0) Serial.println("Bus dead. No silicon found.");
    Serial.println("-------------------------------");
#endif
}

// === [ 4. APPLICATION ] ===
unsigned long previous_millis = 0;

void setup() {
#if DEBUG_MODE_ACTIVATE
    Serial.begin(115200);
#endif
    Wire.begin();
    Wire.setClock(400000); 
    delay(100); 
    
    // 1. Initialize Main MPU-6050 (0x69)
    wake_silicon(MAIN_MPU_ADDR);
    disable_master_lockout(MAIN_MPU_ADDR);
    toggle_bypass_mux(MAIN_MPU_ADDR, false); 
}

void loop() {
    unsigned long current_millis = millis();
    
    if (current_millis - previous_millis >= SCAN_INTERVAL_MS) {
        previous_millis = current_millis;
        
#if DEBUG_MODE_ACTIVATE
        Serial.print("\n[STATE] Hardware Bypass logic is currently: ");
        Serial.println(bypass_state ? "DOUBLE-BYPASS ACTIVE" : "ISOLATED");
        Serial.println("--- INITIATING I2C BUS SCAN ---");
#endif

        execute_i2c_scan();
        
        // Flip the state for the next cycle
        bypass_state = !bypass_state;
        
        if (bypass_state) {
            // EXECUTE THE INCEPTION BYPASS
            // First, open the gates of the main MPU
            toggle_bypass_mux(MAIN_MPU_ADDR, true);
            
            // Now that the GY-91 is visible, configure its internal registers
            wake_silicon(GY91_MPU_ADDR);
            disable_master_lockout(GY91_MPU_ADDR);
            // Finally, open the gates of the GY-91's MPU to reveal the compass
            toggle_bypass_mux(GY91_MPU_ADDR, true);
        } else {
            // ISOLATE EVERYTHING
            toggle_bypass_mux(MAIN_MPU_ADDR, false);
            // We do not need to command the GY-91 to close, because closing 
            // the main MPU severs the physical wire connection entirely.
        }
    }
}