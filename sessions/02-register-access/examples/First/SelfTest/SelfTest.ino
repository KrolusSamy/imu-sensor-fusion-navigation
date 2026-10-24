/**
 * @file    SelfTest.ino
 * @author  Krolus Hebiesh
 * @version 1.0
 * @brief   Executes 6-axis deflection using strict ±8g and ±250dps baselines.
 */

#include <Wire.h>
#include <math.h>

// === [ 1. SYSTEM CONFIGURATION ] ===
#define DEBUG_MODE_ACTIVATE 1

// === [ 2. HARDWARE ABSTRACTION LAYER (HAL) ] ===
const uint8_t MPU_ADDR = 0x68;
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_GYRO_CONFIG = 0x1B;  
const uint8_t REG_ACCEL_CONFIG = 0x1C; 
const uint8_t REG_ACCEL_XOUT_H = 0x3B; 

// === [ 3. MIDDLEWARE ] ===
int16_t base_data[6] = {0}; 
int16_t test_data[6] = {0};
float factory_trim[6] = {0.0f};

void wake_silicon_and_scale() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x01); // Wake and PLL lock
    Wire.endTransmission();

    // STRICT DATASHEET REQUIREMENT: Accel to ±8g (0x10), Gyro to ±250dps (0x00)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_GYRO_CONFIG);
    Wire.write(0x00);
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_CONFIG);
    Wire.write(0x10); 
    Wire.endTransmission();
}

void fetch_all_kinematics(int16_t* buffer) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    
    Wire.requestFrom(MPU_ADDR, (uint8_t)14);
    if (Wire.available() == 14) {
        buffer[0] = (Wire.read() << 8) | Wire.read();
        buffer[1] = (Wire.read() << 8) | Wire.read();
        buffer[2] = (Wire.read() << 8) | Wire.read();
        Wire.read(); Wire.read(); // Discard temp data
        buffer[3] = (Wire.read() << 8) | Wire.read();
        buffer[4] = (Wire.read() << 8) | Wire.read();
        buffer[5] = (Wire.read() << 8) | Wire.read();
    }
}

void trigger_electrostatic_deflection(bool enable) {
    // 0xF0 sets Accel ST bits to 1 AND maintains ±8g scale
    uint8_t a_payload = enable ? 0xF0 : 0x10; 
    // 0xE0 sets Gyro ST bits to 1 AND maintains ±250dps scale
    uint8_t g_payload = enable ? 0xE0 : 0x00; 
    
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_GYRO_CONFIG);
    Wire.write(g_payload);
    Wire.endTransmission();
    
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_CONFIG);
    Wire.write(a_payload);
    Wire.endTransmission();
}

void calculate_factory_trim() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x0D); // Self-test registers 13-16
    Wire.endTransmission(false);
    
    Wire.requestFrom(MPU_ADDR, (uint8_t)4);
    if (Wire.available() == 4) {
        uint8_t r13 = Wire.read(), r14 = Wire.read(), r15 = Wire.read(), r16 = Wire.read();

        // Extract and concatenate accel and gyro bits
        uint8_t xa = ((r13 & 0xE0) >> 3) | ((r16 & 0x30) >> 4);
        uint8_t ya = ((r14 & 0xE0) >> 3) | ((r16 & 0x0C) >> 2);
        uint8_t za = ((r15 & 0xE0) >> 3) |  (r16 & 0x03);
        uint8_t xg = r13 & 0x1F, yg = r14 & 0x1F, zg = r15 & 0x1F;

        factory_trim[0] = (xa == 0) ? 0 : 4096.0f * 0.34f * powf(0.92f / 0.34f, (xa - 1.0f) / 30.0f);
        factory_trim[1] = (ya == 0) ? 0 : 4096.0f * 0.34f * powf(0.92f / 0.34f, (ya - 1.0f) / 30.0f);
        factory_trim[2] = (za == 0) ? 0 : 4096.0f * 0.34f * powf(0.92f / 0.34f, (za - 1.0f) / 30.0f);
        
        factory_trim[3] = (xg == 0) ? 0 :  25.0f * 131.0f * powf(1.046f, xg - 1.0f);
        factory_trim[4] = (yg == 0) ? 0 : -25.0f * 131.0f * powf(1.046f, yg - 1.0f); 
        factory_trim[5] = (zg == 0) ? 0 :  25.0f * 131.0f * powf(1.046f, zg - 1.0f);
    }
}

// === [ 4. APPLICATION ] ===
void setup() {
#if DEBUG_MODE_ACTIVATE
    Serial.begin(115200);
#endif
    Wire.begin(); Wire.setClock(400000); 
    Wire.setWireTimeout(25000, true);
    
    wake_silicon_and_scale();
    delay(100); 

    calculate_factory_trim();
    fetch_all_kinematics(base_data);
    
    trigger_electrostatic_deflection(true);
    delay(100); 
    fetch_all_kinematics(test_data);
    trigger_electrostatic_deflection(false);

#if DEBUG_MODE_ACTIVATE
    Serial.println("--- 6-AXIS SILICON DIAGNOSTICS ---");
    const char* axes[6] = {"AX", "AY", "AZ", "GX", "GY", "GZ"};
    for (int i = 0; i < 6; i++) {
        float str = test_data[i] - base_data[i];
        float error = ((str - factory_trim[i]) / factory_trim[i]) * 100.0f; 
        
        Serial.print(axes[i]); Serial.print(" | Error: "); 
        Serial.print(error); Serial.print("% | Result: ");
        // Tolerance is +/- 14% for all axes
        if (abs(error) <= 14.0f) Serial.println("PASS");
        else Serial.println("FAIL");
    }
#endif
}

void loop() {}