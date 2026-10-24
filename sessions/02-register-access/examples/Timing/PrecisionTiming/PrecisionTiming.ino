/**
 * @file    Phase5_PrecisionTiming.ino
 * @author  Krolus Hebiesh
 * @version 1.1
 * @date    2026-08-30
 * @brief   Implements a strict 50Hz hardware-synchronized state machine using interrupt latching.
 */

#include <Wire.h>

// === [ 1. SYSTEM CONFIGURATION ] ===
#define DEBUG_MODE_ACTIVATE 1
const uint8_t MPU_INT_PIN = 2; // Physical wire connected to MPU-6050 INT pin

// === [ 2. HARDWARE ABSTRACTION LAYER (HAL) ] ===
const uint8_t MPU_ADDR = 0x68;
const uint8_t REG_SMPLRT_DIV = 0x19;   // Register 25: Throttle control[cite: 2]
const uint8_t REG_CONFIG = 0x1A;       // Register 26: DLPF control[cite: 2]
const uint8_t REG_INT_PIN_CFG = 0x37;  // Register 55: Latch behavior[cite: 2]
const uint8_t REG_INT_ENABLE = 0x38;   // Register 56: Interrupt triggers[cite: 2]
const uint8_t REG_INT_STATUS = 0x3A;   // Register 58: Interrupt flag clear[cite: 2]
const uint8_t REG_ACCEL_XOUT_H = 0x3B; // Register 59: Start of Accel data[cite: 2]
const uint8_t REG_SIGNAL_PATH_RESET = 0x68; // Register 104: Signal path flush[cite: 2]
const uint8_t REG_PWR_MGMT_1 = 0x6B;   // Register 107: Power Management[cite: 2]

// === [ 3. MIDDLEWARE ] ===
volatile bool data_ready_flag = false;
int16_t raw_ax, raw_ay, raw_az;

/**
 * @brief Configures the internal silicon timing, throttle, and latch mechanisms.
 */
void configure_precision_timing() {
    // 1. Strict Hardware Reset
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x80); // DEVICE_RESET = 1: Reverts chip to factory defaults[cite: 2]
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_SIGNAL_PATH_RESET);
    Wire.write(0x07); // Flushes analog/digital sensor paths[cite: 2]
    Wire.endTransmission();
    delay(100);

    // 2. Wake & Lock PLL
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x01); // Wake chip and lock clock to X-axis gyro PLL[cite: 2]
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_CONFIG);
    Wire.write(0x00); // DLPF OFF: Internal Gyro Output Rate defaults to 8kHz[cite: 1, 2]
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_SMPLRT_DIV);
    Wire.write(0x9F); // Divider = 159. 8000Hz / (1 + 159) = 50Hz Telemetry[cite: 2]
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_INT_PIN_CFG);
    Wire.write(0x20); // LATCH_INT_EN = 1: Forces INT pin HIGH until explicitly cleared[cite: 2]
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_INT_ENABLE);
    Wire.write(0x01); // DATA_RDY_EN = 1: Routes ADC completion pulse to INT pin[cite: 2]
    Wire.endTransmission();
}

/**
 * @brief Hardware Interrupt Service Routine (ISR)
 */
void mpu_data_ready_isr() {
    // Keep ISRs lightning fast. Only flip a boolean state flag.
    data_ready_flag = true;
}

/**
 * @brief Acknowledges the hardware interrupt and extracts kinematics.
 */
void fetch_synchronized_data() {
    // 1. Clear the latched hardware pin immediately to re-arm the sensor
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_INT_STATUS);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)1);
    Wire.read(); // Reading Register 58 physically drops the INT pin back to LOW[cite: 2]

    // 2. Burst read 6 bytes sequentially to prevent data tearing across axes
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
    Wire.setClock(400000);            // 400kHz Fast Mode for rigorous I2C throughput
    Wire.setWireTimeout(25000, true); // Prevent indefinite bus lockups 
    
    pinMode(MPU_INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), mpu_data_ready_isr, RISING);
    
    configure_precision_timing();
}

void loop() {
    // The main loop is completely non-blocking and slave to the hardware interrupt
    if (data_ready_flag) {
        data_ready_flag = false; 
        Wire.clearWireTimeoutFlag();
        
        fetch_synchronized_data();
        
#if DEBUG_MODE_ACTIVATE
        Serial.print(raw_ax); Serial.print(",");
        Serial.print(raw_ay); Serial.print(",");
        Serial.println(raw_az);
#endif
    }
}