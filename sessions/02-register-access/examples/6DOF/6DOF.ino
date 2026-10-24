/**
 * @file    Phase5_6DOF_Telemetry.ino
 * @author  Krolus Hebiesh
 * @version 1.2
 * @date    2026-08-30
 * @brief   Extracts 6-axis kinematics (Accel + Gyro) at 50Hz with a mid-range 44Hz DLPF and strict I2C resilience.
 */

#include <Wire.h>

// === [ 1. SYSTEM CONFIGURATION ] ===
#define DEBUG_MODE_ACTIVATE 1
const uint8_t MPU_INT_PIN = 2; // Physical wire routing the sensor's hardware trigger to the MCU

// === [ 2. HARDWARE ABSTRACTION LAYER (HAL) ] ===
// MPU-6050 Core I2C Address (AD0 pin grounded)
const uint8_t MPU_ADDR = 0x68;

// Register Map Definitions[cite: 2]
const uint8_t REG_SMPLRT_DIV        = 0x19; // Reg 25: Divides internal clock for target sample rate[cite: 2]
const uint8_t REG_CONFIG            = 0x1A; // Reg 26: Digital Low Pass Filter (DLPF) bandwidth[cite: 2]
const uint8_t REG_GYRO_CONFIG       = 0x1B; // Reg 27: Gyroscope full-scale range[cite: 2]
const uint8_t REG_ACCEL_CONFIG      = 0x1C; // Reg 28: Accelerometer full-scale range[cite: 2]
const uint8_t REG_INT_PIN_CFG       = 0x37; // Reg 55: Interrupt pin behavior (latching)[cite: 2]
const uint8_t REG_INT_ENABLE        = 0x38; // Reg 56: Enables DATA_RDY_EN hardware trigger[cite: 2]
const uint8_t REG_INT_STATUS        = 0x3A; // Reg 58: Reading this drops the INT pin LOW[cite: 2]
const uint8_t REG_ACCEL_XOUT_H      = 0x3B; // Reg 59: Base address for the 14-byte kinematic block[cite: 2]
const uint8_t REG_SIGNAL_PATH_RESET = 0x68; // Reg 104: Flushes analog/digital sensor ADCs[cite: 2]
const uint8_t REG_PWR_MGMT_1        = 0x6B; // Reg 107: Sleep, Reset, and PLL clock selection[cite: 2]

// === [ 3. MIDDLEWARE ] ===
volatile bool data_ready_flag = false; // Thread-safe flag for the ISR
int16_t raw_ax, raw_ay, raw_az;        // 16-bit 2's complement acceleration buckets
int16_t raw_gx, raw_gy, raw_gz;        // 16-bit 2's complement angular rate buckets

// === [ 4. APPLICATION ] ===

/**
 * @brief Configures the internal silicon timing, throttle, and latch mechanisms.
 *        Executes a strict sequential boot-up to ensure stable MEMS oscillator states.
 */
void configure_aerospace_timing() {
    // 1. Hard Silicon Reset: Revert all registers to factory defaults
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x80); // Sets DEVICE_RESET bit to 1. The chip reboots[cite: 2].
    Wire.endTransmission();
    delay(100); // Mandatory physical wait for silicon to stabilize

    // 2. Signal Path Flush: Clear out any corrupted data in the ADCs
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_SIGNAL_PATH_RESET);
    Wire.write(0x07); // Resets Gyro, Accel, and Temp signal paths[cite: 2].
    Wire.endTransmission();
    delay(100);

    // 3. Wake & Phase-Locked Loop (PLL) Lock
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x01); // Clears sleep and locks the timing generator to the X-axis gyro[cite: 2]. Highly stable.
    Wire.endTransmission();

    // 4. Digital Low Pass Filter (The "Middle" Ground)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_CONFIG);
    // 0x03 sets DLPF_CFG=3. Accel bandwidth drops to 44Hz (4.9ms delay)[cite: 2].
    // Gyro bandwidth drops to 42Hz (4.8ms delay)[cite: 2]. 
    // CRITICAL: Enabling the DLPF instantly slows the internal sensor clock to 1kHz[cite: 2].
    Wire.write(0x03); 
    Wire.endTransmission();
    
    // 5. Kinematic Scaling (Dynamically resizing the measurement buckets)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_CONFIG);
    Wire.write(0x08); // AFS_SEL=1. Sets Accel to ±4g, yielding 8192 LSB/g resolution[cite: 2].
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_GYRO_CONFIG);
    Wire.write(0x08); // FS_SEL=1. Sets Gyro to ±500°/s, yielding 65.5 LSB/°/s resolution[cite: 2].
    Wire.endTransmission();

    // 6. Precision Throttle (Hardware Decimation)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_SMPLRT_DIV);
    // Divider = 19. Formula: Internal Clock / (1 + Divider)[cite: 2].
    // 1000Hz / (1 + 19) = 50Hz exact hardware sampling rate.
    Wire.write(0x13); 
    Wire.endTransmission();

    // 7. Interrupt Latching (The Deadlock Prevention)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_INT_PIN_CFG);
    Wire.write(0x20); // LATCH_INT_EN=1. Prevents the 50µs pulse trap. Pin stays HIGH until cleared[cite: 2].
    Wire.endTransmission();

    // 8. Interrupt Enable
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_INT_ENABLE);
    Wire.write(0x01); // DATA_RDY_EN=1. Routes the ADC completion flag to the physical INT pin[cite: 2].
    Wire.endTransmission();
}

/**
 * @brief Hardware Interrupt Service Routine (ISR)
 *        Kept ultra-lightweight. Only sets a volatile flag. No I2C or Serial calls allowed here.
 */
void mpu_data_ready_isr() {
    data_ready_flag = true;
}

/**
 * @brief Acknowledges the hardware interrupt and safely extracts the kinematic payload.
 */
void fetch_6dof_kinematics() {
    // 1. Acknowledge the interrupt to re-arm the sensor
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_INT_STATUS);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)1);
    Wire.read(); // Reading Reg 58 physically pulls the latched INT pin back to LOW[cite: 2].

    // 2. Burst Read the 14-byte memory block
    // We start at Reg 59 and read continuously to Reg 72 to guarantee all axes are from the exact same physical timestep[cite: 2].
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_XOUT_H); 
    Wire.endTransmission(false);
    
    Wire.requestFrom(MPU_ADDR, (uint8_t)14); 
    if (Wire.available() == 14) {
        // Registers 59 to 64: Accelerometer (X, Y, Z)[cite: 2]
        raw_ax = (Wire.read() << 8) | Wire.read();
        raw_ay = (Wire.read() << 8) | Wire.read();
        raw_az = (Wire.read() << 8) | Wire.read();
        
        // Registers 65 and 66: Temperature. We discard them to save processing time[cite: 2].
        Wire.read(); Wire.read(); 
        
        // Registers 67 to 72: Gyroscope (X, Y, Z)[cite: 2]
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
    Wire.setClock(400000); // 400kHz Fast Mode for rigorous I2C throughput without blocking
    Wire.setWireTimeout(25000, true); // Instantly resets the I2C bus state machine if breadboard wires bounce
    
    pinMode(MPU_INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), mpu_data_ready_isr, RISING);
    
    configure_aerospace_timing();
}

void loop() {
    // Completely non-blocking state machine. The CPU sleeps/idles until the silicon commands it.
    if (data_ready_flag) {
        data_ready_flag = false; 
        Wire.clearWireTimeoutFlag(); // Reset the diagnostic timeout flag before the transaction
        
        fetch_6dof_kinematics();
        
#if DEBUG_MODE_ACTIVATE
        // Comma-separated CSV output perfectly mapped to the dynamic PyQtGraph dashboard parser
        Serial.print(raw_ax); Serial.print(",");
        Serial.print(raw_ay); Serial.print(",");
        Serial.print(raw_az); Serial.print(",");
        Serial.print(raw_gx); Serial.print(",");
        Serial.print(raw_gy); Serial.print(",");
        Serial.println(raw_gz);
#endif
    }
}