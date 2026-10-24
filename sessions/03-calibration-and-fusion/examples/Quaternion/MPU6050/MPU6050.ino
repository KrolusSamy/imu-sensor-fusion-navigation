/**
 * @file    MPU6050.ino
 * @brief   Non-blocking, interrupt-driven 6-axis kinematics for Python serialization.
 *          Features strict I2C bus recovery and hardware-level timing.
 */

#include <Wire.h>

// === [ Hardware Mapping & Registers ] ===
const uint8_t MPU_ADDR = 0x68;
const uint8_t MPU_INT_PIN = 2; 

const uint8_t REG_SMPLRT_DIV        = 0x19; 
const uint8_t REG_CONFIG            = 0x1A; 
const uint8_t REG_GYRO_CONFIG       = 0x1B; 
const uint8_t REG_ACCEL_CONFIG      = 0x1C; 
const uint8_t REG_INT_PIN_CFG       = 0x37; 
const uint8_t REG_INT_ENABLE        = 0x38; 
const uint8_t REG_INT_STATUS        = 0x3A; 
const uint8_t REG_ACCEL_XOUT_H      = 0x3B; 
const uint8_t REG_SIGNAL_PATH_RESET = 0x68; 
const uint8_t REG_PWR_MGMT_1        = 0x6B; 

// === [ Middleware State ] ===
volatile bool data_ready_flag = false; 
int16_t raw_ax, raw_ay, raw_az;
int16_t raw_gx, raw_gy, raw_gz;

// === [ Interrupt Service Routine ] ===
void mpu_data_ready_isr() {
    data_ready_flag = true;
}

// === [ Silicon Initialization ] ===
void configure_aerospace_timing() {
    // 1. Hard Silicon Reset
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x80); 
    Wire.endTransmission();
    delay(100); 

    // 2. Signal Path Flush
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_SIGNAL_PATH_RESET);
    Wire.write(0x07); 
    Wire.endTransmission();
    delay(100);

    // 3. Wake & PLL Lock (X-Axis Gyro Reference)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x01); 
    Wire.endTransmission();

    // 4. Digital Low Pass Filter (44Hz Accel / 42Hz Gyro, clocks chip to 1kHz)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_CONFIG);
    Wire.write(0x03); 
    Wire.endTransmission();
    
    // 5. Kinematic Scaling (±4g Accel, ±500°/s Gyro)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_CONFIG);
    Wire.write(0x08); 
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_GYRO_CONFIG);
    Wire.write(0x08); 
    Wire.endTransmission();

    // 6. Hardware Decimation (1000Hz / (1 + 19) = 50Hz)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_SMPLRT_DIV);
    Wire.write(0x13); 
    Wire.endTransmission();

    // 7. Interrupt Latching (Prevents 50µs deadlock)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_INT_PIN_CFG);
    Wire.write(0x20); 
    Wire.endTransmission();

    // 8. Interrupt Enable
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_INT_ENABLE);
    Wire.write(0x01); 
    Wire.endTransmission();
}

// === [ Data Extraction ] ===
void fetch_6dof_kinematics() {
    // Acknowledge interrupt to drop INT pin LOW
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_INT_STATUS);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)1);
    Wire.read(); 

    // Burst read physical memory block
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_XOUT_H); 
    Wire.endTransmission(false);
    
    Wire.requestFrom(MPU_ADDR, (uint8_t)14); 
    if (Wire.available() == 14) {
        raw_ax = (Wire.read() << 8) | Wire.read();
        raw_ay = (Wire.read() << 8) | Wire.read();
        raw_az = (Wire.read() << 8) | Wire.read();
        
        Wire.read(); Wire.read(); // Discard temperature
        
        raw_gx = (Wire.read() << 8) | Wire.read();
        raw_gy = (Wire.read() << 8) | Wire.read();
        raw_gz = (Wire.read() << 8) | Wire.read();
    }
}

void setup() {
    Serial.begin(115200);
    
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
        Wire.clearWireTimeoutFlag(); 
        
        fetch_6dof_kinematics();
        
        // Strict CSV formatting
        Serial.print(raw_ax); Serial.print(",");
        Serial.print(raw_ay); Serial.print(",");
        Serial.print(raw_az); Serial.print(",");
        Serial.print(raw_gx); Serial.print(",");
        Serial.print(raw_gy); Serial.print(",");
        Serial.println(raw_gz);
    }
}