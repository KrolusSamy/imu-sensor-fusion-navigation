/**
 * @file    GyroCalibration.ino
 * @author  Krolus Hebiesh
 * @version 1.2
 * @date    2026-08-30
 * @brief   Gyroscope boot calibration sketch comparing uncorrected vs calibrated data across all three axes (X, Y, Z).
 */

#include <Wire.h>

// === [ 1. SYSTEM CONFIGURATION ] ===
#define DEBUG_MODE_ACTIVATE 1
#define ENABLE_SERIAL_PLOTTER 0 // 0 = Multi-Axis Diagnostic Logs, 1 = Clean CSV Plotter Stream
#define CALIBRATION_SAMPLES 10000
#define TELEMETRY_INTERVAL_MS 100

// === [ 2. HARDWARE ABSTRACTION LAYER (HAL) ] ===
const uint8_t MPU_ADDR = 0x68;
const uint8_t REG_PWR_MGMT_1 = 0x6B;   // Register 107: Master Power Management 1
const uint8_t REG_ACCEL_XOUT_H = 0x3B; // Register 59: Start of inertial burst data read

// Engineering conversion factor (Default range: +/-250 dps -> 131 LSB/(deg/s))
const float GYRO_SENSITIVITY = 131.0f;    

// === [ 3. MIDDLEWARE ] ===
int16_t raw_gx = 0, raw_gy = 0, raw_gz = 0;

// Accumulators for computing dynamic zero-rate bias across all axes
int32_t gyro_bias_x_sum = 0;
int32_t gyro_bias_y_sum = 0;
int32_t gyro_bias_z_sum = 0;

// Computed floating-point offsets
float gyro_offset_x = 0.0f;
float gyro_offset_y = 0.0f;
float gyro_offset_z = 0.0f;

// Uncorrected states for comparison
float uncorrected_gx = 0.0f, uncorrected_gy = 0.0f, uncorrected_gz = 0.0f;

// Calibrated states for comparison
float calibrated_gx = 0.0f, calibrated_gy = 0.0f, calibrated_gz = 0.0f;

/**
 * @brief Wakes the silicon and locks the clock to the X-Axis Gyro PLL.
 */
void initialize_sensor_hardware() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x80); // Force full device internal reset
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x01); // Wake chip and lock PLL reference to X-Gyro
    Wire.endTransmission();
}

/**
 * @brief Executes a 2000-sample static boot calibration to isolate thermal and bias offset for all axes.
 */
void execute_gyro_boot_calibration() {
#if DEBUG_MODE_ACTIVATE
    #if ENABLE_SERIAL_PLOTTER == 0
        Serial.println("[CALIBRATION] Keep sensor completely flat and motionless...");
    #endif
#endif

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(REG_ACCEL_XOUT_H); 
        Wire.endTransmission(false);
        
        // Burst read 14 bytes to clear the bus pointer across all inertial registers
        Wire.requestFrom(MPU_ADDR, (uint8_t)14);
        if (Wire.available() == 14) {
            // Discard Accelerometer (6 bytes) and Temperature (2 bytes)
            for(int j=0; j<8; j++) Wire.read();
            
            // Accumulate raw 16-bit 2's complement gyroscope rates for X, Y, and Z
            gyro_bias_x_sum += (int16_t)((Wire.read() << 8) | Wire.read());
            gyro_bias_y_sum += (int16_t)((Wire.read() << 8) | Wire.read());
            gyro_bias_z_sum += (int16_t)((Wire.read() << 8) | Wire.read());
        }
        delay(2); // Inter-sample delay for independent sampling
    }

    // Compute arithmetic mean offsets for all axes
    gyro_offset_x = (float)gyro_bias_x_sum / CALIBRATION_SAMPLES;
    gyro_offset_y = (float)gyro_bias_y_sum / CALIBRATION_SAMPLES;
    gyro_offset_z = (float)gyro_bias_z_sum / CALIBRATION_SAMPLES;

#if DEBUG_MODE_ACTIVATE
    #if ENABLE_SERIAL_PLOTTER == 0
        Serial.println("[CALIBRATION] Complete! Computed Offsets for All Axes:");
        Serial.print("  X Bias Offset: "); Serial.println(gyro_offset_x);
        Serial.print("  Y Bias Offset: "); Serial.println(gyro_offset_y);
        Serial.print("  Z Bias Offset: "); Serial.println(gyro_offset_z);
        Serial.println("----------------------------------------------------------");
    #endif  
#endif
}

/**
 * @brief Fetches raw sensor data and calculates both uncorrected and calibrated metrics for X, Y, and Z.
 */
void fetch_and_process_data() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    
    Wire.requestFrom(MPU_ADDR, (uint8_t)14);
    if (Wire.available() == 14) {
        // Discard Accelerometer and Temperature data bytes
        for(int j=0; j<8; j++) Wire.read();
        
        // Read raw gyroscope data components for all three axes
        raw_gx = (Wire.read() << 8) | Wire.read();
        raw_gy = (Wire.read() << 8) | Wire.read();
        raw_gz = (Wire.read() << 8) | Wire.read();

        // 1. Uncorrected conversions (Demonstrates raw factory bias/drift)
        uncorrected_gx = (float)raw_gx / GYRO_SENSITIVITY;
        uncorrected_gy = (float)raw_gy / GYRO_SENSITIVITY;
        uncorrected_gz = (float)raw_gz / GYRO_SENSITIVITY;

        // 2. Calibrated conversions (Middleware offset subtractions)
        calibrated_gx  = ((float)raw_gx - gyro_offset_x) / GYRO_SENSITIVITY;
        calibrated_gy  = ((float)raw_gy - gyro_offset_y) / GYRO_SENSITIVITY;
        calibrated_gz  = ((float)raw_gz - gyro_offset_z) / GYRO_SENSITIVITY;
    }
}

// === [ 4. APPLICATION ] ===
unsigned long previous_millis = 0;

void setup() {
#if DEBUG_MODE_ACTIVATE
    Serial.begin(115200);
    while(!Serial);
#endif

    // Set I2C bus to high-speed 400kHz Fast Mode
    Wire.begin();
    Wire.setClock(400000);

    // Initialize core hardware
    initialize_sensor_hardware();

    // Run the mandatory boot-time static bias capture
    execute_gyro_boot_calibration();
}

void loop() {
    unsigned long current_millis = millis();

    // Strictly non-blocking telemetry loop using millis() timing
    if (current_millis - previous_millis >= TELEMETRY_INTERVAL_MS) {
        previous_millis = current_millis;

        fetch_and_process_data();

#if DEBUG_MODE_ACTIVATE
        #if ENABLE_SERIAL_PLOTTER == 1
            // Clean comma-separated stream for Python/Arduino plotters:
            // Uncorr GX, Cal GX, Uncorr GY, Cal GY, Uncorr GZ, Cal GZ
            Serial.print(uncorrected_gx); Serial.print(",");
            Serial.print(calibrated_gx); Serial.print(",");
            Serial.print(uncorrected_gy); Serial.print(",");
            Serial.print(calibrated_gy); Serial.print(",");
            Serial.print(uncorrected_gz); Serial.print(",");
            Serial.println(calibrated_gz);
        #else
            // Detailed human-readable comparison view for all three axes
            Serial.println("--- INERTIAL AXIS CALIBRATION COMPARISON ---");
            Serial.print("GX -> Raw: "); Serial.print(uncorrected_gx, 2); Serial.print(" dps | Cal: "); Serial.print(calibrated_gx, 2); Serial.println(" dps");
            Serial.print("GY -> Raw: "); Serial.print(uncorrected_gy, 2); Serial.print(" dps | Cal: "); Serial.print(calibrated_gy, 2); Serial.println(" dps");
            Serial.print("GZ -> Raw: "); Serial.print(uncorrected_gz, 2); Serial.print(" dps | Cal: "); Serial.print(calibrated_gz, 2); Serial.println(" dps");
            Serial.println("----------------------------------------------------------");
        #endif
#endif
    }
}