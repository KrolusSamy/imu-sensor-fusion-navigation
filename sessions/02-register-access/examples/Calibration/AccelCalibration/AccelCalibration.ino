/**
 * @file    Phase6_AccelerometerCalibration.ino
 * @author  Krolus Hebiesh
 * @version 1.1
 * @date    2026-08-30
 * @brief   Executes an interactive 6-Point Tumble Calibration with robust serial character sanitization.
 */

#include <Wire.h>

// === [ 1. SYSTEM CONFIGURATION ] ===
#define DEBUG_MODE_ACTIVATE 1

// === [ 2. HARDWARE ABSTRACTION LAYER (HAL) ] ===
const uint8_t MPU_ADDR = 0x68;
const uint8_t REG_ACCEL_CONFIG = 0x1C; // Register 28: Accelerometer Scale
const uint8_t REG_ACCEL_XOUT_H = 0x3B; // Register 59: Start of Accel data
const uint8_t REG_PWR_MGMT_1   = 0x6B; // Register 107: Power Management

// Target Sensitivity for ±4g range = 8192 LSB/g
const float TARGET_SENSITIVITY = 8192.0f; 

// === [ 3. MIDDLEWARE ] ===
float accel_offset_x = 0.0f, accel_scale_x = 1.0f;
float accel_offset_y = 0.0f, accel_scale_y = 1.0f;
float accel_offset_z = 0.0f, accel_scale_z = 1.0f;

bool calibration_complete = false;

/**
 * @brief Clears out all pending serial input, including hidden carriage returns and newlines.
 */
void clear_serial_buffer() {
    while (Serial.available() > 0) {
        Serial.read();
    }
}

/**
 * @brief Halts execution until a clean character is received, ignoring stray control bytes.
 */
void wait_for_user_acknowledgment() {
    clear_serial_buffer(); // Purge old characters first
    while (true) {
        if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n');
            input.trim(); // Strips spaces, \r, and \n
            
            // If anything valid remains after trimming, proceed
            if (input.length() > 0) {
                break;
            }
        }
    }
}

/**
 * @brief Initializes the sensor and sets the kinematic scale to ±4g.
 */
void initialize_sensor_for_calibration() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x01); // Wake chip and lock clock to X-axis gyro PLL
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_CONFIG);
    Wire.write(0x08); // AFS_SEL = 1: ±4g range (8192 LSB/g)
    Wire.endTransmission();
    delay(100);
}

/**
 * @brief Helper function to average 200 samples for a targeted axis (0=X, 1=Y, 2=Z).
 */
int16_t read_axis_average(uint8_t axis_index) {
    long sum = 0;
    int samples = 200;

    for (int i = 0; i < samples; i++) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(REG_ACCEL_XOUT_H); 
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, (uint8_t)6);
        
        if (Wire.available() == 6) {
            int16_t raw_data[3];
            raw_data[0] = (Wire.read() << 8) | Wire.read(); // X
            raw_data[1] = (Wire.read() << 8) | Wire.read(); // Y
            raw_data[2] = (Wire.read() << 8) | Wire.read(); // Z
            
            sum += raw_data[axis_index];
        }
        delay(3);
    }
    return (int16_t)(sum / samples);
}

/**
 * @brief Guides the user through the 6-point tumble calibration procedure safely.
 */
void run_6point_calibration() {
    int16_t x_plus = 0, x_minus = 0;
    int16_t y_plus = 0, y_minus = 0;
    int16_t z_plus = 0, z_minus = 0;

    Serial.println("\n==========================================");
    Serial.println("       6-POINT TUMBLE CALIBRATION         ");
    Serial.println("==========================================");

    // --- FACE 1: +Z UP ---
    Serial.println("\n[1/6] Place sensor completely FLAT, facing UP (+Z against gravity).");
    Serial.println("Press any character when ready...");
    wait_for_user_acknowledgment();
    
    Serial.println("Sampling +Z...");
    z_plus = read_axis_average(2);

    // --- FACE 2: -Z UP ---
    Serial.println("\n[2/6] Flip sensor completely UPSIDE DOWN (-Z against gravity).");
    Serial.println("Press any character when ready...");
    wait_for_user_acknowledgment();
    
    Serial.println("Sampling -Z...");
    z_minus = read_axis_average(2);

    // --- FACE 3: +X UP ---
    Serial.println("\n[3/6] Stand sensor on its LEFT edge (+X against gravity).");
    Serial.println("Press any character when ready...");
    wait_for_user_acknowledgment();
    
    Serial.println("Sampling +X...");
    x_plus = read_axis_average(0);

    // --- FACE 4: -X UP ---
    Serial.println("\n[4/6] Stand sensor on its RIGHT edge (-X against gravity).");
    Serial.println("Press any character when ready...");
    wait_for_user_acknowledgment();
    
    Serial.println("Sampling -X...");
    x_minus = read_axis_average(0);

    // --- FACE 5: +Y UP ---
    Serial.println("\n[5/6] Stand sensor on its TOP edge (+Y against gravity).");
    Serial.println("Press any character when ready...");
    wait_for_user_acknowledgment();
    
    Serial.println("Sampling +Y...");
    y_plus = read_axis_average(1);

    // --- FACE 6: -Y UP ---
    Serial.println("\n[6/6] Stand sensor on its BOTTOM edge (-Y against gravity).");
    Serial.println("Press any character when ready...");
    wait_for_user_acknowledgment();
    
    Serial.println("Sampling -Y...");
    y_minus = read_axis_average(1);

    // --- MATH COMPUTATION ---
    accel_offset_x = (float)(x_plus + x_minus) / 2.0f;
    accel_offset_y = (float)(y_plus + y_minus) / 2.0f;
    accel_offset_z = (float)(z_plus + z_minus) / 2.0f;

    accel_scale_x = (2.0f * TARGET_SENSITIVITY) / (float)(x_plus - x_minus);
    accel_scale_y = (2.0f * TARGET_SENSITIVITY) / (float)(y_plus - y_minus);
    accel_scale_z = (2.0f * TARGET_SENSITIVITY) / (float)(z_plus - z_minus);

    Serial.println("\n==========================================");
    Serial.println("          CALIBRATION COMPLETE            ");
    Serial.println("==========================================");
    Serial.print("X Offset: "); Serial.print(accel_offset_x); Serial.print(" | X Scale: "); Serial.println(accel_scale_x, 4);
    Serial.print("Y Offset: "); Serial.print(accel_offset_y); Serial.print(" | Y Scale: "); Serial.println(accel_scale_y, 4);
    Serial.print("Z Offset: "); Serial.print(accel_offset_z); Serial.print(" | Z Scale: "); Serial.println(accel_scale_z, 4);
    Serial.println("==========================================");

    calibration_complete = true;
}

// === [ 4. APPLICATION ] ===
void setup() {
#if DEBUG_MODE_ACTIVATE
    Serial.begin(115200);
#endif
    Wire.begin();
    Wire.setClock(400000); 
    Wire.setWireTimeout(25000, true);

    initialize_sensor_for_calibration();
    run_6point_calibration();
}

void loop() {
    if (calibration_complete) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(REG_ACCEL_XOUT_H);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, (uint8_t)6);

        if (Wire.available() == 6) {
            int16_t raw_x = (Wire.read() << 8) | Wire.read();
            int16_t raw_y = (Wire.read() << 8) | Wire.read();
            int16_t raw_z = (Wire.read() << 8) | Wire.read();

            float corrected_x = ((float)raw_x - accel_offset_x) * accel_scale_x;
            float corrected_y = ((float)raw_y - accel_offset_y) * accel_scale_y;
            float corrected_z = ((float)raw_z - accel_offset_z) * accel_scale_z;

#if DEBUG_MODE_ACTIVATE
            Serial.print("Calibrated Accel [g] -> X: ");
            Serial.print(corrected_x / TARGET_SENSITIVITY, 3);
            Serial.print(" | Y: ");
            Serial.print(corrected_y / TARGET_SENSITIVITY, 3);
            Serial.print(" | Z: ");
            Serial.println(corrected_z / TARGET_SENSITIVITY, 3);
#endif
        }
        delay(100);
    }
}