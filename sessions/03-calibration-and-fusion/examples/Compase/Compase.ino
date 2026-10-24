/**
 * @file    com.ino
 * @author  Krolus Hebiesh
 * @version 2.0
 * @date    2026-09-07
 * @brief   Tilt-compensated compass using GY-91 (MPU-9250 + AK8963).
 */

#include <Wire.h>
#include <math.h>

// --- I2C Addresses ---
const uint8_t MPU9250_ADDR = 0x68; 
const uint8_t AK8963_ADDR  = 0x0C; 

// --- Registers ---
const uint8_t ACCEL_XOUT_H     = 0x3B;
const uint8_t MPU_PWR_MGMT_1   = 0x6B;
const uint8_t MPU_INT_PIN_CFG  = 0x37;
const uint8_t MAG_CNTL1        = 0x0A;
const uint8_t MAG_XOUT_L       = 0x03;

// ==========================================
// 1. PYTHON-GENERATED HARD/SOFT IRON MATRICES
// ==========================================
const float MAG_BIAS[3]  = {-26.50, 18.50, -247.00};
const float MAG_SCALE[3] = {1.0070, 1.0165, 0.9773};

// Magnetic declination for Cairo, Egypt (+5.1 degrees)
const float DECLINATION_ANGLE = 5.1;

// --- Timing Configuration (20Hz non-blocking loop) ---
unsigned long last_execution_time = 0;
const uint16_t INTERVAL_MS = 50; 

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  // Wake MPU & Open Bypass Mux
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(MPU_PWR_MGMT_1); Wire.write(0x01);
  Wire.endTransmission(); delay(10);

  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(MPU_INT_PIN_CFG); Wire.write(0x02);
  Wire.endTransmission(); delay(10);

  // Init AK8963 Magnetometer (16-bit, Cont. Mode 2)
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(MAG_CNTL1); Wire.write(0x00);
  Wire.endTransmission(); delay(10);

  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(MAG_CNTL1); Wire.write(0x16);
  Wire.endTransmission(); delay(10);
  
  Serial.println("Tilt-Compensated Compass Initialized.");
  delay(500);
}

void loop() {
  unsigned long current_time = millis();
  
  // Non-blocking timer replaces delay() to allow future task scaling
  if (current_time - last_execution_time >= INTERVAL_MS) {
    last_execution_time = current_time;

    // --- 1. Read Accelerometer ---
    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU9250_ADDR, (uint8_t)6);
    
    if (Wire.available() == 6) {
      float ax = (int16_t)((Wire.read() << 8) | Wire.read());
      float ay = (int16_t)((Wire.read() << 8) | Wire.read());
      float az = (int16_t)((Wire.read() << 8) | Wire.read());

      // --- 2. Read Magnetometer ---
      Wire.beginTransmission(AK8963_ADDR);
      Wire.write(MAG_XOUT_L);
      Wire.endTransmission(false);
      Wire.requestFrom((uint8_t)AK8963_ADDR, (uint8_t)7);

      if (Wire.available() == 7) {
        uint8_t magData[7];
        for (int i = 0; i < 7; i++) magData[i] = Wire.read();

        if (!(magData[6] & 0x08)) { // Check for sensor overflow flag
          int16_t mx_raw = (int16_t)((magData[1] << 8) | magData[0]);
          int16_t my_raw = (int16_t)((magData[3] << 8) | magData[2]);
          int16_t mz_raw = (int16_t)((magData[5] << 8) | magData[4]);
          
          // Apply 3D Calibration (Hard-Iron offset & Soft-Iron scale)
          float mx_cal = (mx_raw - MAG_BIAS[0]) * MAG_SCALE[0];
          float my_cal = (my_raw - MAG_BIAS[1]) * MAG_SCALE[1];
          float mz_cal = (mz_raw - MAG_BIAS[2]) * MAG_SCALE[2];

          // Align Magnetometer axes to Accelerometer orientation
          float mx = my_cal;
          float my = mx_cal;
          float mz = -mz_cal;

          // --- 3. Sensor Fusion Math (Tilt Compensation) ---
          float roll = atan2(ay, az);
          float pitch = atan2(-ax, sqrt(ay * ay + az * az));

          // Compensate horizontal components against tilt angles
          float X_compensated = mx * cos(pitch) + mz * sin(pitch);
          float Y_compensated = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch);

          // Calculate Absolute Heading
          float heading = atan2(Y_compensated, X_compensated) * 180.0 / PI;

          // Correct to True North using regional magnetic declination
          heading += DECLINATION_ANGLE;

          // Normalize bounds to [0, 360] degrees
          if (heading < 0)   { heading += 360; }
          if (heading >= 360){ heading -= 360; }

          // --- Serial Telemetry Output ---
          Serial.print("Heading: ");
          Serial.print(heading, 1);
          Serial.println(" degrees");
        }
      }
    }
  }
}