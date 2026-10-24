#include <Arduino.h>
#include "GY91.h"
#include "Madgwick9DoF.h"

#define DEBUG_MODE_ACTIVATE 1

GY91 imu_hw;
Madgwick9DoF sensor_fusion;

// === [ APP LAYER: TIMING & SCHEDULING ] ===
// Hardware SMPLRT_DIV is set to 4 (200Hz).
// Therefore, new data is physically ready exactly every 5000 microseconds.
const uint32_t SAMPLE_PERIOD_US = 5000; 
uint32_t lastLoopTime = 0;

int16_t raw_accel[3], raw_gyro[3], raw_mag[3];

void setup() {
#if DEBUG_MODE_ACTIVATE
    Serial.begin(115200);
    while(!Serial);
    Serial.println("Initializing GY-91 9-DoF System...");
#endif

    imu_hw.init();
    
    // Hardware delay: Allow the AK8963 and MPU6500 to fill their data registers
    delay(20); 

    // 1. Extract the very first physical reading from the HAL
    imu_hw.readRawAccelGyro(raw_accel, raw_gyro);
    imu_hw.readRawMag(raw_mag);

    // 2. Feed that physical baseline into the dynamic initializer
    sensor_fusion.begin(raw_accel, raw_mag);
    
    lastLoopTime = micros();
}

void loop() {
    uint32_t currentTime = micros();
    
    // Non-blocking deterministic polling: avoids I2C flood, matches exact hardware sample rate
    if (currentTime - lastLoopTime >= SAMPLE_PERIOD_US) {
        float dt = (float)(currentTime - lastLoopTime) / 1000000.0f;
        lastLoopTime = currentTime;

        // 1. Data Extraction (HAL)
        imu_hw.readRawAccelGyro(raw_accel, raw_gyro);
        imu_hw.readRawMag(raw_mag);

        // 2. Fusion Pipeline (Middleware)
        sensor_fusion.update(raw_accel, raw_gyro, raw_mag, dt);

        // 3. Telemetry (Application)
#if DEBUG_MODE_ACTIVATE
        Serial.print(sensor_fusion.getRoll());Serial.print(",");
        Serial.print(sensor_fusion.getPitch());Serial.print(",");
        Serial.println(sensor_fusion.getYaw());
#endif
    }
    
    // MCU is free here to process other tasks without lag
}