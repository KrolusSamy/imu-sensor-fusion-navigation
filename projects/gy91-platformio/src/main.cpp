/**
 * @file    main.cpp
 * @author  Krolus Hebiesh
 * @brief   Application layer orchestrating HAL and Middleware.
 */
#include <Arduino.h>
#include "System_Config.h"
#include "HAL_GY91.h"
#include "Kinematics.h"

IMU_RawData_t rawData;
IMU_State_t   vehicleState;
unsigned long lastTelemetryTime = 0;

void setup() {
    #if DEBUG_MODE_ACTIVATE
        Serial.begin(115200);
        delay(1000);
    #endif

    HAL_GY91_Init(); // Boot physical silicon
}

void loop() {
    // 1. HAL Fetch
    HAL_GY91_ReadRaw(&rawData);

    // 2. Middleware Compute
    Middleware_CalculateAttitude(&rawData, &vehicleState);

    // 3. Application Telemetry (Non-Blocking)
    #if DEBUG_MODE_ACTIVATE
        if (millis() - lastTelemetryTime >= TELEMETRY_RATE_MS) {
            lastTelemetryTime = millis();
            
            // Print strict CSV format: ax, ay, az, gx, gy, gz
            Serial.print(rawData.ax); Serial.print(",");
            Serial.print(rawData.ay); Serial.print(",");
            Serial.print(rawData.az); Serial.print(",");
            Serial.print(rawData.gx); Serial.print(",");
            Serial.print(rawData.gy); Serial.print(",");
            Serial.println(rawData.gz);
        }
    #endif
}