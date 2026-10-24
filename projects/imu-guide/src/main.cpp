/**
 * @file    main.cpp
 * @author  Krolus Hebiesh
 * @version 2.0
 * @date    2026-09-07
 * @brief   Application Orchestrator supporting compile-time switching between 
 *          interrupt-driven MPU-6050 and polled GY-91 hardware abstraction layers.
 */

#include <Arduino.h>
#include <Wire.h>
#include "Kinematics.h"

// === [ 1. SYSTEM CONFIGURATION ] ===
#define DEBUG_MODE_ACTIVATE 1

// Hardware Selection Flag:
// Set to 0 for MPU-6050 (Interrupt-Driven on Pin 2)
// Set to 1 for GY-91 (MPU-9250 + AK8963, Non-blocking Polled Timer)
#define TARGET_SENSOR_GY91 1 

#if TARGET_SENSOR_GY91
    #include "GY91.h"
    const uint32_t POLL_INTERVAL_MS = 20; // 50Hz sample rate timer
    uint32_t last_poll_time = 0;
#else
    #include "MPU6050.h"
    const uint8_t MPU_INT_PIN = 2; 
    volatile bool data_ready_flag = false;
#endif

// === [ 2. GLOBAL STATE ] ===
IMU_Data_t current_kinematics;
uint8_t raw_i2c_buffer[14];

// === [ 3. INTERRUPT SERVICE ROUTINE (MPU-6050 Only) ] ===
#if !TARGET_SENSOR_GY91
void mpu_data_ready_isr() {
    data_ready_flag = true;
}
#endif

// === [ 4. APPLICATION SETUP ] ===
void setup() {
#if DEBUG_MODE_ACTIVATE
    Serial.begin(115200);
#endif
    
    Wire.begin(); 
    Wire.setClock(400000);            // Enforce 400kHz Fast Mode I2C
    Wire.setWireTimeout(25000, true); // Guard against clock-stretch bus locks
    
#if TARGET_SENSOR_GY91
    GY91_Init();
#else
    pinMode(MPU_INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), mpu_data_ready_isr, RISING);
    MPU6050_Init();
#endif
}

// === [ 5. APPLICATION LOOP ] ===
void loop() {
#if TARGET_SENSOR_GY91
    // GY-91 Polling Architecture (Safely handles unrouted breakout INT pins)
    uint32_t current_time = millis();
    if (current_time - last_poll_time >= POLL_INTERVAL_MS) {
        last_poll_time = current_time;
        Wire.clearWireTimeoutFlag();
        
        GY91_Clear_Interrupt();
        
        if (GY91_Read_Burst(raw_i2c_buffer, 14)) {
            Process_Raw_Buffer(raw_i2c_buffer, &current_kinematics);
            
#if DEBUG_MODE_ACTIVATE
            Serial.print(current_kinematics.ax); Serial.print(",");
            Serial.print(current_kinematics.ay); Serial.print(",");
            Serial.print(current_kinematics.az); Serial.print(",");
            Serial.print(current_kinematics.gx); Serial.print(",");
            Serial.print(current_kinematics.gy); Serial.print(",");
            Serial.println(current_kinematics.gz);
#endif
        }
    }
#else
    // MPU-6050 Interrupt-Driven Architecture
    if (data_ready_flag) {
        data_ready_flag = false; 
        Wire.clearWireTimeoutFlag(); 
        
        MPU6050_Clear_Interrupt();
        
        if (MPU6050_Read_Burst(raw_i2c_buffer, 14)) {
            Process_Raw_Buffer(raw_i2c_buffer, &current_kinematics);
            
#if DEBUG_MODE_ACTIVATE
            Serial.print(current_kinematics.ax); Serial.print(",");
            Serial.print(current_kinematics.ay); Serial.print(",");
            Serial.print(current_kinematics.az); Serial.print(",");
            Serial.print(current_kinematics.gx); Serial.print(",");
            Serial.print(current_kinematics.gy); Serial.print(",");
            Serial.println(current_kinematics.gz);
#endif
        }
    }
#endif
}