#ifndef HAL_GY91_H
#define HAL_GY91_H

#include <Arduino.h>
#include <Wire.h>
#include "System_Config.h"

// Struct to hold raw integers stripped directly from silicon
typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t mx, my, mz;
} IMU_RawData_t;

void HAL_GY91_Init();
void HAL_GY91_ReadRaw(IMU_RawData_t *data);
void HAL_GY91_ClearInterrupt();

#endif