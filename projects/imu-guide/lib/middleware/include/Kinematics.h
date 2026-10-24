/**
 * @file    Kinematics.h
 * @author  Krolus Hebiesh
 * @brief   Hardware-agnostic math layer for processing raw data streams.
 */

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>

typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
} IMU_Data_t;

void Process_Raw_Buffer(uint8_t* buffer, IMU_Data_t* out_data);

#endif // KINEMATICS_H