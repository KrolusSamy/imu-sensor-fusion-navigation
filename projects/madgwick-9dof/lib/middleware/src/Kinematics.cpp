/**
 * @file    Kinematics.cpp
 * @author  Krolus Hebiesh
 * @brief   Converts raw byte arrays into signed 16-bit physical kinematics.
 */

#include "Kinematics.h"

void Process_Raw_Buffer(uint8_t* buffer, IMU_Data_t* out_data) {
    // Bitwise shift 8-bit registers into 16-bit 2's complement integers.
    // Skips bytes 6 and 7 (buffer[6], buffer[7]) to discard thermal data.
    out_data->ax = (buffer[0] << 8) | buffer[1];
    out_data->ay = (buffer[2] << 8) | buffer[3];
    out_data->az = (buffer[4] << 8) | buffer[5];
    
    out_data->gx = (buffer[8] << 8) | buffer[9];
    out_data->gy = (buffer[10] << 8) | buffer[11];
    out_data->gz = (buffer[12] << 8) | buffer[13];
}