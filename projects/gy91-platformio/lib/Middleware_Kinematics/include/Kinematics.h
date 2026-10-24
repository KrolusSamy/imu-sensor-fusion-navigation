#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <math.h>
#include "System_Config.h"
#include "HAL_GY91.h"

// Struct to hold finalized engineering units
typedef struct {
    float pitch;
    float roll;
    float yaw;
} IMU_State_t;

void Middleware_CalculateAttitude(IMU_RawData_t *raw, IMU_State_t *state);

#endif