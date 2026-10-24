#include "Kinematics.h"

void Middleware_CalculateAttitude(IMU_RawData_t *raw, IMU_State_t *state) {
    // 1. Frame Remapping (Hardware Frame to World Frame)
    float ax = -(float)raw->ax;
    float ay = -(float)raw->ay;
    float az =  (float)raw->az;
    float gz = -(float)raw->gz;

    // 2. Accelerometer Math (Absolute Tilt)
    state->roll  = atan2(ay, az) * (180.0 / PI);
    state->pitch = atan2(-ax, sqrt(ay * ay + az * az)) * (180.0 / PI);

    // 3. Gyroscope Math (Discrete Time Integration)
    float yaw_rate = gz / GYRO_SENSITIVITY;
    state->yaw += (yaw_rate * DT);
}