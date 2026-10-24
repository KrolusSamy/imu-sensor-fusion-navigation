#include "Madgwick9DoF.h"

// === [ MIDDLEWARE: CALIBRATION CONSTANTS ] ===
// Values provided from physical pre-calibration sequence
const float ACCEL_BIAS[3]  = {-54.00f, 120.50f, 4887.00f};
const float ACCEL_SCALE[3] = {-1.0005f, -1.0005f, 0.9921f};
const float GYRO_BIAS[3]   = {12.46f, -138.24f, -41.57f};
const float MAG_BIAS[3]    = {-37.50f, 73.50f, -331.50f};
const float MAG_SCALE[3]   = {1.0957f, 1.0705f, 0.8672f};

// Physics Multipliers based on chosen Full-Scale Range

const float GYRO_RES = (3.1415926535f / 180.0f) / 131.0f; 

Madgwick9DoF::Madgwick9DoF() {
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    beta = 0.1f; // Gradient descent step size
}
void Madgwick9DoF::begin(int16_t* rawAccel, int16_t* rawMag) {
    // Apply calibration to find true initial vectors
    float ax = (rawAccel[0] - ACCEL_BIAS[0]) * ACCEL_SCALE[0];
    float ay = (rawAccel[1] - ACCEL_BIAS[1]) * ACCEL_SCALE[1];
    float az = (rawAccel[2] - ACCEL_BIAS[2]) * ACCEL_SCALE[2];
    
    float my =  (rawMag[0] - MAG_BIAS[0]) * MAG_SCALE[0];   
    float mx = -(rawMag[1] - MAG_BIAS[1]) * MAG_SCALE[1];   
    float mz = -(rawMag[2] - MAG_BIAS[2]) * MAG_SCALE[2];   

    // 1. Calculate absolute starting pitch and roll from gravity
    float initial_roll = atan2(ay, az);
    float initial_pitch = atan2(-ax, sqrt(ay * ay + az * az));

    // 2. Calculate tilt-compensated starting yaw from magnetic north
    float mx_comp = mx * cos(initial_pitch) + my * sin(initial_roll) * sin(initial_pitch) + mz * cos(initial_roll) * sin(initial_pitch);
    float my_comp = my * cos(initial_roll) - mz * sin(initial_roll);
    float initial_yaw = atan2(-my_comp, mx_comp);

    // 3. Convert absolute Euler angles into initial Quaternions
    float cy = cos(initial_yaw * 0.5f);
    float sy = sin(initial_yaw * 0.5f);
    float cp = cos(initial_pitch * 0.5f);
    float sp = sin(initial_pitch * 0.5f);
    float cr = cos(initial_roll * 0.5f);
    float sr = sin(initial_roll * 0.5f);

    q0 = cr * cp * cy + sr * sp * sy;
    q1 = sr * cp * cy - cr * sp * sy;
    q2 = cr * sp * cy + sr * cp * sy;
    q3 = cr * cp * sy - sr * sp * cy;
}
// === [ MIDDLEWARE: MADGWICK FILTER CORE ] ===
void Madgwick9DoF::update(int16_t* rawAccel, int16_t* rawGyro, int16_t* rawMag, float dt) {
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    
    // 1. Apply calibration models
    ax = (rawAccel[0] - ACCEL_BIAS[0]) * ACCEL_SCALE[0];
    ay = (rawAccel[1] - ACCEL_BIAS[1]) * ACCEL_SCALE[1];
    az = (rawAccel[2] - ACCEL_BIAS[2]) * ACCEL_SCALE[2];
    
    gx = (rawGyro[0] - GYRO_BIAS[0]) * GYRO_RES;
    gy = (rawGyro[1] - GYRO_BIAS[1]) * GYRO_RES;
    gz = (rawGyro[2] - GYRO_BIAS[2]) * GYRO_RES;
    
    // Physics Reality: The AK8963 magnetometer is mounted on the MPU9250 die 
    // with its X and Y axes swapped, and Z inverted. We must align them mathematically.
    my =  (rawMag[0] - MAG_BIAS[0]) * MAG_SCALE[0];   // Mag X -> Accel Y
    mx = -(rawMag[1] - MAG_BIAS[1]) * MAG_SCALE[1];   // Mag Y -> Negative Accel X (FIXED)
    mz = -(rawMag[2] - MAG_BIAS[2]) * MAG_SCALE[2];   // Mag Z -> Negative Accel Z

    // 2. Quaternion Derivative from Gyroscope (Rate of Change)
    float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    float qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    float qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Standard Madgwick Optimization: Normalize Accel & Mag
    // (Optimization mathematics omitted for brevity - standard Madgwick 9DoF implementation handles 
    // the objective function J and Jacobian matrices here to calculate the step direction `s`)
    // ... [Insert standard Open-Source Madgwick Gradient Descent Math block here] ...
    // Standard Madgwick Optimization: Normalize Accel & Mag
    float recipNorm;
    
    // Guard against zero-division (e.g., during freefall)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        
        // Normalize accelerometer measurement
        recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
        
        // Normalize magnetometer measurement
        recipNorm = 1.0f / sqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        float _2q0mx = 2.0f * q0 * mx;
        float _2q0my = 2.0f * q0 * my;
        float _2q0mz = 2.0f * q0 * mz;
        float _2q1mx = 2.0f * q1 * mx;
        float _2q0 = 2.0f * q0;
        float _2q1 = 2.0f * q1;
        float _2q2 = 2.0f * q2;
        float _2q3 = 2.0f * q3;
        float _2q0q2 = 2.0f * q0 * q2;
        float _2q2q3 = 2.0f * q2 * q3;
        float q0q0 = q0 * q0;
        float q0q1 = q0 * q1;
        float q0q2 = q0 * q2;
        float q0q3 = q0 * q3;
        float q1q1 = q1 * q1;
        float q1q2 = q1 * q2;
        float q1q3 = q1 * q3;
        float q2q2 = q2 * q2;
        float q2q3 = q2 * q3;
        float q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        float hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        float hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        float _2bx = sqrt(hx * hx + hy * hy);
        float _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        float _4bx = 2.0f * _2bx;
        float _4bz = 2.0f * _2bz;

        // Gradient descent algorithm corrective step
        float s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        float s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        float s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        float s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        
        recipNorm = 1.0f / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }
    // 3. Integrate to yield Quaternions
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // 4. Normalize resulting Quaternion
    recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    computeAngles();
}

void Madgwick9DoF::computeAngles() {
    roll = atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 57.29578f;
    
    // CRITICAL BUG FIX: Clamp the internal product to prevent NaN mathematical crashes
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    pitch = asin(sinp) * 57.29578f;
    
    yaw = atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 57.29578f;
}
