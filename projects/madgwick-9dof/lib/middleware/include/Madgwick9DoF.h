#ifndef MADGWICK_9DOF_H
#define MADGWICK_9DOF_H

#include <math.h>
#include <stdint.h>

class Madgwick9DoF {
public:
    Madgwick9DoF();
    
    // New custom initializer to eliminate boot-up drift
    void begin(int16_t* rawAccel, int16_t* rawMag); 
    
    void update(int16_t* rawAccel, int16_t* rawGyro, int16_t* rawMag, float dt);
    
    float getRoll()  { return roll; }
    float getPitch() { return pitch; }
    float getYaw()   { return yaw; }
    

private:
    float q0, q1, q2, q3; 
    float roll, pitch, yaw;
    float beta; 
    
    void computeAngles();
};

#endif