/**
 * @file    Mag_Calibration.ino
 * @author  Krolus Hebiesh
 * @brief   Streams raw 16-bit AK8963 magnetometer values 
 */

#include <Wire.h>

const uint8_t MPU9250_ADDR = 0x68; 
const uint8_t AK8963_ADDR  = 0x0C; 
const uint8_t MPU_PWR_MGMT_1   = 0x6B;
const uint8_t MPU_INT_PIN_CFG  = 0x37;
const uint8_t MAG_CNTL1        = 0x0A;
const uint8_t MAG_XOUT_L       = 0x03;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); 

  // Wake MPU & Open Bypass Mux
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(MPU_PWR_MGMT_1); Wire.write(0x01); 
  Wire.endTransmission(); delay(10);

  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(MPU_INT_PIN_CFG); Wire.write(0x02); 
  Wire.endTransmission(); delay(10);

  // Init AK8963 (16-bit, Cont. Mode 2)
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(MAG_CNTL1); Wire.write(0x00); 
  Wire.endTransmission(); delay(10);

  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(MAG_CNTL1); Wire.write(0x16); 
  Wire.endTransmission(); delay(10);
}

void loop() {
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(MAG_XOUT_L);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)AK8963_ADDR, (uint8_t)7);

  if (Wire.available() == 7) {
    uint8_t magData[7];
    for (int i = 0; i < 7; i++) magData[i] = Wire.read();

    if (!(magData[6] & 0x08)) { // Check for sensor overflow flag
      int16_t mx_raw = (int16_t)((magData[1] << 8) | magData[0]);
      int16_t my_raw = (int16_t)((magData[3] << 8) | magData[2]);
      int16_t mz_raw = (int16_t)((magData[5] << 8) | magData[4]);
      
      // Stream raw values to Python
      Serial.print(mx_raw); Serial.print(",");
      Serial.print(my_raw); Serial.print(",");
      Serial.println(mz_raw);
    }
  }
  delay(20); 
}