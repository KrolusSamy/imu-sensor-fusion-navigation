/*=========================================================================
 * Tool: Advanced I2C Scanner (with MPU-9250 Bypass)
 *=========================================================================*/
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(1000);
  
  Serial.println("\n--- Advanced I2C Scanner Started ---");

  // MAGIC TRICK: Open the I2C Bypass on the MPU-9250
  // This allows the Arduino to "see" the hidden Magnetometer
  Wire.beginTransmission(0x68); // MPU default address
  Wire.write(0x37);             // INT_PIN_CFG register
  Wire.write(0x02);             // Turn on BYPASS_EN bit
  byte error = Wire.endTransmission();
  
  if(error == 0) {
    Serial.println("Bypass gate opened! Magnetometer should now be visible.");
  }
}

void loop() {
  byte error, address;
  int deviceCount = 0;

  Serial.println("Scanning bus...");

  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("✅ Device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      deviceCount++;
    }  
  }
  
  if (deviceCount == 0) Serial.println("⚠️ No devices found.");
  else Serial.println("--- Scan Complete ---\n");

  delay(5000);
}