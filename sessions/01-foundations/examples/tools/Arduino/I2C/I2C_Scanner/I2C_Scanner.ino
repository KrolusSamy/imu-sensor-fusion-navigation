/*=========================================================================
 * Tool: I2C Address Scanner
 * Description: Scans the I2C bus and prints the address of any connected 
 *              devices. Perfect for verifying wiring before writing complex code.
 *=========================================================================*/
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Give the serial monitor a moment to open
  delay(1000);
  Serial.println("\n--- I2C Scanner Started ---");
}

void loop() {
  byte error, address;
  int deviceCount = 0;

  Serial.println("Scanning bus...");

  // The I2C address space goes from 1 to 127
  for(address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of Write.endTransmission 
    // to see if a device acknowledged the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("✅ Device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
      deviceCount++;
    }
    else if (error == 4) {
      Serial.print("❌ Unknown error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }    
  }
  
  if (deviceCount == 0) {
    Serial.println("⚠️ No I2C devices found. Check your SDA/SCL wiring!");
  } else {
    Serial.println("--- Scan Complete ---\n");
  }

  delay(5000); // Wait 5 seconds and scan again
}