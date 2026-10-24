/**
  ******************************************************************************
  * @file    IMU_System_Main.ino
  * @author  Krolus Samy Hebiesh
  * @version V1.0.0
  * @date    28-Aug-2026
  * @brief   High-performance, interrupt-driven MPU6050 acquisition template.
  *          Structured into HAL, Middleware, and Application layers.
  ******************************************************************************
  */

#include <Wire.h> // Required for I2C communication with the MEMS sensor

// ==============================================================================
// 1. CONFIGURATION PARAMETERS (System_Config)
// ==============================================================================
#define DEBUG_MODE_ACTIVATE 1 // Toggles Serial print statements to save CPU cycles in production[cite: 1]

// ==============================================================================
// 2. HARDWARE ABSTRACTION LAYER (HAL - MPU6050)
// ==============================================================================
const uint8_t MPU_ADDR         = 0x68; // Default I2C address when AD0 pin is grounded
const uint8_t WHO_AM_I_REG     = 0x75; // Register 117: Identity check to verify silicon connection
const uint8_t PWR_MGMT_1_REG   = 0x6B; // Register 107: Power states and PLL clock configuration
const uint8_t INT_ENABLE_REG   = 0x38; // Register 56: Activates hardware interrupt generation
const uint8_t INT_STATUS_REG   = 0x3A; // Register 58: Read to clear the interrupt signal
const uint8_t ACCEL_XOUT_H_REG = 0x3B; // Register 59: Starting register for 14-byte burst read

const uint8_t INTERRUPT_PIN    = 2;    // External hardware interrupt pin (INT0) mapped to MCU

volatile bool dataReady = false;       // Volatile flag toggled by ISR, checked by Main loop

// ==============================================================================
// 3. MIDDLEWARE LAYER (Data Processing & State Variables)
// ==============================================================================
int16_t accelX, accelY, accelZ;        // Raw 16-bit 2's complement accelerometer integers
int16_t gyroX, gyroY, gyroZ;           // Raw 16-bit 2's complement gyroscope integers
int16_t rawTemp;                       // Raw 16-bit thermal data
float temperatureC;                    // Processed engineering unit for thermal compensation

unsigned long lastPrintTime = 0;       // Tracks milliseconds for non-blocking serial telemetry

// ==============================================================================
// INTERRUPT SERVICE ROUTINE (ISR)
// ==============================================================================
/**
  * @brief  Fires instantly when the MPU6050 pulls the MCU's INT pin high.
  *         Must remain as short as possible to avoid crashing the RTOS/Main loop.
  */
void dataReadyISR() {
  dataReady = true; // Signals the application layer that fresh silicon data is available
}

// ==============================================================================
// 4. APPLICATION LAYER (Initialization & Main Loop)
// ==============================================================================
void setup() {
  #if DEBUG_MODE_ACTIVATE
    Serial.begin(115200); // Initialize high-speed UART for telemetry visualization
  #endif

  Wire.begin();           // Initialize MCU as I2C Master
  Wire.setClock(400000);  // Set I2C bus to 400kHz Fast Mode to prevent data bottlenecking

  pinMode(INTERRUPT_PIN, INPUT); // Configure MCU pin to listen to the IMU's hardware trigger

  // --- HAL INIT: Verify Connection ---
  Wire.beginTransmission(MPU_ADDR); // Ping the specific IMU address on the bus
  Wire.write(WHO_AM_I_REG);         // Request the identity register
  Wire.endTransmission(false);      // Hold the bus open
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1); // Request 1 byte back
  uint8_t whoAmI = Wire.read();     // Read the identity byte
  
  if (whoAmI != 0x68) {             // 0x68 is the hardcoded silicon ID
    #if DEBUG_MODE_ACTIVATE
      Serial.println("HAL_ERROR: MPU6050 Missing!"); // Failsafe warning
    #endif
    while (1);                      // Fatal error trap; halts execution to prevent runaway code
  }

  // --- HAL INIT: Wake & Clock Configuration ---
  Wire.beginTransmission(MPU_ADDR); // Open communication
  Wire.write(PWR_MGMT_1_REG);       // Target the power management register
  Wire.write(0x01);                 // 0x01 sets SLEEP=0 and CLKSEL=1 (X-Gyro PLL for stability)
  Wire.endTransmission();           // Execute write

  // --- HAL INIT: Interrupt Configuration ---
  Wire.beginTransmission(MPU_ADDR); // Open communication
  Wire.write(INT_ENABLE_REG);       // Target the interrupt enable register
  Wire.write(0x01);                 // 0x01 sets DATA_RDY_EN bit to trigger INT pin on new data
  Wire.endTransmission();           // Execute write

  // Map the physical pin to the software ISR. Trigger on RISING edge (0V to 3.3V)
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dataReadyISR, RISING);
  
  delay(100); // Brief stabilization buffer before entering continuous loop
}

void loop() {
  // Application layer idles until the HAL signals that new data exists
  if (dataReady) {
    dataReady = false; // Immediately reset flag to arm the system for the next interrupt

    // --- HAL READ: Clear Interrupt Status ---
    Wire.beginTransmission(MPU_ADDR); // Open communication
    Wire.write(INT_STATUS_REG);       // Target the status register
    Wire.endTransmission(false);      // Hold bus
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1); // Request 1 byte
    Wire.read();                      // Reading this specific register pulls the IMU INT pin back LOW

    // --- HAL READ: High-Speed Burst Read ---
    Wire.beginTransmission(MPU_ADDR); // Open communication
    Wire.write(ACCEL_XOUT_H_REG);     // Point to the very first accelerometer data register
    Wire.endTransmission(false);      // Hold bus to ensure data lock
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14); // Request 14 consecutive bytes to guarantee aligned sample timing

    // --- MIDDLEWARE: Reconstruct 16-bit Integers ---
    // Shift the High byte left by 8 bits, bitwise OR it with the Low byte
    accelX  = (Wire.read() << 8) | Wire.read();
    accelY  = (Wire.read() << 8) | Wire.read();
    accelZ  = (Wire.read() << 8) | Wire.read();
    rawTemp = (Wire.read() << 8) | Wire.read();
    gyroX   = (Wire.read() << 8) | Wire.read();
    gyroY   = (Wire.read() << 8) | Wire.read();
    gyroZ   = (Wire.read() << 8) | Wire.read();

    // --- MIDDLEWARE: Engineering Unit Conversion ---
    temperatureC = (rawTemp / 340.0) + 36.53; // Convert raw integer to Celsius using datasheet formula

    // --- APPLICATION: Non-Blocking Telemetry ---
    #if DEBUG_MODE_ACTIVATE
      if (millis() - lastPrintTime >= 100) { // Only execute print payload once every 1000ms
        lastPrintTime = millis();             // Reset the non-blocking timer
        
        Serial.print("Acc[X,Y,Z]: ");
        Serial.print(accelX); Serial.print(",");
        Serial.print(accelY); Serial.print(",");
        Serial.print(accelZ);
        
        Serial.print(" | Gyro[X,Y,Z]: ");
        Serial.print(gyroX); Serial.print(",");
        Serial.print(gyroY); Serial.print(",");
        Serial.print(gyroZ);
        
        Serial.print(" | Temp: ");
        Serial.println(temperatureC);
      }
    #endif
  }
}