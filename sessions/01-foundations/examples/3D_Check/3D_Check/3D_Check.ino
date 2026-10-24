/**
  ******************************************************************************
  * @file    IMU_3D_Check.ino
  * @author  Krolus Hebiesh
  * @version V1.0.0
  * @date    28-Aug-2026
  * @brief   High-performance, interrupt-driven GY-91 acquisition template.
  *          Outputs strict CSV telemetry for 3D Python visualization.
  ******************************************************************************
  */

#include <Wire.h> // Required for I2C communication with the MEMS sensor

// === [System_Config] ===
#define DEBUG_MODE_ACTIVATE 1 // Toggles Serial print statements to save CPU cycles in production
#define TELEMETRY_RATE_MS   30 // Outputs data at ~33Hz for smooth 3D rendering in Python

// === [HAL (Hardware Abstraction Layer)] ===
const uint8_t MPU9250_ADDR     = 0x68; // Default I2C address for MPU9250
const uint8_t AK8963_ADDR      = 0x0C; // Hidden Magnetometer I2C address
const uint8_t BMP280_ADDR      = 0x76; // Barometer I2C address

const uint8_t MPU_WHOAMI_REG   = 0x75; // Register 117: Identity check to verify silicon connection
const uint8_t MPU_PWR_MGMT_1   = 0x6B; // Register 107: Power states and PLL clock configuration
const uint8_t MPU_INT_PIN_CFG  = 0x37; // Register 55: Bypass multiplexer to reveal AK8963
const uint8_t INT_ENABLE_REG   = 0x38; // Register 56: Activates hardware interrupt generation
const uint8_t INT_STATUS_REG   = 0x3A; // Register 58: Read to clear the interrupt signal
const uint8_t ACCEL_XOUT_H_REG = 0x3B; // Register 59: Starting register for high-speed burst read

const uint8_t MAG_WHOAMI_REG   = 0x00; // AK8963 Identity register
const uint8_t MAG_CNTL1        = 0x0A; // AK8963 Control register 1 (Mode/Resolution)
const uint8_t BMP_ID_REG       = 0xD0; // BMP280 Identity register

const uint8_t INTERRUPT_PIN    = 2;    // External hardware interrupt pin mapped to the MCU

volatile bool dataReady = false;       // Volatile flag toggled by ISR; checked by the Application layer

// === [Middleware] ===
int16_t ax, ay, az;                    // Raw 16-bit 2's complement accelerometer integers
int16_t gx, gy, gz;                    // Raw 16-bit 2's complement gyroscope integers 
int16_t rawTemp;                       // Raw 16-bit thermal data from the silicon die

unsigned long lastTelemetryTime = 0;   // Non-blocking timer tracking milliseconds since last PC update

/**
  * @brief  Interrupt Service Routine (ISR). Fires when the MPU9250 INT pin goes HIGH.
  *         Must remain as short as possible to avoid delaying the RTOS/Main loop.
  */
void dataReadyISR() {
  dataReady = true; // Signal the Application layer that fresh silicon data is ready for extraction
}

// === [Application] ===
void setup() {
  #if DEBUG_MODE_ACTIVATE
    Serial.begin(115200); // Initialize high-speed UART for Python CSV telemetry
    delay(1000);          // Allow the Serial bus electrical levels to stabilize
  #endif

  Wire.begin();           // Initialize MCU as the I2C Master
  Wire.setClock(400000);  // Set I2C bus to 400kHz Fast Mode to prevent data bottlenecking

  pinMode(INTERRUPT_PIN, INPUT); // Configure MCU pin to listen for the IMU's hardware trigger

  #if DEBUG_MODE_ACTIVATE
    Serial.println("Arduino Setup Started...");
  #endif

  // --- HAL INIT: Verify MPU9250 Connection ---
  Wire.beginTransmission(MPU9250_ADDR); // Ping the specific IMU address
  Wire.write(MPU_WHOAMI_REG);           // Request the identity register
  Wire.endTransmission(false);          // Hold the I2C bus open to prevent release
  Wire.requestFrom((uint8_t)MPU9250_ADDR, (uint8_t)1); // Request 1 byte back
  uint8_t mpuWhoAmI = Wire.read();      // Read the identity byte
  
  // 0x71 or 0x73 are standard hardcoded silicon IDs for MPU9250
  if (mpuWhoAmI != 0x71 && mpuWhoAmI != 0x73) {
    #if DEBUG_MODE_ACTIVATE
      Serial.print("ERROR: MPU ID failed! Read: 0x");
      Serial.println(mpuWhoAmI, HEX);
    #endif
    while(1); // Fatal error trap; halts execution to prevent runaway kinematics
  }

  // --- HAL INIT: Wake & Clock Configuration ---
  Wire.beginTransmission(MPU9250_ADDR); 
  Wire.write(MPU_PWR_MGMT_1); // Target power management
  Wire.write(0x01);           // Wake chip and align PLL clock to X-Gyro for strict thermal stability
  Wire.endTransmission();     // Execute write

  // --- HAL INIT: I2C Bypass Multiplexer ---
  Wire.beginTransmission(MPU9250_ADDR); 
  Wire.write(MPU_INT_PIN_CFG); // Target the interrupt and bypass config register
  Wire.write(0x02);            // Open the internal silicon gate so the MCU can directly talk to the Magnetometer
  Wire.endTransmission();
  delay(10);                   // Brief pause to allow the physical multiplexer circuit to switch

  // --- HAL INIT: Interrupt Configuration ---
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(INT_ENABLE_REG);
  Wire.write(0x01); // Set DATA_RDY_EN bit so the IMU asserts the INT pin when a sample is ready
  Wire.endTransmission();
  
  // Attach the hardware interrupt. Triggers on a RISING edge (0V to 3.3V/5V)
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dataReadyISR, RISING);

  // --- HAL INIT: Verify Hidden Magnetometer ---
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(MAG_WHOAMI_REG);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)AK8963_ADDR, (uint8_t)1);
  uint8_t magWhoAmI = Wire.read();
  
  if (magWhoAmI != 0x48) { // 0x48 is the AK8963 hardcoded ID
    #if DEBUG_MODE_ACTIVATE
      Serial.println("ERROR: Mag ID failed!");
    #endif
    while(1);
  }

  // Set AK8963 to 16-bit resolution, continuous measurement mode 2 (100Hz)
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(MAG_CNTL1);
  Wire.write(0x16);      // 0x16 translates to 16-bit output at 100Hz
  Wire.endTransmission();

  // --- HAL INIT: Verify Barometer ---
  Wire.beginTransmission(BMP280_ADDR);
  Wire.write(BMP_ID_REG);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)BMP280_ADDR, (uint8_t)1);
  uint8_t bmpId = Wire.read();
  
  if (bmpId != 0x58) { // 0x58 is the BMP280 hardcoded ID
    #if DEBUG_MODE_ACTIVATE
      Serial.println("ERROR: BMP ID failed!");
    #endif
    while(1);
  }

  #if DEBUG_MODE_ACTIVATE
    Serial.println("GY-91 OK. Entering loop...");
  #endif
  
  delay(50); // Final stabilization before entering the high-speed main loop
}

void loop() {
  // Application idles until the HAL hardware interrupt asserts that new data is available
  if (dataReady) {
    dataReady = false; // Arm the flag for the next interrupt cycle immediately

    // --- HAL READ: Clear Interrupt Status ---
    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(INT_STATUS_REG);  // Target the status register
    Wire.endTransmission(false); // Hold bus
    Wire.requestFrom((uint8_t)MPU9250_ADDR, (uint8_t)1); 
    Wire.read();                 // Reading this specific register resets the physical INT pin to LOW

    // --- HAL READ: High-Speed Burst Read ---
    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(ACCEL_XOUT_H_REG); // Point to the very first accelerometer X-axis register
    Wire.endTransmission(false);  // Hold bus to ensure data lock (prevents data tearing)
    Wire.requestFrom((uint8_t)MPU9250_ADDR, (uint8_t)14); // Burst read 14 bytes: 6 Accel, 2 Temp, 6 Gyro

    // --- MIDDLEWARE: Reconstruct 16-bit Integers ---
    // Shift the High byte left by 8 bits, bitwise OR it with the Low byte
    ax      = (Wire.read() << 8) | Wire.read();
    ay      = (Wire.read() << 8) | Wire.read();
    az      = (Wire.read() << 8) | Wire.read();
    rawTemp = (Wire.read() << 8) | Wire.read();
    gx      = (Wire.read() << 8) | Wire.read();
    gy      = (Wire.read() << 8) | Wire.read();
    gz      = (Wire.read() << 8) | Wire.read();

    // (Middleware Layer algorithms for Pitch, Roll, and Yaw would execute here)

    // --- APPLICATION: Non-Blocking Telemetry ---
    #if DEBUG_MODE_ACTIVATE
      // Print strict CSV format (ax,ay,az,gx,gy,gz) mapped to the telemetry rate for Python parsing
      if (millis() - lastTelemetryTime >= TELEMETRY_RATE_MS) { 
        lastTelemetryTime = millis(); // Reset the non-blocking timer
        
        Serial.print(ax); Serial.print(",");
        Serial.print(ay); Serial.print(",");
        Serial.print(az); Serial.print(",");
        Serial.print(gx); Serial.print(","); 
        Serial.print(gy); Serial.print(","); 
        Serial.println(gz); 
      }
    #endif
  }
}