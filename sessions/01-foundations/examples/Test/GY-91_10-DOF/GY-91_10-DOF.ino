/**
  ******************************************************************************
  * @file    GY91_Diagnostic_Test.ino
  * @author  Krolus Hebiesh
  * @version V1.0.0
  * @date    28-Aug-2026
  * @brief   Interrupt-driven, non-blocking 10-DoF sensor diagnostic tool.
  *          Validates all three silicon engines (MPU9250, AK8963, BMP280).
  ******************************************************************************
  */

#include <Wire.h> // Core Arduino I2C hardware library

// === [System_Config] ===
#define DEBUG_MODE_ACTIVATE 1    // Toggles Serial telemetry to save CPU cycles in production builds
#define TELEMETRY_RATE_MS   500  // Rate in milliseconds for the non-blocking PC telemetry output

// === [HAL (Hardware Abstraction Layer)] ===
// --- I2C Device Addresses ---
const uint8_t MPU9250_ADDR = 0x68; // Primary 6-DoF Inertial Measurement Unit
const uint8_t AK8963_ADDR  = 0x0C; // 3-DoF Magnetometer hidden behind the I2C bypass multiplexer
const uint8_t BMP280_ADDR  = 0x76; // 1-DoF Barometric Pressure Sensor

// --- MPU-9250 Register Map ---
const uint8_t MPU_WHOAMI_REG   = 0x75; // Silicon identity register for verification
const uint8_t MPU_PWR_MGMT_1   = 0x6B; // Controls sleep states and clock PLL sources
const uint8_t MPU_INT_PIN_CFG  = 0x37; // Configures physical INT pin behavior and the Bypass Mux
const uint8_t INT_ENABLE_REG   = 0x38; // Activates hardware interrupts
const uint8_t INT_STATUS_REG   = 0x3A; // Clears the interrupt flag when read
const uint8_t ACCEL_XOUT_H_REG = 0x3B; // Starting register for the 14-byte IMU burst read

// --- AK8963 Magnetometer Register Map ---
const uint8_t MAG_WHOAMI_REG   = 0x00; // Silicon identity register for the compass
const uint8_t MAG_CNTL1        = 0x0A; // Configures ADC resolution and sample rate
const uint8_t MAG_XOUT_L       = 0x03; // Starting register for the 7-byte magnetic burst read

// --- BMP280 Barometer Register Map & Factory Trim Parameters ---
const uint8_t BMP_ID_REG       = 0xD0; // Silicon identity register for the barometer
const uint8_t BMP_CTRL_MEAS    = 0xF4; // Configures oversampling and power modes
const uint8_t BMP_PRESS_MSB    = 0xF7; // Starting register for pressure and temperature burst read

// Factory calibration constants programmed into the chip's ROM at manufacturing
uint16_t bmp_dig_T1;
int16_t  bmp_dig_T2, bmp_dig_T3;
uint16_t bmp_dig_P1;
int16_t  bmp_dig_P2, bmp_dig_P3, bmp_dig_P4, bmp_dig_P5, bmp_dig_P6, bmp_dig_P7, bmp_dig_P8, bmp_dig_P9;

// --- Hardware Interrupt Configuration ---
const uint8_t INTERRUPT_PIN = 2; // External hardware interrupt pin mapped to the MCU
volatile bool dataReady = false; // Flag asserted by the ISR, checked by the Application layer

/**
  * @brief  Fetches Non-Volatile Memory (NVM) trim parameters from the BMP280.
  *         Required to compensate for raw pressure drift using Bosch's formula.
  */
void readBMPCoefficients() {
  Wire.beginTransmission(BMP280_ADDR);
  Wire.write(0x88); // Address of the first trim register
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)BMP280_ADDR, (uint8_t)24); // Burst read all 24 bytes of calibration data

  // Reconstruct 16-bit integers (Little Endian format for the BMP280)
  bmp_dig_T1 = (Wire.read() | (Wire.read() << 8));
  bmp_dig_T2 = (Wire.read() | (Wire.read() << 8));
  bmp_dig_T3 = (Wire.read() | (Wire.read() << 8));
  bmp_dig_P1 = (Wire.read() | (Wire.read() << 8));
  bmp_dig_P2 = (Wire.read() | (Wire.read() << 8));
  bmp_dig_P3 = (Wire.read() | (Wire.read() << 8));
  bmp_dig_P4 = (Wire.read() | (Wire.read() << 8));
  bmp_dig_P5 = (Wire.read() | (Wire.read() << 8));
  bmp_dig_P6 = (Wire.read() | (Wire.read() << 8));
  bmp_dig_P7 = (Wire.read() | (Wire.read() << 8));
  bmp_dig_P8 = (Wire.read() | (Wire.read() << 8));
  bmp_dig_P9 = (Wire.read() | (Wire.read() << 8));
}

/**
  * @brief  Interrupt Service Routine (ISR).
  *         Triggers when the MPU9250 finishes a sample cycle and pulls the line HIGH.
  */
void dataReadyISR() {
  dataReady = true;
}

// === [Middleware] ===
int16_t ax, ay, az, gx, gy, gz; // Reconstructed IMU integers
int16_t mx, my, mz;             // Reconstructed Compass integers
float mpuTempC, bmpTempC;       // Processed engineering units for temperature
uint32_t rawPress;              // Uncompensated 20-bit pressure data
bool magOverflow;               // Flag to detect magnetic ADC clipping

int32_t t_fine;                 // Global fine temperature variable required for pressure compensation
unsigned long lastPrintTime = 0;// Non-blocking timer

/**
  * @brief  Official Bosch BMP280 Thermal Compensation Math (32-bit).
  *         Corrects the raw thermal drift based on the factory ROM coefficients.
  */
int32_t compensate_T(int32_t adc_T) {
  int32_t var1, var2, T;
  var1 = (((adc_T >> 3) - ((int32_t)bmp_dig_T1 << 1))) * ((int32_t)bmp_dig_T2) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)bmp_dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp_dig_T1))) >> 12) * ((int32_t)bmp_dig_T3)) >> 14;
  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  return T; // Returns temperature in 0.01 degC increments
}

// === [Application] ===
void setup() {
  #if DEBUG_MODE_ACTIVATE
    Serial.begin(115200); // Initialize UART telemetry
    delay(1000); // Allow terminal connection to stabilize
    Serial.println("\n--- GY-91 10-DoF Comprehensive Initialization ---");
  #endif

  Wire.begin(); 
  Wire.setClock(400000); // Force I2C Fast-Mode (400kHz) to prevent bus bottlenecking
  pinMode(INTERRUPT_PIN, INPUT); 

  // --- 1. MPU-9250 Silicon Verification ---
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(MPU_WHOAMI_REG);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU9250_ADDR, (uint8_t)1);
  uint8_t mpuWhoAmI = Wire.read();
  
  if (mpuWhoAmI != 0x71 && mpuWhoAmI != 0x73) {
    #if DEBUG_MODE_ACTIVATE
      Serial.println("HAL ERROR: MPU9250 NOT FOUND!");
    #endif
    while(1); // System halt. Never fly if the primary IMU is dead.
  }

  // Wake the chip and align the PLL clock to the X-Gyro for strict thermal stability
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(MPU_PWR_MGMT_1);
  Wire.write(0x01);
  Wire.endTransmission();

  // Open the internal I2C multiplexer gate to reveal the AK8963 compass to the master MCU
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(MPU_INT_PIN_CFG);
  Wire.write(0x02);
  Wire.endTransmission();
  
  // Enable the hardware interrupt generation on fresh data
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(INT_ENABLE_REG);
  Wire.write(0x01); 
  Wire.endTransmission();
  
  delay(10); // Provide time for the physical mux transistor to switch

  // --- 2. AK8963 Silicon Verification ---
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(MAG_WHOAMI_REG);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)AK8963_ADDR, (uint8_t)1);
  if (Wire.read() != 0x48) {
    #if DEBUG_MODE_ACTIVATE
      Serial.println("HAL ERROR: AK8963 NOT FOUND!");
    #endif
    while(1);
  }

  // Set Magnetometer to 16-bit resolution, 100Hz continuous measurement mode
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(MAG_CNTL1);
  Wire.write(0x16);
  Wire.endTransmission();

  // --- 3. BMP280 Silicon Verification ---
  Wire.beginTransmission(BMP280_ADDR);
  Wire.write(BMP_ID_REG);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)BMP280_ADDR, (uint8_t)1);
  if (Wire.read() != 0x58) {
    #if DEBUG_MODE_ACTIVATE
      Serial.println("HAL ERROR: BMP280 NOT FOUND!");
    #endif
    while(1);
  }

  readBMPCoefficients(); // Fetch hardware trim data

  // Set Barometer to Normal mode, 1x Oversampling for responsive altitude tracking
  Wire.beginTransmission(BMP280_ADDR);
  Wire.write(BMP_CTRL_MEAS);
  Wire.write(0x27);
  Wire.endTransmission();

  // Map the physical pin to the software ISR. Trigger on RISING edge (0V to 3.3V/5V)
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dataReadyISR, RISING);
}

void loop() {
  // Application idles until the HAL hardware interrupt asserts that new data is available
  if (dataReady) {
    dataReady = false; // Arm the flag for the next interrupt cycle

    // --- HAL READ: Clear Interrupt Status ---
    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(INT_STATUS_REG);  
    Wire.endTransmission(false); 
    Wire.requestFrom((uint8_t)MPU9250_ADDR, (uint8_t)1); 
    Wire.read(); // Reading this register resets the physical INT pin to LOW

    // --- HAL READ: MPU9250 (14-Byte Burst) ---
    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(ACCEL_XOUT_H_REG);
    Wire.endTransmission(false); // Hold the bus to guarantee synchronous data mapping
    Wire.requestFrom((uint8_t)MPU9250_ADDR, (uint8_t)14);

    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
    int16_t rawMpuTemp = (Wire.read() << 8) | Wire.read();
    gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();

    // --- HAL READ: AK8963 (7-Byte Burst) ---
    Wire.beginTransmission(AK8963_ADDR);
    Wire.write(MAG_XOUT_L); 
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)AK8963_ADDR, (uint8_t)7);

    uint8_t magData[7];
    for (int i = 0; i < 7; i++) magData[i] = Wire.read();

    // The 7th byte (ST2) contains the HOFL (Magnetic Sensor Overflow) bit
    magOverflow = (magData[6] & 0x08);
    if (!magOverflow) { // Only update middleware if data is uncorrupted
      mx = (int16_t)((magData[1] << 8) | magData[0]);
      my = (int16_t)((magData[3] << 8) | magData[2]);
      mz = (int16_t)((magData[5] << 8) | magData[4]);
    }

    // --- HAL READ: BMP280 (6-Byte Burst) ---
    Wire.beginTransmission(BMP280_ADDR);
    Wire.write(BMP_PRESS_MSB);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)BMP280_ADDR, (uint8_t)6);

    // Extract 20-bit raw precision registers
    rawPress   = ((uint32_t)Wire.read() << 12) | ((uint32_t)Wire.read() << 4) | (Wire.read() >> 4);
    uint32_t rawBmpTemp = ((uint32_t)Wire.read() << 12) | ((uint32_t)Wire.read() << 4) | (Wire.read() >> 4);

    // --- MIDDLEWARE COMPUTATIONS ---
    mpuTempC = ((float)rawMpuTemp / 333.87) + 21.0; // Silicon internal heat measurement
    
    int32_t calculatedTempC = compensate_T(rawBmpTemp); // Math execution against hardware trim
    bmpTempC = (float)calculatedTempC / 100.0; // Convert fixed-point response to standard float

    // --- APPLICATION LOGIC & TELEMETRY ---
    // (High-speed motor PID loops would execute here based on the Middleware outputs)
    
    #if DEBUG_MODE_ACTIVATE
      // Restrict serial transmission frequency to avoid mathematical stalling in the main loop
      if (millis() - lastPrintTime >= TELEMETRY_RATE_MS) {
        lastPrintTime = millis();
        
        Serial.print("Acc["); Serial.print(ax); Serial.print(","); Serial.print(ay); Serial.print(","); Serial.print(az); Serial.print("] | ");
        Serial.print("Gyro["); Serial.print(gx); Serial.print(","); Serial.print(gy); Serial.print(","); Serial.print(gz); Serial.print("] | ");
        
        if (magOverflow) {
          Serial.print("Mag[OVERFLOW!] | ");
        } else {
          Serial.print("Mag["); Serial.print(mx); Serial.print(","); Serial.print(my); Serial.print(","); Serial.print(mz); Serial.print("] | ");
        }
        
        Serial.print("Press: "); Serial.print(rawPress);
        Serial.print(" | IMU Temp: "); Serial.print(mpuTempC, 1); Serial.print("C");
        Serial.print(" | BMP Temp: "); Serial.print(bmpTempC, 1); Serial.println("C");
      }
    #endif
  }
}