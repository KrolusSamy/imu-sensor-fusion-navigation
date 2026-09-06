# IMU Sensor Fusion

**Instructor:** Krolus Hebiesh  
**Target Audience:** Students or hobbies who want to know more about the Inertial Measurement Unit (**IMU**) & Autonomous Navigation preparing for autonomous robotics, UAV, UGV, and Formula Student competitions.  

---

## 📂 Project Directory Structure
The repository is organized into modular directories:

```text
IMU_Guide/
├── .pio/
├── .vscode/
├── components/
│   ├── app/
│   │   ├── include/
│   │   └── scr/
│   ├── hal/
│   │   ├── include/
│   │   └── scr/
│   ├── mcal/                 # Optional / Out of scope (low-level silicon register drivers)
│   │   ├── include/
│   │   └── scr/
│   └── middleware/
│   │   ├── include/
│   │   └── scr/
├── main/
│   └── main.c
├── tools/
│   └── scripts/
│       └── launch_visualizer.py
├── visualizer.py
├── platformio.ini
└── README.md
```

## 🏛️ The Modular Three-Layer Architecture
To avoid the chaos of monolithic codebases, this architecture separates hardware interfacing from mathematical processing and high-level execution. *Note: Microcontroller Abstraction Layer (MCAL) management is treated as optional or out-of-scope, leveraging standard Arduino libraries (like `Wire.h`) within the HAL layer.*

* **Layer 1: The Hardware Abstraction Layer (HAL) (`components/hal/`)**
  * **The Fetcher:** Manages physical interactions with external hardware components (such as the IMU). It handles I2C bus communication, register maps, and raw 16-bit data extraction. It performs zero complex math, simply packaging raw registers into clean structures.
* **Layer 2: The Middleware / Service Layer (`components/middleware/`)**
  * **The Cruncher:** Stripped down to focus purely on mathematics—including Kinematics, Euler integration, and Sensor Fusion algorithms. It is entirely hardware-agnostic and decoupled from specific microcontroller pins or communication protocols.
* **Layer 3: The Application Layer (`components/app/` & `main/`)**
  * **The Orchestrator:** Controls the main execution flow (`main.c`). It calls the HAL to retrieve sensor data, feeds it to the Middleware for processing, and executes high-level system decisions and telemetry reporting.



## 🐍 Python Integration & Automated Visualization
Beyond standard C++ compilation, PlatformIO runs on SCons, enabling seamless Python automation integrated directly into the firmware build cycle.

### How Python is Integrated
1. **Post-Upload Automation Hook (`platformio.ini`):** 
   PlatformIO is configured to automatically trigger a post-build script the moment firmware successfully flashes to the target hardware.
2. **The Automation Script (`tools/scripts/launch_visualizer.py`):** 
   Registers a callback that executes immediately after uploading, using Python's `subprocess` and PlatformIO's interpreter path (`sys.executable`) to spawn the PC-side visualizer.
3. **The 3D Visualizer (`visualizer.py`):** 
   Catches streaming CSV telemetry over serial using `pyserial` and renders real-time orientation models using `pygame`.

### PlatformIO Configuration (`platformio.ini`)
```ini
; ==============================================================================
; PlatformIO Project Configuration File
; ==============================================================================

[env]
framework = arduino
monitor_speed = 115200
monitor_filters = time, default
build_flags = 
    -I include/

[env:nanoatmega328]
platform = atmelavr
board = nanoatmega328
board_build.mcu = atmega328p
board_build.f_cpu = 16000000L

upload_protocol = arduino
upload_port = COM9
upload_speed = 115200

; Automatically runs Python script to launch the visualizer immediately after flashing
extra_scripts = post:tools/scripts/launch_visualizer.py
```
### Developer Quick-Reference: Python Environment Setup
To install required PC telemetry dependencies inside PlatformIO's isolated environment, execute this command directly in your VS Code terminal:

```bash
& "C:\Users\'USER_NAME'\.platformio\penv\Scripts\python.exe" -m pip install pygame pyserial
```
## AI Prompt Template: Monolithic Code Refactoring
This might help if you want to change your file to the previous strcture. 

```bash
Act as an expert Embedded Systems Firmware Engineer specializing in high-performance microcontroller architecture and aerospace sensor integration. 

You will help me refactor and divide a single monolithic Arduino/C++ firmware file into our professional modular architecture (HAL, Middleware/Service, and Application Layers) based on the specs below. 

Important Collaborative Rule: Do not write the full code immediately. First, analyze the monolithic file I will provide, break down your proposed partitioning strategy (identifying exactly what moves to HAL, Middleware, and main.c), and wait for my approval or edits before generating the final code blocks.


### Strict Engineering & Coding Standards to Follow:

1. File Structure & Architecture Separation
   * HAL Layer: Isolate all physical hardware interactions, I2C/SPI addresses, register maps, and raw data extraction. No complex math here.
   * Middleware Layer: Isolate pure mathematical modeling, kinematics, zero-rate bias calibration, and sensor fusion algorithms. Keep it entirely hardware-agnostic.
   * Application Layer (main.c / main.cpp): The orchestrator managing setup initialization, ISR routines, and non-blocking loops.
   * Visually divide code blocks using clear comment banners (e.g., // === [ HAL LAYER: I2C BURST EXTRACTION ] ===).

2. Commenting & Documentation Standards
   * Doxygen Headers: Include @file, @author, @version, @date, and @brief.
   * Silicon-Level & Physics Explanations: Inline comments must explain the hardware reality and physics (e.g., explaining bit configurations on internal PLL clocks or register states).

3. Performance & I2C Resilience Standards
   * Hardware Throttling: Configure internal Sample Rate Dividers and Digital Low Pass Filters (DLPF) rather than relying solely on millis().
   * Interrupt Safety: Configure latched interrupts and status register clear calls to prevent missed edges.
   * I2C Resilience: Implement 400kHz Fast Mode (Wire.setClock(400000)), wire timeout guards (Wire.setWireTimeout()), and single burst reads to prevent data tearing.
   * Debug Wrappers: Wrap debugging telemetry inside #if DEBUG_MODE_ACTIVATE conditional compilation blocks.

---

Here is my monolithic code to analyze. Please provide your architectural breakdown plan first and wait for my go-ahead:

[PASTE YOUR MONOLITHIC CODE HERE WHEN READY]
```