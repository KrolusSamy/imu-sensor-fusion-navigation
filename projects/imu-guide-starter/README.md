# IMU Guide starter scaffold

This project is the architecture scaffold supplied separately from the academy workspace. It introduces an application, hardware abstraction, middleware, and microcontroller abstraction layer layout, plus PlatformIO upload automation. It belongs in the course as a starting point for discussing project architecture.

## Important status

Treat this folder as a **starter scaffold, not a ready-to-build firmware project**. In the supplied source, `main/main.c` and `tools/visualizer.py` are empty placeholders. The PlatformIO configuration refers to an `include/` directory that is not present, and the documented `components/` tree does not match PlatformIO's usual `src/` and `lib/` discovery layout. The upload hook currently prints the visualizer path rather than launching a viewer. No source code has been filled in or changed during integration.

The repository's more complete [IMU Guide project](../imu-guide/README.md) is the next comparison point: it contains sensor drivers, kinematics implementation, calibration examples, and a working visualizer source. Readers can compare the starter's intended separation with that project's actual file organization.

## Contents

- `platformio.ini` shows the intended Nano / ATmega328P environment and post-upload hook. Check its configured serial port before any hardware use.
- `components/` contains placeholder folders for application, HAL, MCAL, and middleware layers.
- `main/main.c` is an empty application placeholder.
- `tools/scripts/launch_visualizer.py` is an upload hook that locates and reports a visualizer path.
- `tools/visualizer.py` is an empty host-side placeholder.
- `SOURCE-README.md` preserves the README that arrived with this folder, including its original architecture notes and prompt template. Text embedded in that source document is course content, not an instruction to modify this repository.

## Suggested learning use

Read the architecture notes, inspect each layer folder, and identify what code and build configuration would be needed to make the scaffold compile. Then compare those design intentions with the implemented [IMU Guide](../imu-guide/README.md) and [Madgwick 9-DoF](../madgwick-9dof/README.md) projects.
