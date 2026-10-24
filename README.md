# IMU Sensor Fusion and Autonomous Navigation Academy

This repository is the teaching material for a three-session academy on inertial measurement units (IMUs), embedded sensor interfacing, calibration, orientation, and sensor fusion. It keeps the original Arduino sketches and Python tools intact while organizing them as a course and as standalone PlatformIO projects.

## Learning path

1. [Session 1: Foundations](sessions/01-foundations/README.md) introduces 3D orientation, MEMS sensor behavior, communications, and visualization.
2. [Session 2: Register access](sessions/02-register-access/README.md) moves from sensor identity and diagnostics into raw measurements, hardware filtering, calibration, and deterministic timing.
3. [Session 3: Calibration and fusion](sessions/03-calibration-and-fusion/README.md) connects signal filtering, magnetometer calibration, compass math, quaternions, and sensor fusion.

## Repository layout

```text
.
├── docs/                         Course outline and preserved Word source
├── sessions/                     Slides and small lesson examples by session
├── projects/                     Complete PlatformIO project workspaces
│   ├── gy91-platformio/          GY-91 HAL and kinematics demonstration
│   ├── imu-guide-starter/        Architecture scaffold for the IMU guide
│   ├── imu-guide/                Implemented modular IMU guide project
│   └── madgwick-9dof/            Nine degree of freedom Madgwick project
└── README.md
```

Each educational directory has its own README. Start with the README at the relevant session or project before opening source files.

## PlatformIO projects

The folders under [`projects/`](projects/README.md) are independent PlatformIO workspaces or teaching scaffolds. The `imu-guide-starter` folder is intentionally documented as an architecture exercise and is not ready to build as supplied. The other three folders are independent PlatformIO workspaces. Open a project folder (the folder containing `platformio.ini`) in VS Code with PlatformIO, review its README and serial/upload settings, then build or upload for the configured board. The existing configuration targets an Arduino Nano / ATmega328P; serial ports and attached hardware may need to match your own setup. Build outputs are excluded from version control.

## Hardware and tools

The examples refer to MPU-6050 and GY-91-class sensor boards. Check each lesson and project README for its specific assumptions before wiring hardware. Python tools may use serial communication and visualization libraries; their individual README files describe their role. No source code was modified as part of this educational reorganization.

## Course source material

The original curriculum document is preserved at [docs/reference-materials/academy-curriculum.docx](docs/reference-materials/academy-curriculum.docx). The [course outline](docs/curriculum.md) turns its three lectures into a repository navigation guide. Original slide decks remain with their corresponding session.


## Large presentation files

The original PowerPoint decks exceed GitHub's normal per-file upload limit. This repository marks `*.pptx` for Git LFS in `.gitattributes`. Install Git LFS and run `git lfs install` before adding and committing the decks; confirm that the target GitHub account or organization has sufficient LFS storage and bandwidth for these files.
