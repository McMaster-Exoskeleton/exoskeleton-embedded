# discoveryIMUTest - Usage Guide

## Important Note

The code in this directory contains **user-level implementations only**. To successfully use these files, you must:

1. **Use the complete CubeIDE project** - This code is designed to work within the full STM32CubeIDE project environment with all associated configuration files.

2. **Reference board documentation** - Ensure you have access to the STM32 Discovery board documentation and pinout diagrams specific to your board variant.

3. **Include all environment setup files** - The project requires all generated CubeIDE files and configuration documents to function properly.

These files (`main.c`, `mpu9250.c`, and their headers) represent only the user code implementation layer. They should be integrated into the complete CubeIDE project structure, not used standalone.

## Files Included

- `main.c` / `main.h` - Main application entry point
- `mpu9250.c` / `mpu9250.h` - MPU9250 IMU sensor implementation

For setup and configuration details, refer to your STM32CubeIDE project documentation and board reference materials.
