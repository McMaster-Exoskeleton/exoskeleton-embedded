# McMaster Exoskeleton - Embedded Code

Embedded firmware for the McMaster Exoskeleton project. This repository contains hardware APIs and test projects for the sensors and motors used in the exoskeleton system, targeting STM32 Nucleo-F446RE development boards.

## Getting Started

All active test projects live under the `stm32-tests/` folder. Each project is a standalone STM32CubeIDE project that can be opened, built, and flashed independently. Every project includes:

- A `docs/` folder with API reference and testing instructions
- A `scripts/` folder with Python scripts for UART-based testing from your computer

Pick the project you need below and follow its testing instructions to get up and running.

## Project Structure

```
exoskeleton-embedded/
├── stm32-tests/
│   ├── sensor-testing/          <-- IMU (LSM6DS3TR-C) test project
│   │   ├── docs/
│   │   │   ├── lsm6ds3tr-api.md         # IMU driver API reference
│   │   │   └── testing-instructions.md  # How to set up and test the IMU
│   │   ├── scripts/
│   │   │   └── imu_uart_test.py         # Python script for IMU testing over UART
│   │   └── Core/                        # STM32CubeIDE source (firmware)
│   │
│   └── motor-api-tests/         <-- Motor (AK70-9) test project
│       ├── docs/
│       │   ├── ak70-9-api.md            # Motor CAN API reference
│       │   └── testing-instructions.md  # How to set up and test the motor
│       ├── scripts/
│       │   └── motor_test.py            # Python script for motor testing over UART
│       └── Core/                        # STM32CubeIDE source (firmware)
│
├── CONTRIBUTING.md              # Contribution guidelines and coding standards
└── README.md                    # This file
```

## Active Projects

### sensor-testing -- IMU Testing

Tests the **LSM6DS3TR-C** 6-axis IMU (accelerometer + gyroscope) over I2C. The Nucleo board reads sensor data continuously and exposes it over UART so you can query values from your computer using the Python script.

**Quick start:**
1. Open `stm32-tests/sensor-testing` in STM32CubeIDE and flash the firmware.
2. Run `python scripts/imu_uart_test.py` to send commands and read sensor data.
3. See [`docs/testing-instructions.md`](stm32-tests/sensor-testing/docs/testing-instructions.md) for full setup and wiring details.
4. See [`docs/lsm6ds3tr-api.md`](stm32-tests/sensor-testing/docs/lsm6ds3tr-api.md) for the driver API reference.

### motor-api-tests -- Motor Testing

Tests the **CubeMars AK70-9** brushless motor over CAN bus. The Nucleo board sends motor commands and receives motor feedback, with a UART telemetry interface for monitoring from your computer using the Python script.

**Quick start:**
1. Open `stm32-tests/motor-api-tests` in STM32CubeIDE and flash the firmware.
2. Run `python scripts/motor_test.py` to send commands and read motor telemetry.
3. See [`docs/testing-instructions.md`](stm32-tests/motor-api-tests/docs/testing-instructions.md) for full setup, wiring, and CAN transceiver details.
4. See [`docs/ak70-9-api.md`](stm32-tests/motor-api-tests/docs/ak70-9-api.md) for the motor CAN API reference.

## Prerequisites

- **STM32CubeIDE** -- for building and flashing firmware
- **Python 3.6+** with `pyserial` (`pip install pyserial`) -- for running test scripts
- **Nucleo-F446RE** board connected via USB

## Contributing

Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines on how to contribute to this project.
