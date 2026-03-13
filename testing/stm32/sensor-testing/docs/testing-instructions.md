# Sensor Testing Instructions

Step-by-step guide for testing the LSM6DS3TR-C IMU using the **sensor-testing** project on a Nucleo-F446RE board.

---

## Hardware Requirements

- STM32 Nucleo-F446RE development board
- LSM6DS3TR-C IMU breakout board
- USB Mini-B cable (for connecting the Nucleo to your computer)
- Jumper wires for I2C connection
- Computer running Windows (or Linux/macOS)

### Wiring (I2C3)

| LSM6DS3TR-C Pin | Nucleo F446RE Pin | Description |
|---|---|---|
| VCC | 3.3V | Power supply |
| GND | GND | Ground |
| SDA | PA8 (I2C3_SDA) | I2C data line |
| SCL | PC9 (I2C3_SCL) | I2C clock line |
| SA0 | GND | Sets I2C address to 0x6A |

> **Note:** Ensure SA0 is connected to GND. This sets the 7-bit I2C address to `0x6A` (binary `1101010`). If SA0 is left floating or connected to VCC, the address will be `0x6B` and the driver will not detect the sensor.

---

## Computer Requirements

- **Python 3.6+** installed and accessible from the terminal
- **pyserial** Python package:
  ```
  pip install pyserial
  ```
- **STM32CubeIDE** (for building and flashing the firmware)
- A USB driver for the ST-Link debugger (usually installed automatically with CubeIDE)

---

## STM32CubeIDE Setup

### Opening the Project

1. Open STM32CubeIDE.
2. Go to **File > Open Projects from File System**.
3. Navigate to `stm32-tests/sensor-testing` and select the folder.
4. Click **Finish** to import the project.

### Enabling Float Formatting (Required)

The firmware uses `sprintf` with `%f` format specifiers to send floating-point sensor data over UART. By default, the STM32 linker does not include float support for printf/sprintf. You **must** enable it:

1. Right-click the project in the **Project Explorer** > **Properties**.
2. Go to **C/C++ Build > Settings > MCU GCC Linker > Miscellaneous**.
3. In the **Other flags** field, add:
   ```
   -u _printf_float
   ```
4. Click **Apply and Close**.

> **If you skip this step**, all `READ` responses will show `0.00` for every value, or the firmware may crash.

### Building and Flashing

1. Click the **Build** button (hammer icon) or press `Ctrl+B`.
2. Connect the Nucleo board via USB.
3. Click the **Run** button (green play icon) or press `F11` to flash the firmware.
4. The board will reset and begin running. The onboard LED (LD2) should be on.

---

## Finding Your COM Port

The Nucleo board creates a virtual COM port through its ST-Link debugger. You need to find this port to connect the Python script.

### Windows

1. Press `Win + X` and select **Device Manager** (or search for "Device Manager" in the Start menu).
2. Expand the **Ports (COM & LPT)** section.
3. Look for **STMicroelectronics STLink Virtual COM Port (COMx)** where `x` is the port number.
4. Note this port number (e.g., `COM3`, `COM5`).

> **Tip:** If you don't see it, try unplugging and replugging the USB cable. If the port still doesn't appear, you may need to install the ST-Link USB driver from [ST's website](https://www.st.com/en/development-tools/stsw-link009.html).

### Linux

Run the following command:
```bash
ls /dev/ttyACM*
```
The port is typically `/dev/ttyACM0`.

### macOS

Run the following command:
```bash
ls /dev/cu.usbmodem*
```

---

## Testing Instructions

### Step 1: Flash the Firmware

Follow the [STM32CubeIDE Setup](#stm32cubeide-setup) section above to build and flash the firmware onto the Nucleo board.

### Step 2: Identify the COM Port

Follow the [Finding Your COM Port](#finding-your-com-port) section above.

### Step 3: Run the Python Test Script

Open a terminal and navigate to the `sensor-testing` project folder:

```bash
cd stm32-tests/sensor-testing
```

Run the script with auto-detection:
```bash
python scripts/imu_uart_test.py
```

Or specify the COM port directly:
```bash
python scripts/imu_uart_test.py COM3        # Windows
python scripts/imu_uart_test.py /dev/ttyACM0 # Linux
```

You should see:
```
Auto-detected port: COM3
STM32 Nucleo F446RE - IMU UART Test
==================================================
Port: COM3
Baud Rate: 115200
Available commands: READ, STATUS, REGISTER, CONFIG, POWER
Type 'quit' to exit
==================================================
Connected to COM3

Enter commands:
--------------------------------------------------
```

### Step 4: Test the Connection

Type `STATUS` and press Enter:
```
Command: STATUS
Sent: STATUS
Response: STATUS:CONNECTED
```

If you see `STATUS:LOST`, check your I2C wiring and ensure SA0 is connected to GND.

### Step 5: Read Sensor Data

Type `READ` and press Enter:
```
Command: READ
Sent: READ
Response: AX:0.12 AY:-0.05 AZ:9.78 GX:0.03 GY:-0.01 GZ:0.02
```

- **AX, AY, AZ** = Filtered accelerometer values in m/s^2 (expect ~9.8 on the Z-axis when flat)
- **GX, GY, GZ** = Filtered gyroscope values in degrees/second (expect ~0 when stationary)

### Step 6: Exit

Type `quit` or press `Ctrl+C` to close the script.

---

## Available Commands

| Command | Response Format | Description |
|---|---|---|
| `READ` | `AX:<val> AY:<val> AZ:<val> GX:<val> GY:<val> GZ:<val>` | Filtered accel (m/s^2) and gyro (dps) readings |
| `STATUS` | `STATUS:CONNECTED` or `STATUS:LOST` | IMU I2C connection status |
| `REGISTER` | `REGISTER:0x6A` | I2C device address |
| `CONFIG` | `GYRO_CFG:0x44 ACCEL_CFG:0x48` | Gyro and accel control register values |
| `POWER` | `POWER_CFG:0x00` | Power configuration register value |

---

## Common Errors

### All READ values are 0.00

**Cause:** The `-u _printf_float` linker flag is missing.

**Fix:** Follow the [Enabling Float Formatting](#enabling-float-formatting-required) section above.

### STATUS returns LOST

**Possible causes:**
- I2C wires are not connected or connected to the wrong pins.
- SA0 pin is not connected to GND (address mismatch).
- The IMU board is not powered (check 3.3V connection).

**Fix:** Double-check all wiring against the [Wiring table](#wiring-i2c3) above.

### "No serial ports found" when running the Python script

**Possible causes:**
- The Nucleo board is not plugged in.
- The ST-Link USB driver is not installed.
- Another program (CubeIDE serial monitor, PuTTY, etc.) has the port open.

**Fix:** Close any other programs that might be using the COM port, then retry.

### "Could not open COMx: Access denied"

**Cause:** Another application already has the COM port open.

**Fix:** Close STM32CubeIDE's built-in serial monitor, PuTTY, or any other terminal that may be connected to the same port.

### "No response received" after sending a command

**Possible causes:**
- The firmware was not flashed successfully.
- The board was not reset after flashing.

**Fix:** Re-flash the firmware and press the black reset button on the Nucleo board.

### Calibration issues (readings drift or seem offset)

**Cause:** The IMU was moving during the startup calibration (first ~300 ms after power-on).

**Fix:** Keep the sensor stationary for at least 1 second after powering on or resetting the board. The driver takes 100 samples during startup to calculate offsets.
