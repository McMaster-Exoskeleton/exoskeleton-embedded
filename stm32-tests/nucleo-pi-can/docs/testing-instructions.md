# Nucleo-Pi CAN Testing Instructions

Step-by-step guide for testing IMU-to-CAN transmission using the **nucleo-pi-can** project on a Nucleo-F446RE board.

This project reads accelerometer data from the LSM6DS3TR-C IMU over I2C and continuously broadcasts it over the CAN bus (standard ID `0x123`). A UART command interface is also available for direct debugging.

---

## Hardware Requirements

- STM32 Nucleo-F446RE development board
- LSM6DS3TR-C IMU breakout board
- CAN transceiver module (e.g., SN65HVD230 or MCP2551) -- the Nucleo does not have a built-in CAN transceiver
- CAN receiver (e.g., Raspberry Pi with MCP2515 HAT, USB-CAN adapter, or second Nucleo)
- USB Mini-B cable (for connecting the Nucleo to your computer)
- Jumper wires
- Computer running Windows (or Linux/macOS)

### Wiring

#### IMU (I2C3)

| LSM6DS3TR-C Pin | Nucleo F446RE Pin | Description |
|---|---|---|
| VCC | 3.3V | Power supply |
| GND | GND | Ground |
| SDA | PA8 (I2C3_SDA) | I2C data line |
| SCL | PC9 (I2C3_SCL) | I2C clock line |
| SA0 | 3.3V | Sets I2C address to 0x6B |

> **Note:** In this project SA0 is connected to 3.3V, setting the 7-bit I2C address to `0x6B`. This differs from the sensor-testing project where SA0 is tied to GND (`0x6A`).

#### CAN Bus (CAN1)

The Nucleo's CAN1 peripheral outputs CAN_TX and CAN_RX at 3.3V logic level. A CAN transceiver is required to convert this to the differential CAN bus signals (CANH/CANL).

| Nucleo F446RE Pin | CAN Transceiver Pin | Description |
|---|---|---|
| PA11 (CAN1_RX) | RXD / CAN_RX | CAN receive |
| PA12 (CAN1_TX) | TXD / CAN_TX | CAN transmit |
| 3.3V | VCC | Power supply |
| GND | GND | Ground |

| CAN Transceiver Pin | CAN Receiver | Description |
|---|---|---|
| CANH | CANH | CAN bus high |
| CANL | CANL | CAN bus low |

> **Note:** Terminate the CAN bus with 120-ohm resistors at each end. Many transceiver modules include an onboard termination resistor enabled by a jumper.

#### UART (Debug/Command Interface)

UART2 is routed through the ST-Link debugger and appears as a virtual COM port. No additional wiring is needed -- just connect the USB cable.

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
3. Navigate to `stm32-tests/nucleo-pi-can` and select the folder.
4. Click **Finish** to import the project.

### Enabling Float Formatting (Required)

The firmware uses `sprintf` with `%f` format specifiers for UART responses. By default, the STM32 linker does not include float support for printf/sprintf. You **must** enable it:

1. Right-click the project in the **Project Explorer** > **Properties**.
2. Go to **C/C++ Build > Settings > MCU GCC Linker > Miscellaneous**.
3. In the **Other flags** field, add:
   ```
   -u _printf_float
   ```
4. Click **Apply and Close**.

> **If you skip this step**, all `READ` responses will show `0.00` for every value.

### Building and Flashing

1. Click the **Build** button (hammer icon) or press `Ctrl+B`.
2. Connect the Nucleo board via USB.
3. Click the **Run** button (green play icon) or press `F11` to flash the firmware.
4. The board will reset and begin running.

---

## Finding Your COM Port

The Nucleo board creates a virtual COM port through its ST-Link debugger.

### Windows

1. Press `Win + X` and select **Device Manager**.
2. Expand the **Ports (COM & LPT)** section.
3. Look for **STMicroelectronics STLink Virtual COM Port (COMx)**.
4. Note the port number (e.g., `COM3`, `COM5`).

> **Tip:** If you don't see it, try unplugging and replugging the USB cable. If the port still doesn't appear, install the ST-Link USB driver from [ST's website](https://www.st.com/en/development-tools/stsw-link009.html).

### Linux

```bash
ls /dev/ttyACM*
```
The port is typically `/dev/ttyACM0`.

### macOS

```bash
ls /dev/cu.usbmodem*
```

---

## Testing Instructions

### Step 1: Set Up Hardware

1. Wire the IMU to the Nucleo as described in the [IMU wiring table](#imu-i2c3) above.
2. Wire the CAN transceiver between the Nucleo and your CAN receiver.
3. Connect the Nucleo board to your computer via USB.

### Step 2: Flash the Firmware

Follow the [STM32CubeIDE Setup](#stm32cubeide-setup) section above to build and flash the firmware.

### Step 3: Identify the COM Port

Follow the [Finding Your COM Port](#finding-your-com-port) section above.

### Step 4: Run the Python Debug Script

Open a terminal and navigate to the `nucleo-pi-can` project folder:

```bash
cd stm32-tests/nucleo-pi-can
```

Run the script (hardcoded to `COM3` by default -- edit the `PORT` variable in the script to match your port):
```bash
python imu_uart_test.py
```

You should see:
```
STM32 Nucleo F446RE - IMU UART Test
==================================================
Port: COM3
Baud Rate: 115200
Available commands: READ, STATUS, REGISTER, CONFIG, POWER
==================================================
Connected to COM3

Enter commands (Ctrl+C to exit):
--------------------------------------------------
```

### Step 5: Verify the IMU Connection

Type `STATUS` and press Enter:
```
Command: STATUS
Sent: STATUS
Response: STATUS:CONNECTED
```

If you see `STATUS:LOST`, check your I2C wiring and ensure SA0 is connected to 3.3V.

### Step 6: Read IMU Data over UART

Type `READ` and press Enter:
```
Command: READ
Sent: READ
Response: AX:0.12 AY:-0.05 AZ:9.78 GX:0.03 GY:-0.01 GZ:0.02
```

### Step 7: Verify CAN Transmission

While the firmware is running, IMU data is continuously sent on CAN ID `0x123` every ~20 ms. Verify reception on your CAN receiver.

For a Raspberry Pi with a CAN interface (e.g., MCP2515 HAT):
```bash
candump can0
```

Expected output:
```
can0  123   [6]  00 0C FF D3 03 D2
```

See the [CAN Frame Format](can-protocol.md#can-frame-format) section in the protocol doc for how to decode the bytes.

### Step 8: Exit

Press `Ctrl+C` to close the Python script.

---

## Available UART Commands

| Command | Response Format | Description |
|---|---|---|
| `READ` | `AX:<val> AY:<val> AZ:<val> GX:<val> GY:<val> GZ:<val>` | Filtered accel (m/s²) and gyro (dps) readings |
| `STATUS` | `STATUS:CONNECTED` or `STATUS:LOST` | IMU I2C connection status |
| `REGISTER` | `REGISTER:0x6B` | I2C device address |
| `CONFIG` | `GYRO_CFG:0x44 ACCEL_CFG:0x48` | Gyro and accel control register values |
| `POWER` | `POWER_CFG:0x00` | Power configuration register value |

---

## Common Errors

### STATUS returns LOST

**Possible causes:**
- I2C wires are not connected or connected to the wrong pins.
- SA0 pin is not connected to 3.3V (address mismatch -- this project uses `0x6B`, not `0x6A`).
- The IMU board is not powered (check 3.3V connection).

**Fix:** Double-check all wiring against the [IMU wiring table](#imu-i2c3) above.

### All READ values are 0.00

**Cause:** The `-u _printf_float` linker flag is missing.

**Fix:** Follow the [Enabling Float Formatting](#enabling-float-formatting-required) section above.

### No CAN frames received

**Possible causes:**
- CAN transceiver not wired correctly.
- CAN bus not terminated with 120-ohm resistors at both ends.
- Firmware not flashed, or IMU is in `LOST` state (CAN transmission is skipped when the IMU is disconnected).
- CAN receiver interface not configured for 1 Mbit/s.

**Fix:** Verify STATUS is `CONNECTED`, check all CAN wiring, and confirm bus termination and baud rate.

### "No serial ports found" when running the Python script

**Possible causes:**
- The Nucleo board is not plugged in.
- The ST-Link USB driver is not installed.
- Another program has the port open.

**Fix:** Close any other programs using the COM port, then retry.

### "Could not open COMx: Access denied"

**Cause:** Another application already has the COM port open.

**Fix:** Close STM32CubeIDE's built-in serial monitor, PuTTY, or any other terminal connected to the same port.

### No response received after sending a command

**Possible causes:**
- The firmware was not flashed successfully.
- The board was not reset after flashing.

**Fix:** Re-flash the firmware and press the black reset button on the Nucleo board.
