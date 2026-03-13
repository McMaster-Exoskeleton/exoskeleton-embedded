# Nucleo-Pi CAN Testing Instructions

Step-by-step guide for testing IMU-to-CAN transmission using the **nucleo-pi-can** project on a Nucleo-F446RE board.

This project reads accelerometer and gyroscope data from the LSM6DS3TR-C IMU via DMA-backed I2C, continuously broadcasts accelerometer data over the CAN bus (standard ID `0x123`), and stores the last 100 readings in a C++ circular buffer. A UART command interface is available for direct debugging and buffer readback.

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

Run the script with auto-detection:
```bash
python imu_uart_test.py
```

Or specify the COM port directly:
```bash
python imu_uart_test.py COM3         # Windows
python imu_uart_test.py /dev/ttyACM0 # Linux
```

You should see:
```
Auto-detected port: COM3
STM32 Nucleo F446RE - IMU UART Test
==================================================
Port:      COM3
Baud Rate: 115200
Commands:  READ, READLATEST, READALL, STATUS, REGISTER, CONFIG, POWER
Type 'quit' to exit
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
| `READ` | `AX:<val> AY:<val> AZ:<val> GX:<val> GY:<val> GZ:<val>` | Current filtered accel (m/s²) and gyro (dps) from live struct |
| `READLATEST` | `LATEST AX:<val> AY:<val> AZ:<val> GX:<val> GY:<val> GZ:<val>` | Most recent reading from the 100-sample circular buffer |
| `READALL` | `ALL:COUNT=<n>` followed by `[i] AX:... GZ:...` per line | All stored readings oldest-first from the circular buffer |
| `STATUS` | `STATUS:CONNECTED` or `STATUS:LOST` | IMU I2C connection status |
| `REGISTER` | `REGISTER:0x6B` | I2C device address |
| `CONFIG` | `GYRO_CFG:0x44 ACCEL_CFG:0x48` | Gyro and accel control register values |
| `POWER` | `POWER_CFG:0x00` | Power configuration register value |

### READALL Example Output

```
Command: READALL
Sent: READALL
  ALL:COUNT=100
  [0] AX:0.11 AY:-0.04 AZ:9.77 GX:0.01 GY:0.00 GZ:-0.01
  [1] AX:0.12 AY:-0.05 AZ:9.78 GX:0.01 GY:0.00 GZ:-0.01
  ...
  [99] AX:0.10 AY:-0.03 AZ:9.79 GX:0.00 GY:0.00 GZ:0.00
  (102 line(s) received)
```

> **Note:** `READALL` sends 100+ lines over UART at 115200 baud. Expect approximately 2–3 seconds for full output. The Python script handles this automatically.

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

### READALL returns `ALL:EMPTY`

**Cause:** The command was sent before the circular buffer accumulated any readings. This can happen if:
- The IMU is in `LOST` state (DMA callback is never triggered).
- The command was sent immediately after power-on before calibration finished.

**Fix:** Wait for `STATUS:CONNECTED`, then wait a second for the buffer to fill, then retry.

### Hard fault / firmware crash on READALL

**Possible cause:** The `IMUReading snapshot[100]` stack allocation (2400 bytes) combined with other stack usage exceeds the available stack.

**Note:** The linker scripts (`STM32F446RETX_FLASH.ld` and `STM32F446RETX_RAM.ld`) have already been updated to `_Min_Stack_Size = 0x1000` (4096 bytes) to accommodate this. If you see a hard fault after regenerating the `.ioc` in CubeMX (which resets the linker script), re-apply this change:
```
_Min_Stack_Size = 0x1000;
```

---

## Implementation Notes

### DMA vs Blocking I2C

The original `nucleo-pi-can` project used `lsm6ds3tr_read()` (blocking `HAL_I2C_Mem_Read`). This held the CPU for ~1.4 ms per read (12 bytes at 100 kHz I2C). With DMA:

- `lsm6ds3tr_init_dma_read()` issues the transfer and returns immediately (<10 µs).
- The CPU is free for CAN transmission and UART command handling while the 12-byte transfer completes.
- `HAL_I2C_MemRxCpltCallback()` fires from the DMA ISR when the transfer is done, processes the bytes, and pushes to the buffer.

This is effective for this use case: the 20 ms main-loop delay means the DMA transfer (takes ~1.4 ms) is always complete before the next call.

### Circular Buffer Data Structure

A **ring buffer with a static array** was chosen over alternatives:

| Structure | Push O(1) | Pop O(1) | No heap alloc | ISR-safe copy |
|---|---|---|---|---|
| `std::deque` | Yes | Yes | **No** (heap) | No |
| `std::array` + manual index | Yes | Yes | Yes | Yes |
| Linked list | O(1) amortized | Yes | **No** (heap) | No |
| **Ring buffer (chosen)** | **Yes** | **Yes** | **Yes** | **Yes** |

Heap allocation is avoided on bare-metal firmware because `malloc`/`new` can fragment SRAM and fail unpredictably. The static 100-entry array uses 2400 bytes of `.bss`.

### ISR Safety

`HAL_I2C_MemRxCpltCallback` runs from the DMA ISR and calls `imu_buffer_push()`. The UART command handler runs from `main()`. To prevent a torn read (push happening mid-copy), `process_command()` disables `DMA1_Stream1_IRQn` around the `imu_buffer_get_latest()` and `imu_buffer_get_all()` calls, then re-enables it before the (slow) UART transmission.

### Calibration

`lsm6ds3tr_calibrate()` runs once at startup for ~0.3 s (100 samples × 3 ms delay). Keep the sensor **stationary** during power-on. Calibration computes gyroscope bias offsets only; accelerometer offset calibration would require knowing the orientation axis and is left for a future iteration.
