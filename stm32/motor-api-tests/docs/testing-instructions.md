# Motor API Testing Instructions

Step-by-step guide for testing the AK70-9 motor using the **motor-api-tests** project on a Nucleo-F446RE board.

---

## Hardware Requirements

- STM32 Nucleo-F446RE development board
- CubeMars AK70-9 KV60 motor
- CAN transceiver module (e.g., SN65HVD230 or MCP2551) -- the Nucleo does not have a built-in CAN transceiver
- Appropriate power supply for the AK70-9 motor
- USB Mini-B cable (for connecting the Nucleo to your computer)
- Jumper wires
- Computer running Windows (or Linux/macOS)

### Wiring

#### CAN Bus (CAN1)

The Nucleo's CAN1 peripheral outputs CAN_TX and CAN_RX at 3.3V logic level. A CAN transceiver is required to convert this to the differential CAN bus signals (CANH/CANL).

| Nucleo F446RE Pin | CAN Transceiver Pin | Description |
|---|---|---|
| PA11 (CAN1_RX) | RXD / CAN_RX | CAN receive |
| PA12 (CAN1_TX) | TXD / CAN_TX | CAN transmit |
| 3.3V | VCC | Power supply |
| GND | GND | Ground |

| CAN Transceiver Pin | AK70-9 Motor | Description |
|---|---|---|
| CANH | CANH | CAN bus high |
| CANL | CANL | CAN bus low |

> **Note:** Ensure the CAN bus is properly terminated with 120-ohm resistors at each end. Many CAN transceiver modules include an onboard termination resistor that can be enabled with a jumper.

#### UART (Telemetry)

UART2 is routed through the ST-Link debugger on the Nucleo and appears as a virtual COM port on your computer. No additional wiring is needed -- just connect the USB cable.

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
3. Navigate to `stm32-tests/motor-api-tests` and select the folder.
4. Click **Finish** to import the project.

### Enabling Float Formatting (Required)

The firmware uses `snprintf` with `%f` format specifiers to send floating-point motor data over UART. By default, the STM32 linker does not include float support for printf/snprintf. You **must** enable it:

1. Right-click the project in the **Project Explorer** > **Properties**.
2. Go to **C/C++ Build > Settings > MCU GCC Linker > Miscellaneous**.
3. In the **Other flags** field, add:
   ```
   -u _printf_float
   ```
4. Click **Apply and Close**.

> **If you skip this step**, all float values in responses (position, speed, current) will show as `0.00` or the firmware may crash.

### Building and Flashing

1. Click the **Build** button (hammer icon) or press `Ctrl+B`.
2. Connect the Nucleo board via USB.
3. Click the **Run** button (green play icon) or press `F11` to flash the firmware.
4. The board will reset and begin running.

---

## Finding Your COM Port

The Nucleo board creates a virtual COM port through its ST-Link debugger.

### Windows

1. Press `Win + X` and select **Device Manager** (or search for "Device Manager" in the Start menu).
2. Expand the **Ports (COM & LPT)** section.
3. Look for **STMicroelectronics STLink Virtual COM Port (COMx)** where `x` is the port number.
4. Note this port number (e.g., `COM3`, `COM5`).

> **Tip:** If you don't see it, try unplugging and replugging the USB cable. If the port still doesn't appear, you may need to install the ST-Link USB driver from [ST's website](https://www.st.com/en/development-tools/stsw-link009.html).

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

1. Wire the CAN transceiver between the Nucleo and the AK70-9 motor as described in the [Wiring](#wiring) section.
2. Power the AK70-9 motor with its appropriate power supply.
3. Connect the Nucleo board to your computer via USB.

### Step 2: Flash the Firmware

Follow the [STM32CubeIDE Setup](#stm32cubeide-setup) section above to build and flash the firmware.

### Step 3: Identify the COM Port

Follow the [Finding Your COM Port](#finding-your-com-port) section above.

### Step 4: Run the Python Test Script

Open a terminal and navigate to the `motor-api-tests` project folder:

```bash
cd stm32-tests/motor-api-tests
```

Run the script with auto-detection:
```bash
python scripts/motor_test.py
```

Or specify the COM port directly:
```bash
python scripts/motor_test.py COM5        # Windows
python scripts/motor_test.py /dev/ttyACM0 # Linux
```

You should see:
```
Auto-detected port: COM5
Connected to COM5 at 115200 baud

Available commands:
  PING       - Test connection
  READ_ALL   - Read all motor values
  READ_POS   - Read position (degrees)
  READ_SPD   - Read speed (ERPM)
  READ_CUR   - Read current (amps)
  READ_TEMP  - Read temperature (C)
  READ_ERR   - Read error code
  quit       - Exit
```

### Step 5: Test the Connection

Type `PING` and press Enter:
```
> PING
  <- PONG
```

If you see `PONG`, the UART connection is working.

### Step 6: Read Motor Data

Read all motor feedback values:
```
> READ_ALL
  <- ALL:POS=0.00,SPD=0.0,CUR=0.00,TEMP=25,ERR=0
```

Read individual values:
```
> READ_POS
  <- POS:0.00

> READ_TEMP
  <- TEMP:25
```

### Step 7: Monitor for Errors

The script automatically displays motor error notifications prominently:
```
*** MOTOR ERROR [2]: OVER_CURRENT ***
```

Check the error state at any time:
```
> READ_ERR
  <- ERR:0:NONE
```

### Step 8: Exit

Type `quit` or press `Ctrl+C` to close the script.

---

## Available Commands

| Command | Response Format | Description |
|---|---|---|
| `PING` | `PONG` | Test UART connection |
| `READ_ALL` | `ALL:POS=<deg>,SPD=<rpm>,CUR=<A>,TEMP=<C>,ERR=<code>` | All motor feedback values |
| `READ_POS` | `POS:<degrees>` | Motor position in degrees |
| `READ_SPD` | `SPD:<erpm>` | Motor speed in electrical RPM |
| `READ_CUR` | `CUR:<amps>` | Motor current in amps |
| `READ_TEMP` | `TEMP:<celsius>` | Driver board temperature in C |
| `READ_ERR` | `ERR:<code>:<description>` | Error code with description |

### Proactive Notifications

The firmware automatically sends error notifications when the motor's error state changes:

```
!ERR:<code>:<description>
```

These are displayed by the Python script as:
```
*** MOTOR ERROR [<code>]: <description> ***
```

---

## Common Errors

### All float values are 0.00

**Cause:** The `-u _printf_float` linker flag is missing.

**Fix:** Follow the [Enabling Float Formatting](#enabling-float-formatting-required) section above.

### PING returns no response

**Possible causes:**
- Firmware was not flashed successfully.
- The COM port is incorrect.
- Another program has the COM port open.

**Fix:** Re-flash the firmware, verify the correct COM port, and close any other serial monitors.

### "No serial ports found" when running the Python script

**Possible causes:**
- The Nucleo board is not plugged in.
- The ST-Link USB driver is not installed.
- Another program has the port open.

**Fix:** Close any other programs using the COM port, then retry.

### "Could not open COMx: Access denied"

**Cause:** Another application already has the COM port open.

**Fix:** Close STM32CubeIDE's built-in serial monitor, PuTTY, or any other terminal connected to the same port.

### Motor error notifications appearing

See the [Motor Error Codes table in the API documentation](ak70-9-api.md#motor-error-codes) for error descriptions.

Common causes:
- **OVER_TEMP / MOSFET_OVER_TEMP:** Motor or driver overheating. Reduce load or add cooling.
- **OVER_CURRENT:** Current draw exceeds 60A. Reduce torque/duty command.
- **UNDER_VOLTAGE:** Power supply voltage too low. Check power supply connections.
- **ENCODER_FAULT:** Encoder issue. Check motor cable connections.
- **MOTOR_STALL:** Motor is physically blocked or load is too high.

### CAN bus not receiving data (all READ values stay at 0)

**Possible causes:**
- CAN transceiver not wired correctly.
- CAN bus not terminated (needs 120-ohm resistors).
- Motor driver ID mismatch (check the motor's configured CAN ID).
- Motor not powered.

**Fix:** Verify all CAN wiring, ensure bus termination, and check that the motor's power supply is connected and turned on.
