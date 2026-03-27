# CAN API Integration Testing Instructions

Step-by-step guide for testing the CAN API across the full exoskeleton network, scaling from a single link to the complete 5-node system. Each phase builds on the previous one, verifying one new thing at a time.

---

## Hardware Requirements

- Raspberry Pi (3B+ or 4) with MCP2515 CAN HAT
- 4x STM32 Nucleo-F446RE development boards
- 4x CubeMars AK70-9 KV60 motors with power supplies
- 4x LSM6DS3TR-C IMU breakout boards
- 5x CAN transceiver modules (SN65HVD230 or MCP2551) -- one per node including the Pi
- 2x 120-ohm termination resistors (1/4W or higher)
- USB hub (4+ ports, for connecting multiple Nucleos to one PC)
- USB Mini-B cables (one per Nucleo)
- Jumper wires
- Computer running Windows (or Linux/macOS)
- Breadboard or prototype board for the CAN bus trunk

> **Note:** You do not need all the hardware at once. The phases are designed so you can start with just the Pi, one Nucleo, and one transceiver per side.

### Wiring

#### CAN Bus Per Nucleo (CAN1)

Each Nucleo connects to the CAN bus through its own transceiver module.

| Nucleo F446RE Pin | CAN Transceiver Pin | Description |
|---|---|---|
| PA11 (CAN1_RX) | RXD / CAN_RX | CAN receive |
| PA12 (CAN1_TX) | TXD / CAN_TX | CAN transmit |
| 3.3V | VCC | Power supply |
| GND | GND | Ground |

#### IMU Per Nucleo (I2C3)

| LSM6DS3TR-C Pin | Nucleo F446RE Pin | Description |
|---|---|---|
| VCC | 3.3V | Power supply |
| GND | GND | Ground |
| SDA | PA8 (I2C3_SDA) | I2C data line |
| SCL | PC9 (I2C3_SCL) | I2C clock line |
| SA0 | 3.3V | Sets I2C address to 0x6B |

#### Motor Per Nucleo

| CAN Transceiver Pin | AK70-9 Motor | Description |
|---|---|---|
| CANH | CANH | CAN bus high |
| CANL | CANL | CAN bus low |

> **Note:** The motor shares the same CAN bus as the inter-node network. The hardware filters in `can_common_init()` separate standard frames (network messages) from extended frames (VESC motor protocol) into different FIFOs.

#### UART (Debug/Command Interface)

UART2 is routed through the ST-Link debugger on each Nucleo and appears as a virtual COM port. No additional wiring is needed -- just connect the USB cable. When you have multiple Nucleos plugged in, each one gets its own COM port.

---

## CAN Bus Trunk and Stub Layout

CAN bus reliability depends on the physical wiring topology. At 1 Mbit/s, reflections from long stubs will corrupt data. Follow these rules.

### The Trunk (Mainline)

The trunk is the main cable that runs end-to-end. All nodes connect to it via short stubs.

```
 [120Ω]                                                              [120Ω]
   │                                                                    │
   ├──────────┬──────────┬──────────┬──────────┬──────────┬─────────────┤
   │          │          │          │          │          │             │
  Pi       STM32 #1   STM32 #2   STM32 #3   STM32 #4   (future)
 Node 0    Node 1     Node 2     Node 3     Node 4
           L. Hip     R. Hip     L. Knee    R. Knee

           ◄─── trunk spacing: 10-30 cm between taps ───►
```

**Tips for the trunk:**
- Use twisted pair wire (or at minimum, keep CANH and CANL wires close together and equal length)
- The trunk should be a single continuous run -- no star or tree topology
- Keep total trunk length under 1 meter for bench testing
- Both ends of the trunk get a 120-ohm termination resistor between CANH and CANL

### Stubs

A stub is the short wire from the trunk to a node's CAN transceiver.

**At 1 Mbit/s, keep stubs under 30 cm.** Shorter is better. Ideally under 10 cm.

```
    Trunk (CANH) ═══════╤═══════════
                        │ ← stub (< 30 cm)
                   ┌────┴────┐
                   │  CAN    │
                   │Transceivr│
                   └────┬────┘
                        │
                   ┌────┴────┐
                   │ Nucleo  │
                   └─────────┘
    Trunk (CANL) ═══════╧═══════════
```

**Tips for stubs:**
- Each stub should be two wires (CANH + CANL) of equal length
- Do **not** put a termination resistor on a stub -- only on the trunk ends
- If a transceiver module has an onboard termination jumper, **disable it** unless that module is at a physical end of the trunk

### Bench Setup Suggestion

For bench testing, a breadboard works well as the trunk:

1. Run two long strips on the breadboard as CANH and CANL (the trunk)
2. Place a 120-ohm resistor between CANH and CANL at each end of the strips
3. Tap off the strips at regular intervals for each node's transceiver
4. Keep transceiver-to-breadboard wires short (< 10 cm)

```
Breadboard layout:

    Row A (CANH): [120Ω]──●────●────●────●────●──[120Ω]
    Row B (CANL): [120Ω]──●────●────●────●────●──[120Ω]
                           │    │    │    │    │
                           Pi  STM1 STM2 STM3 STM4
                       (via transceiver modules)
```

> **Note:** The two 120-ohm resistors shown are the same two physical resistors -- one at the left end between rows A and B, one at the right end between rows A and B.

### Common Wiring Mistakes

| Mistake | Symptom | Fix |
|---|---|---|
| CANH and CANL swapped | No communication at all | Check transceiver datasheet for pin order |
| Missing termination | Intermittent errors, works close but fails at distance | Add 120Ω at both ends of trunk |
| Too many termination resistors | Bus voltage levels wrong, intermittent errors | Only 2 resistors total, at trunk ends only |
| Stub too long (> 30 cm) | Works at low speed, fails at 1 Mbit/s | Shorten stub or move node closer to trunk |
| Star topology (no trunk) | Random bit errors, especially with 3+ nodes | Rewire as linear trunk with short stubs |
| Missing ground between nodes | Random errors or no communication | Connect GND between all nodes |

---

## Computer Requirements

- **Python 3.6+** installed and accessible from the terminal
- **pyserial** Python package:
  ```
  pip install pyserial
  ```
- **python-can** Python package (for Pi-side scripts):
  ```
  pip install python-can
  ```
- **STM32CubeIDE** (for building and flashing the firmware)
- **can-utils** on the Raspberry Pi:
  ```bash
  sudo apt install can-utils
  ```
- A USB driver for the ST-Link debugger (usually installed automatically with CubeIDE)

---

## STM32CubeIDE Setup

### Opening the Project

1. Open STM32CubeIDE.
2. Go to **File > Open Projects from File System**.
3. Navigate to `testing/stm32/can-api` and select the folder.
4. Click **Finish** to import the project.

### Enabling Float Formatting (Required)

The test firmware uses `snprintf` with `%f` format specifiers for UART output. By default, the STM32 linker does not include float support. You **must** enable it:

1. Right-click the project in the **Project Explorer** > **Properties**.
2. Go to **C/C++ Build > Settings > MCU GCC Linker > Miscellaneous**.
3. In the **Other flags** field, add:
   ```
   -u _printf_float
   ```
4. Click **Apply and Close**.

> **If you skip this step**, all float values in UART output will show as `0.00`.

### Setting the Node ID Before Flashing

The test firmware uses a `#define` to set the node ID. Before flashing each board, change this value:

In `Core/Inc/can/test_config.h`:
```c
#define MY_NODE_ID  CAN_NODE_LEFT_HIP   // Change per board
```

| Board | Value |
|---|---|
| Left Hip | `CAN_NODE_LEFT_HIP` (1) |
| Right Hip | `CAN_NODE_RIGHT_HIP` (2) |
| Left Knee | `CAN_NODE_LEFT_KNEE` (3) |
| Right Knee | `CAN_NODE_RIGHT_KNEE` (4) |

### Building and Flashing

1. Click the **Build** button (hammer icon) or press `Ctrl+B`.
2. Connect the Nucleo board via USB.
3. Click the **Run** button (green play icon) or press `F11` to flash the firmware.
4. The board will reset and begin running.
5. Repeat for each Nucleo, changing `MY_NODE_ID` each time.

---

## Finding Your COM Ports

Each Nucleo board creates its own virtual COM port through its ST-Link debugger. When testing with multiple boards, you need to identify which COM port belongs to which board.

### Windows

1. Press `Win + X` and select **Device Manager**.
2. Expand the **Ports (COM & LPT)** section.
3. You will see multiple **STMicroelectronics STLink Virtual COM Port** entries.

> **Tip:** To identify which port is which board, unplug one Nucleo at a time and note which port disappears. Label your USB cables or Nucleos (e.g., tape marked "Node 1 = COM3").

### Linux

```bash
ls /dev/ttyACM*
```

With 4 boards connected, you'll see `/dev/ttyACM0` through `/dev/ttyACM3`. To map them:
```bash
# Shows which physical USB port maps to which ttyACM device
for dev in /dev/ttyACM*; do
    echo "$dev -> $(udevadm info -q path $dev | grep -o 'usb[0-9]/[0-9]-[0-9]')"
done
```

### macOS

```bash
ls /dev/cu.usbmodem*
```

### Multi-Board UART Monitoring

To monitor multiple boards at once, open a separate terminal window for each COM port. On Windows with PuTTY or the Python script:

```
Terminal 1: python uart_monitor.py COM3   # Node 1 - Left Hip
Terminal 2: python uart_monitor.py COM5   # Node 2 - Right Hip
Terminal 3: python uart_monitor.py COM7   # Node 3 - Left Knee
Terminal 4: python uart_monitor.py COM9   # Node 4 - Right Knee
```

> **Note:** Close STM32CubeIDE's built-in serial monitor before opening the port in another program. Only one program can hold a COM port at a time.

---

## UART Command Interface

The test firmware on each STM32 exposes a UART command interface at 115200 baud. Commands are sent as ASCII strings terminated by a newline.

### Available Commands

| Command | Response Format | Description |
|---|---|---|
| `PING` | `PONG` | Test UART connection |
| `STATUS` | `STATUS:OK node=<id>` | Reports node ID and init status |
| `SEND_IMU` | `TX_IMU:AX=<v> AY=<v> AZ=<v> GX=<v> GY=<v> GZ=<v>` | Send test IMU data over CAN |
| `SEND_ESTOP` | `TX_ESTOP:reason=<r>` | Send ESTOP broadcast |
| `SEND_MOTOR_STATUS` | `TX_MOTOR:POS=<v> SPD=<v> CUR=<v> TEMP=<v> ERR=<v>` | Send test motor status |
| `READ_MOTOR` | `MOTOR:POS=<deg> SPD=<rpm> CUR=<A> TEMP=<C> ERR=<code>` | Read live VESC motor feedback |
| `LOG_ON` | `LOG:ON` | Enable automatic logging of received CAN frames |
| `LOG_OFF` | `LOG:OFF` | Disable CAN receive logging |

### Automatic RX Logging Format

When `LOG_ON` is active, received CAN frames are printed as:

```
RX ESTOP: reason=0 (MANUAL) from=node0
RX TORQUE_CMD: torque=5.500 Nm from=node0
RX MOTOR_STATUS: pos=45.0 spd=1000.0 cur=2.50 temp=35 err=0 from=node1
RX IMU_ACCEL: ax=0.12 ay=-9.81 az=0.05 from=node1
RX IMU_GYRO: gx=1.50 gy=-0.30 gz=0.80 from=node1
RX EXT_FRAME: id=0x00000168 dlc=8 data=[...]  (VESC motor feedback)
```

---

## Raspberry Pi Setup

### Enable CAN Interface

Add to `/boot/config.txt`:
```
dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25
```

Reboot, then bring up the interface:
```bash
sudo ip link set can0 up type can bitrate 1000000
```

Verify:
```bash
ip -details link show can0
```

You should see `state UP` and `bitrate 1000000`.

---

## Phase 1: Pi + 1 STM32 (Basic Messaging)

**Goal:** Verify all message types can be sent and received between the Pi and a single STM32.

### Hardware for This Phase

- Raspberry Pi with CAN HAT + transceiver
- 1x Nucleo-F446RE with CAN transceiver
- 2x 120-ohm resistors (one at each end)
- No motor or IMU required

```
[120Ω]──── Pi ────────── STM32 #1 ────[120Ω]
           Node 0        Node 1
```

### Step 1: Wire the Bus

1. Connect Pi transceiver CANH to STM32 transceiver CANH.
2. Connect Pi transceiver CANL to STM32 transceiver CANL.
3. Place a 120-ohm resistor at each end (between CANH and CANL at the Pi side, and between CANH and CANL at the STM32 side).
4. Connect the Nucleo to your PC via USB.

### Step 2: Flash the STM32

1. Set `MY_NODE_ID` to `CAN_NODE_LEFT_HIP` (1).
2. Build and flash as described in [STM32CubeIDE Setup](#stm32cubeide-setup).

### Step 3: Verify UART

Open a terminal to the STM32's COM port:
```bash
python uart_monitor.py COM3
```

Type `PING`:
```
> PING
  <- PONG
```

Type `STATUS`:
```
> STATUS
  <- STATUS:OK node=1
```

### Step 4: Enable RX Logging on STM32

```
> LOG_ON
  <- LOG:ON
```

### Step 5: Test Pi → STM32 (Torque Command)

On the Pi, run:
```python
from can_api import can_common, can_motor

bus = can_common.create_bus("can0")
can_motor.send_torque_cmd(bus, can_common.NODE_LEFT_HIP, 5.5)
print("Sent 5.5 Nm to Node 1")
```

**Expected on STM32 UART:**
```
RX TORQUE_CMD: torque=5.500 Nm from=node0
```

### Step 6: Test Pi → STM32 (ESTOP)

On the Pi:
```python
from can_api import can_common, can_system

bus = can_common.create_bus("can0")
can_system.send_estop(bus, can_common.NODE_PI, can_system.ESTOP_MANUAL)
```

**Expected on STM32 UART:**
```
RX ESTOP: reason=0 (MANUAL) from=node0
```

### Step 7: Test STM32 → Pi (IMU Data)

On the STM32 UART, type:
```
> SEND_IMU
  <- TX_IMU:AX=1.00 AY=-9.81 AZ=0.50 GX=0.10 GY=-0.20 GZ=0.30
```

On the Pi, run:
```python
from can_api import can_common, can_imu

bus = can_common.create_bus("can0")
while True:
    msg = can_common.recv(bus, timeout=1.0)
    if msg is None:
        print("No message received")
        break
    msg_type, src, dest = can_common.parse_can_id(msg.arbitration_id)
    if msg_type == can_common.MSG_IMU_ACCEL:
        ax, ay, az = can_imu.parse_imu_accel(msg)
        print(f"IMU Accel from node {src}: ({ax:.2f}, {ay:.2f}, {az:.2f})")
    elif msg_type == can_common.MSG_IMU_GYRO:
        gx, gy, gz = can_imu.parse_imu_gyro(msg)
        print(f"IMU Gyro from node {src}: ({gx:.2f}, {gy:.2f}, {gz:.2f})")
```

**Expected on Pi:**
```
IMU Accel from node 1: (1.00, -9.81, 0.50)
IMU Gyro from node 1: (0.10, -0.20, 0.30)
```

### Step 8: Test STM32 → Pi (Motor Status)

On the STM32 UART:
```
> SEND_MOTOR_STATUS
  <- TX_MOTOR:POS=45.0 SPD=1000.0 CUR=2.50 TEMP=35 ERR=0
```

On the Pi:
```python
from can_api import can_common, can_motor

bus = can_common.create_bus("can0")
msg = can_common.recv(bus, timeout=1.0)
if msg:
    msg_type, src, dest = can_common.parse_can_id(msg.arbitration_id)
    if msg_type == can_common.MSG_MOTOR_STATUS:
        pos, spd, cur, temp, err = can_motor.parse_motor_status(msg)
        print(f"Motor status from node {src}: pos={pos:.1f} spd={spd:.0f} cur={cur:.2f} temp={temp} err={err}")
```

**Expected on Pi:**
```
Motor status from node 1: pos=45.0 spd=1000.0 cur=2.50 temp=35 err=0
```

### Phase 1 Pass Criteria

- [ ] `PING` / `PONG` works over UART
- [ ] Pi sends TORQUE_CMD, STM32 logs it with correct value (5.500 Nm)
- [ ] Pi sends ESTOP, STM32 logs it with correct reason
- [ ] STM32 sends IMU accel + gyro, Pi parses correct values
- [ ] STM32 sends motor status, Pi parses all 5 fields correctly

---

## Phase 2: Pi + 1 STM32 + 1 Motor (Full Chain)

**Goal:** Verify the complete data path: Pi sends torque → STM32 drives motor → motor feedback → STM32 relays status to Pi.

### Additional Hardware

- 1x AK70-9 motor with power supply
- Motor connects to the same CAN bus (CANH/CANL)

```
[120Ω]──── Pi ────────── STM32 #1 ────────── AK70-9 ────[120Ω]
           Node 0        Node 1               Motor ID 104
```

> **Note:** The motor's CAN transceiver is built into the motor. Connect the motor's CANH/CANL directly to the bus trunk. Move the termination resistor from the STM32 end to the motor end (the motor is now the physical endpoint).

### Step 1: Power Up

1. Connect the motor to the bus.
2. Power the motor with its supply (**keep hands clear of the rotor**).
3. Flash the STM32 with `MY_NODE_ID = CAN_NODE_LEFT_HIP`.

### Step 2: Verify Motor Feedback

On the STM32 UART:
```
> LOG_ON
  <- LOG:ON
```

You should see periodic motor feedback frames:
```
RX EXT_FRAME: id=0x00000168 dlc=8 data=[00 00 00 00 00 00 19 00]
```

Read parsed motor data:
```
> READ_MOTOR
  <- MOTOR:POS=0.00 SPD=0.0 CUR=0.00 TEMP=25 ERR=0
```

### Step 3: Pi Sends Torque, Motor Responds

On the Pi:
```python
from can_api import can_common, can_motor
import time

bus = can_common.create_bus("can0")

# Send a small torque command (be careful with motors!)
can_motor.send_torque_cmd(bus, can_common.NODE_LEFT_HIP, 0.5)  # 0.5 Nm

# Read back motor status
for _ in range(20):
    msg = can_common.recv(bus, timeout=0.1)
    if msg is None:
        continue
    msg_type, src, dest = can_common.parse_can_id(msg.arbitration_id)
    if msg_type == can_common.MSG_MOTOR_STATUS:
        pos, spd, cur, temp, err = can_motor.parse_motor_status(msg)
        print(f"pos={pos:.1f}deg spd={spd:.0f}ERPM cur={cur:.2f}A temp={temp}C err={err}")
```

**Expected:** The motor should move slightly, and the Pi should receive status with non-zero speed and current values.

> **Warning:** Start with small torque values (< 1 Nm). Secure the motor before testing. The AK70-9 can produce up to 24.7 Nm peak torque.

### Step 4: Verify ESTOP Stops Motor

While the motor is running with a small torque:

1. Send ESTOP from Pi:
   ```python
   can_system.send_estop(bus, can_common.NODE_PI, can_system.ESTOP_MANUAL)
   ```
2. The STM32 should log `RX ESTOP`, set torque to 0, and stop the motor.

**Expected on STM32 UART:**
```
RX ESTOP: reason=0 (MANUAL) from=node0
*** ESTOP ACTIVATED — motor disabled ***
```

### Phase 2 Pass Criteria

- [ ] Motor feedback (extended frames) parsed correctly by STM32
- [ ] Pi sends torque, motor physically responds
- [ ] Motor status relayed from STM32 to Pi with correct values
- [ ] ESTOP from Pi stops the motor

---

## Phase 3: Pi + 2 STM32s (Filtering, No Motors)

**Goal:** Verify that hardware filters correctly route torque commands -- each STM32 only receives commands addressed to it.

### Hardware for This Phase

- Raspberry Pi with CAN HAT
- 2x Nucleo-F446RE with CAN transceivers
- No motors needed

```
[120Ω]──── Pi ──────── STM32 #1 ──────── STM32 #2 ────[120Ω]
           Node 0      Node 1             Node 2
                       L. Hip             R. Hip
```

### Step 1: Flash Both STM32s

- Board 1: `MY_NODE_ID = CAN_NODE_LEFT_HIP` (1)
- Board 2: `MY_NODE_ID = CAN_NODE_RIGHT_HIP` (2)

### Step 2: Open Two UART Terminals

```
Terminal 1: python uart_monitor.py COM3   # Node 1
Terminal 2: python uart_monitor.py COM5   # Node 2
```

Enable logging on both:
```
> LOG_ON
```

### Step 3: Send Targeted Torque Commands

On the Pi:
```python
from can_api import can_common, can_motor

bus = can_common.create_bus("can0")

# Send to Node 1 only
can_motor.send_torque_cmd(bus, can_common.NODE_LEFT_HIP, 3.0)

# Send to Node 2 only
can_motor.send_torque_cmd(bus, can_common.NODE_RIGHT_HIP, 7.0)
```

**Expected on Terminal 1 (Node 1):**
```
RX TORQUE_CMD: torque=3.000 Nm from=node0
```
Node 1 should **not** see the 7.0 Nm command.

**Expected on Terminal 2 (Node 2):**
```
RX TORQUE_CMD: torque=7.000 Nm from=node0
```
Node 2 should **not** see the 3.0 Nm command.

### Step 4: Verify ESTOP Reaches Both

```python
can_system.send_estop(bus, can_common.NODE_PI, can_system.ESTOP_MANUAL)
```

**Expected on both terminals:**
```
RX ESTOP: reason=0 (MANUAL) from=node0
```

ESTOP is broadcast -- every node must receive it.

### Step 5: Verify Both STM32s Can Send to Pi

On STM32 #1 UART:
```
> SEND_IMU
```

On STM32 #2 UART:
```
> SEND_IMU
```

On the Pi, run a receiver that prints the source node for each message. You should see data from both node 1 and node 2.

### Phase 3 Pass Criteria

- [ ] Node 1 receives only its torque command (3.0 Nm)
- [ ] Node 2 receives only its torque command (7.0 Nm)
- [ ] Neither node sees the other's torque command
- [ ] Both nodes receive ESTOP broadcast
- [ ] Pi receives IMU data from both nodes with correct source IDs

---

## Phase 4: Pi + 2 STM32s + 2 Motors (Targeted Motor Control)

**Goal:** Verify that targeted torque commands drive the correct motor. This is the key proof that the addressing scheme works end-to-end with real actuators.

### Hardware for This Phase

- Everything from Phase 3
- 2x AK70-9 motors with power supplies

```
[120Ω]──── Pi ──────── STM32 #1 ──────── STM32 #2 ──────── Motor 2 ────[120Ω]
           Node 0      Node 1             Node 2
                       L. Hip             R. Hip
                         │                  │
                       Motor 1            Motor 2
```

> **Note:** Both motors share the same CAN bus. The STM32 firmware routes only its own received torque command to its motor via the VESC extended-frame protocol.

### Step 1: Power Up Both Motors

1. Connect both motors to the trunk.
2. Power both motors (**keep clear of rotors**).
3. Flash STM32 #1 as Node 1 and STM32 #2 as Node 2.

### Step 2: Verify Both Motors Report Feedback

On each UART terminal:
```
> READ_MOTOR
  <- MOTOR:POS=0.00 SPD=0.0 CUR=0.00 TEMP=25 ERR=0
```

### Step 3: Send Different Torques to Each Motor

On the Pi:
```python
from can_api import can_common, can_motor
import time

bus = can_common.create_bus("can0")

# Send 0.3 Nm to Left Hip (Node 1)
can_motor.send_torque_cmd(bus, can_common.NODE_LEFT_HIP, 0.3)

time.sleep(0.5)

# Send 0.8 Nm to Right Hip (Node 2)
can_motor.send_torque_cmd(bus, can_common.NODE_RIGHT_HIP, 0.8)
```

**Expected:**
- Motor 1 (Left Hip) spins slowly with low torque.
- Motor 2 (Right Hip) spins faster/stronger with higher torque.
- **The motors should behave noticeably differently** because they received different commands.

**Verify on UART terminals:**

Terminal 1 (Node 1):
```
RX TORQUE_CMD: torque=0.300 Nm from=node0
```

Terminal 2 (Node 2):
```
RX TORQUE_CMD: torque=0.800 Nm from=node0
```

### Step 4: Verify Motor Status from Both Nodes

On the Pi:
```python
from can_api import can_common, can_motor

bus = can_common.create_bus("can0")

for _ in range(50):
    msg = can_common.recv(bus, timeout=0.1)
    if msg is None:
        continue
    msg_type, src, dest = can_common.parse_can_id(msg.arbitration_id)
    if msg_type == can_common.MSG_MOTOR_STATUS:
        pos, spd, cur, temp, err = can_motor.parse_motor_status(msg)
        node_name = {1: "L.Hip", 2: "R.Hip"}.get(src, f"node{src}")
        print(f"[{node_name}] pos={pos:.1f} spd={spd:.0f} cur={cur:.2f} temp={temp} err={err}")
```

**Expected:** You should see interleaved status from both motors with different speed/current values reflecting their different torque commands.

### Step 5: ESTOP Stops Both Motors

```python
can_system.send_estop(bus, can_common.NODE_PI, can_system.ESTOP_MANUAL)
```

**Both motors should stop.**

### Phase 4 Pass Criteria

- [ ] Pi sends different torques to Node 1 and Node 2
- [ ] Motor 1 and Motor 2 respond with visibly different behavior
- [ ] Each STM32 only drives its own motor (no cross-talk)
- [ ] Pi receives motor status from both nodes with correct source IDs
- [ ] ESTOP stops both motors simultaneously

---

## Phase 5: Full Network (Pi + 4 STM32s + 4 Motors)

**Goal:** Verify the complete 5-node network at target data rates.

### Hardware for This Phase

- Everything from previous phases, scaled to 4 nodes
- 4x Nucleo-F446RE with CAN transceivers
- 4x AK70-9 motors with power supplies
- 4x LSM6DS3TR-C IMU breakout boards
- USB hub for 4 Nucleos

```
[120Ω]── Pi ──── STM32#1 ──── STM32#2 ──── STM32#3 ──── STM32#4 ──[120Ω]
         N0       N1/L.Hip     N2/R.Hip     N3/L.Knee    N4/R.Knee
                    │            │             │             │
                 Motor 1      Motor 2       Motor 3       Motor 4
                 + IMU         + IMU         + IMU         + IMU
```

### Step 1: Flash All 4 STM32s

- Board 1: `CAN_NODE_LEFT_HIP` (1)
- Board 2: `CAN_NODE_RIGHT_HIP` (2)
- Board 3: `CAN_NODE_LEFT_KNEE` (3)
- Board 4: `CAN_NODE_RIGHT_KNEE` (4)

### Step 2: Open 4 UART Terminals

```
Terminal 1: python uart_monitor.py COM3   # Node 1 - L. Hip
Terminal 2: python uart_monitor.py COM5   # Node 2 - R. Hip
Terminal 3: python uart_monitor.py COM7   # Node 3 - L. Knee
Terminal 4: python uart_monitor.py COM9   # Node 4 - R. Knee
```

Enable logging on all:
```
> LOG_ON
```

### Step 3: Verify All Nodes Respond

On each terminal:
```
> PING
  <- PONG

> STATUS
  <- STATUS:OK node=<expected_id>
```

### Step 4: Targeted Torque to All 4 Motors

On the Pi:
```python
from can_api import can_common, can_motor

bus = can_common.create_bus("can0")

torques = {
    can_common.NODE_LEFT_HIP:   0.3,
    can_common.NODE_RIGHT_HIP:  0.5,
    can_common.NODE_LEFT_KNEE:  0.4,
    can_common.NODE_RIGHT_KNEE: 0.6,
}

for node, torque in torques.items():
    can_motor.send_torque_cmd(bus, node, torque)
```

**Expected:** Each terminal shows only its own torque value. Each motor responds proportionally.

### Step 5: Full Data Rate Test

Run the complete control loop at target rate (200 Hz torque commands, 500 Hz IMU):

On the Pi:
```python
from can_api import can_common, can_motor, can_imu
import time

bus = can_common.create_bus("can0")
nodes = [1, 2, 3, 4]

start = time.time()
tx_count = 0
rx_count = 0

# Run for 5 seconds
while time.time() - start < 5.0:
    # Send torque to all 4 nodes (200 Hz = every 5 ms)
    for node in nodes:
        can_motor.send_torque_cmd(bus, node, 0.1)
        tx_count += 1

    # Drain receive buffer
    while True:
        msg = can_common.recv(bus, timeout=0.001)
        if msg is None:
            break
        rx_count += 1

    time.sleep(0.005)

elapsed = time.time() - start
print(f"TX: {tx_count} frames in {elapsed:.1f}s ({tx_count/elapsed:.0f} frames/s)")
print(f"RX: {rx_count} frames in {elapsed:.1f}s ({rx_count/elapsed:.0f} frames/s)")
```

**Expected output (approximate):**
```
TX: 4000 frames in 5.0s (800 frames/s)
RX: ~20000 frames in 5.0s (~4000 frames/s)
```

The RX count includes 4 nodes x (IMU accel + IMU gyro + motor status) at their respective rates.

### Step 6: ESTOP Stops All Motors

```python
can_system.send_estop(bus, can_common.NODE_PI, can_system.ESTOP_MANUAL)
```

**All four motors should stop immediately.**

### Phase 5 Pass Criteria

- [ ] All 4 nodes respond to PING/STATUS with correct IDs
- [ ] Each node receives only its own torque command
- [ ] All 4 motors respond to their individual torque values
- [ ] Pi receives IMU data from all 4 nodes
- [ ] Pi receives motor status from all 4 nodes
- [ ] Data rate test runs for 5 seconds without errors or dropped frames
- [ ] ESTOP broadcast stops all 4 motors

---

## Common Errors

### No CAN frames received by any node

**Possible causes:**
- CAN interface not up on the Pi (`ip link show can0` should show `state UP`).
- Bitrate mismatch -- all nodes must use 1 Mbit/s. STM32 CAN config: prescaler 6, BS1 11TQ, BS2 2TQ.
- Missing or incorrect termination resistors.
- CANH/CANL swapped on one node.

**Fix:** Verify the bus is up, check bitrate settings, and double-check all wiring against the [wiring tables](#wiring).

### STM32 receives ESTOP but not TORQUE_CMD

**Cause:** The node ID passed to `can_common_init()` does not match the destination in the torque command.

**Fix:** Verify `MY_NODE_ID` matches the node you are sending to. Check with `STATUS` command.

### One STM32 receives another STM32's torque command

**Cause:** Both boards were flashed with the same `MY_NODE_ID`.

**Fix:** Re-flash one board with the correct node ID. Verify with `STATUS`.

### Motor feedback not appearing (READ_MOTOR shows all zeros)

**Possible causes:**
- Motor not powered.
- Motor CAN ID mismatch (firmware expects ID 104).
- Motor's CANH/CANL not connected to the bus trunk.

**Fix:** Verify motor power, check CAN wiring, and confirm the motor's VESC CAN ID is 104.

### Intermittent errors or garbage data with 3+ nodes

**Possible causes:**
- Star topology instead of linear trunk.
- Stubs too long (> 30 cm at 1 Mbit/s).
- Missing ground connection between nodes.
- Too many termination resistors (only 2 allowed).

**Fix:** Rewire as a linear trunk with short stubs. See [CAN Bus Trunk and Stub Layout](#can-bus-trunk-and-stub-layout).

### "TX mailbox full" at high data rates

**Cause:** Sending frames faster than the bus can transmit them.

**Fix:** Add a small delay between sends (1 ms minimum). At 1 Mbit/s, a full 8-byte frame takes ~130 µs including overhead.

### All float values are 0.00

**Cause:** The `-u _printf_float` linker flag is missing.

**Fix:** Follow the [Enabling Float Formatting](#enabling-float-formatting-required) section.

### "No serial ports found" when running the Python script

**Possible causes:**
- The Nucleo board is not plugged in.
- The ST-Link USB driver is not installed.
- Another program has the port open.

**Fix:** Close any other programs using the COM port, then retry.
