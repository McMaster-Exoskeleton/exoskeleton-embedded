# exo-can SETUP

This document explains how to build, flash, and validate the STM32 CAN bring up plus the current C++ CAN module integration on two NUCLEO F446RE boards using an SN65HVD230 transceiver.

## Scope

* Target board: NUCLEO F446RE
* Peripheral: CAN1
* Pins: PA11 (CAN1_RX), PA12 (CAN1_TX)
* Transceiver: SN65HVD230DR (3.3 V)
* Current test behavior
  * Each board periodically transmits a command frame to a peer node ID
  * Each board toggles LD2 when it receives an allowed CAN frame
* Filtering
  * Hardware filters are configured using an allowlist of standard IDs (node specific), not accept all

## Prerequisites

### Software

* STM32CubeIDE installed (1.19.x or similar)
* Git installed (Git Bash on Windows is fine)

### Hardware

* 2x NUCLEO F446RE boards
* 2x SN65HVD230 transceiver modules (or equivalent)
* Twisted pair wiring for CANH and CANL
* 2x 120 ohm termination resistors (one at each physical end of the bus)
* 2x USB cables for flashing both boards

## Repo setup

1. Clone the repository and checkout the branch

   ```bash
   git clone https://github.com/McMaster-Exoskeleton/exoskeleton-embedded.git
   cd exoskeleton-embedded
   git checkout canbus-implementation
   ```

2. Pull latest changes

   ```bash
   git pull
   ```

## Open the STM32 project in STM32CubeIDE

Project path in the repo:

* `stm32/exo-can/`

In CubeIDE:

1. File -> Open Projects from File System… (or Import -> Existing Projects into Workspace)
2. Select directory: `exoskeleton-embedded/stm32/exo-can`
3. Import it into your workspace

Notes:

* Prefer building the project that lives inside the repo folder, so changes are version controlled.
* Avoid editing a detached copy in a separate CubeIDE workspace directory unless you intentionally mirror changes back into the repo.

## Build

In CubeIDE:

1. Select the project
2. Project -> Build Project

Expected output:

* Build produces an `.elf` and usually a `.bin` or `.hex` under `Debug/` (or `Release/`).

## Flash to NUCLEO

In CubeIDE:

1. Run -> Debug (or Run)
2. CubeIDE programs the board through ST LINK

Repeat for the second board.

## Hardware wiring

### STM32 to transceiver (SN65HVD230)

* NUCLEO PA12 (CAN1_TX) -> SN65HVD230 TXD
* NUCLEO PA11 (CAN1_RX) -> SN65HVD230 RXD
* NUCLEO 3.3 V -> SN65HVD230 VCC (do not use 5 V unless your module supports it)
* NUCLEO GND -> SN65HVD230 GND

### CAN bus between transceivers

* CANH on transceiver A <-> CANH on transceiver B
* CANL on transceiver A <-> CANL on transceiver B

### Termination

* Place a 120 ohm resistor across CANH and CANL at each physical end of the bus.
* With two nodes, that usually means one 120 ohm at each end.

Common mistakes:

* No shared ground between nodes
* Missing termination, or termination placed incorrectly
* CANH and CANL swapped on one end

## CAN bitrate and timing

Bitrate is configured via CubeMX (the `.ioc` settings). Current target is 500 kbps using:

* CAN clock: 45 MHz
* Prescaler: 6
* SyncSeg: 1
* TimeSeg1: 12
* TimeSeg2: 2

Formula:

* Bitrate = CAN_clock / (Prescaler * (SyncSeg + TimeSeg1 + TimeSeg2))
* 500000 = 45000000 / (6 * (1 + 12 + 2)) = 45000000 / 90

If you change clock tree, APB settings, prescaler, or time segments, recompute bitrate.

## Node configuration for 2 board test

The application uses per board constants for:

* ThisNode: which IDs the board accepts (filters)
* PeerNode: which node the board sends commands to

Find these in:

* `Core/Src/can/can_app.cpp`

Example intent for two boards:

* Board A: ThisNode = 1, PeerNode = 2
* Board B: ThisNode = 2, PeerNode = 1

After changing node IDs, rebuild and flash.

## CAN IDs and why some are accepted

IDs come from:

* `Core/Inc/can/can_protocol.hpp`

High level scheme:

* Command to a node: CmdId(node) = CmdBase + node
* State from a node:  StateId(node) = StateBase + node
* Heartbeat:          HbId(node) = HbBase + node

In the current bring up test, a joint style node typically accepts:

* CmdId(ThisNode)
* HbId(ThisNode) (if included)

It does not accept StateId(ThisNode) because state frames are intended to flow joint to master, not back into the joint.

## Filtering behavior (bxCAN allowlist)

Filtering is configured in:

* `Core/Src/can/can_bus_stm32.cpp`

The driver init takes an allowlist of standard IDs and configures bxCAN filters using:

* IDLIST mode
* 32 bit scale
* FIFO0 assignment

Key behavior:

* One filter bank can match two exact standard IDs in 32 bit list mode.
* If more than two IDs are provided, additional banks are configured in a loop.

Result:

* MCU receives only traffic intended for that node, reducing CPU load and preventing RX queue flooding later.

## Runtime behavior and validation on hardware

Expected behavior:

* Each board transmits a command frame every 100 ms to the peer command ID.
* When a board receives a CAN frame that passes its filter, it toggles LD2.

Basic test checklist:

1. Flash Board A with ThisNode = 1, PeerNode = 2
2. Flash Board B with ThisNode = 2, PeerNode = 1
3. Connect CAN wiring and termination
4. Observe LEDs toggling on both boards when traffic is received

Filter validation test:

* Change the transmit ID on one board to an ID not accepted by the other board allowlist.
* Reflash, the other board should stop toggling, confirming filtering is active.

## Project structure (key files)

Protocol and data types:

* `Core/Inc/can/can_protocol.hpp`  
  Defines CAN ID helpers and 8 byte payload structs (command, state, heartbeat).

* `Core/Inc/can/can_frame.hpp`  
  Lightweight frame representation used internally, independent of HAL structs.

RX buffering:

* `Core/Inc/can/ring_buffer.hpp`  
  Fixed size RX ring buffer used to move RX work out of interrupt context.

STM32 CAN wrapper:

* `Core/Inc/can/can_bus_stm32.hpp`
* `Core/Src/can/can_bus_stm32.cpp`  
  Configures filters, starts CAN, enables notifications, provides send and RX queueing.

C and C++ bridge and app entrypoints:

* `Core/Inc/can/can_app.h`
* `Core/Src/can/can_app.cpp`  
  CanApp_Init and CanApp_Tick entrypoints plus HAL RX callback forwarding into the C++ driver.

Generated code:

* CubeMX generates `main.c`, interrupts, MSP init, and the `.ioc`.
* Keep custom logic inside the `can/` module and only minimal glue in generated sections.

## Common pitfalls

### Duplicate HAL callbacks

Only define each HAL callback once. For example:

* `HAL_CAN_RxFifo0MsgPendingCallback` must exist in exactly one file.
* Defining it in both `main.c` and `can_app.cpp` causes multiple definition linker errors.

### C and C++ linkage

Functions called from C (`main.c`) must have C linkage:

* `CanApp_Init`
* `CanApp_Tick`

These are declared in `can_app.h` with extern C guards.

### Filters not taking effect

* Confirm filters are configured before `HAL_CAN_Start`
* Confirm CAN1 and FIFO0 are used consistently
* Confirm ID packing matches standard 11 bit IDs

## Next steps after bring up

* Replace LED toggling with real message dispatch and payload decoding.
* Implement command sequence echo and timeout safety behavior (loss of command leads to safe state).
* Add counters for dropped frames and RX queue overflow.
* Add Pi SocketCAN integration once MCU side is stable.
