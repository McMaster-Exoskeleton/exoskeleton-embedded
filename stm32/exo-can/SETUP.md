# STM32 CAN Bus Communication Test

## Project Overview

This CubeIDE project implements CAN bus communication testing between two STM32 Nucleo-F446RE boards. Each board has a CAN transceiver connected via CAN TX/RX pins, with the transceivers' HIGH and LOW lines connected together and terminated with 120Ω resistors.

## Hardware Configuration

### Per Board Setup:
- **Microcontroller**: Nucleo-F446RE
- **CAN Transceiver**: Connected to CAN1 TX/RX pins
- **Termination**: 120Ω resistors on each transceiver's HIGH and LOW ports
- **UART**: USART2 on PA2(TX) and PA3(RX) at 115200 baud
- **LED**: LD2 on PA5 (onboard LED)

### CAN Bus Configuration:
- **CAN1**: 500 kbps (Prescaler=6, TimeSeg1=11TQ, TimeSeg2=2TQ)
- **CAN2**: Initialized but not used in current code
- **Filter**: Accept all CAN messages (mask 0x0000)

## Current Implementation

### What the Code Does:

**File Structure:**
- `Core/Src/main.c` - Main program and peripheral initialization
- `Core/Src/can/can_app.cpp` - Application logic (lines 11-38)
- `Core/Src/can/can_bus_stm32.cpp` - CAN driver wrapper
- `Core/Inc/can/can_frame.hpp` - CAN frame data structure
- `Core/Inc/can/ring_buffer.hpp` - RX buffer (32 frame capacity)

**Functionality:**

1. **Automatic CAN Transmission** (can_app.cpp:17-26)
   - Sends CAN message every 100ms
   - CAN ID: `0x123` (standard ID)
   - Data: Single byte counter (0, 1, 2, 3...)
   - Increments automatically

2. **CAN Reception** (can_app.cpp:28-32)
   - Receives CAN messages via interrupt callback
   - Stores frames in ring buffer
   - Toggles onboard LED (LD2) for each received message

### Current Behavior with Two Boards:

| Board 1 | CAN Bus | Board 2 |
|---------|---------|---------|
| Sends 0x123 counter every 100ms → | → | Receives & blinks LED |
| Receives & blinks LED | ← | ← Sends 0x123 counter every 100ms |

**Result:** Both boards continuously transmit and both LEDs blink when receiving from each other.

## What's Missing for Full UART-CAN Bridge

### Intended Goal:
```
Computer → UART → Board 1 → CAN → Board 2 → UART → Computer
```

### Missing Components:

| Feature | Status | Notes |
|---------|--------|-------|
| UART RX Handler | ❌ Not implemented | Need to receive data from computer |
| UART → CAN Bridge | ❌ Not implemented | Forward UART data as CAN messages |
| CAN → UART Bridge | ❌ Not implemented | Forward CAN messages to UART |
| Message Protocol | ❌ Not implemented | Define UART/CAN data packaging |

**Currently:** Boards communicate via CAN but do NOT interact with computer via UART.

## Testing the Current Setup

### Prerequisites:
- Two Nucleo-F446RE boards programmed with this code
- CAN transceivers connected to each board
- HIGH and LOW lines connected between transceivers
- 120Ω termination resistors on both ends
- USB cables to power both boards

### Test Procedure:

#### Step 1: Hardware Connection
1. Connect Board 1's CAN transceiver HIGH to Board 2's transceiver HIGH
2. Connect Board 1's CAN transceiver LOW to Board 2's transceiver LOW
3. Verify 120Ω resistors are between HIGH and LOW on both transceivers
4. Connect USB cables to both boards for power

#### Step 2: Program Both Boards
1. Open this project in STM32CubeIDE
2. Build the project (Project → Build All)
3. Connect Board 1 via USB
4. Flash the program (Run → Debug or Run)
5. Disconnect Board 1
6. Connect Board 2 via USB
7. Flash the same program
8. Keep both boards powered

#### Step 3: Observe Operation
**Expected Behavior:**
- Both board LEDs (LD2) should blink rapidly
- Each board sends a message every 100ms
- Each board toggles LED when receiving the other's message
- Result: ~10 blinks per second on each board

**Troubleshooting:**

| Symptom | Possible Cause | Solution |
|---------|---------------|----------|
| No LED blinking on either board | CAN bus not connected | Check HIGH/LOW wiring |
| | Missing termination resistors | Verify 120Ω resistors present |
| | Both boards not powered | Check USB connections |
| Only one LED blinking | One board not transmitting | Re-flash board, check power |
| | CAN transceiver issue | Check transceiver connections |
| Intermittent blinking | Loose connections | Secure all wiring |
| | Incorrect termination | Verify 120Ω value and placement |

#### Step 4: Debug Monitoring (Optional)
1. Keep Board 1 connected via ST-Link debugger
2. Set breakpoint at `can_app.cpp:30` (CAN receive handler)
3. Run in debug mode
4. Breakpoint should trigger every 100ms when receiving from Board 2
5. Inspect `f.id` (should be `0x123`) and `f.data[0]` (incrementing counter)

### Success Criteria:
✅ Both LEDs blink continuously at ~10 Hz
✅ No error handler triggered (boards don't freeze)
✅ Counter increments in debugger

## Pin Connections Summary

### Board Connections:
```
Board 1                          Board 2
-------                          -------
CAN TX → Transceiver             CAN TX → Transceiver
CAN RX ← Transceiver             CAN RX ← Transceiver
         |                                |
         HIGH ←------[CAN Bus]-------→ HIGH
         LOW  ←------[CAN Bus]--------→ LOW
         |                                |
      [120Ω]                          [120Ω]
```

### Nucleo-F446RE Default CAN Pins:
- **CAN1_TX**: PA12
- **CAN1_RX**: PA11

## Code Key Points

### Initialization Flow (main.c):
```c
main() → MX_CAN1_Init() → MX_CAN2_Init() → CanApp_Init() → main loop
```

### Main Loop (main.c:106-108):
```c
while (1) {
    CanApp_Tick();  // Sends counter every 100ms, processes RX queue
}
```

### CAN Message Format:
- **ID**: 0x123 (11-bit standard ID)
- **DLC**: 1 byte
- **Data**: 8-bit counter value

## Next Steps

To implement the full UART-CAN bridge functionality:
1. Add UART RX interrupt handler
2. Create UART receive buffer
3. Implement UART → CAN forwarding logic
4. Implement CAN → UART forwarding logic
5. Define message protocol/framing
6. Test bidirectional communication

## Hardware Requirements

- 2x STM32 Nucleo-F446RE boards
- 2x CAN transceivers (e.g., TJA1050, MCP2551)
- 2x 120Ω resistors (1/4W or higher)
- Jumper wires for connections
- 2x USB cables
