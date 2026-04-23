# Old vs New CAN Implementation — Comparison

This compares the CAN usage in the **production** folder (`src/`) with the
**torque-controller test project** (`testing/stm32/torque-controller/`).

## TL;DR

| Aspect                     | Production (`src/`)              | Torque-controller (`testing/`)      |
|----------------------------|----------------------------------|-------------------------------------|
| CAN ID scheme              | Hardcoded (0x123–0x130)          | Structured: `type/src/dest`         |
| STM32 filter setup         | **None** (no filters configured) | 3 filter banks, 2 FIFOs             |
| STM32 RX                   | **Not implemented** (TX only)    | Ring buffer + ISR + both FIFOs      |
| Error recovery             | `AutoBusOff=DISABLE`             | `AutoBusOff=ENABLE` + SCE handler   |
| Auto-retransmit            | `AutoRetransmission=DISABLE`     | `AutoRetransmission=ENABLE`         |
| Pi filter                  | **None** (accepts all frames)    | Uses `apis/can/python` helpers      |
| Pi CAN ID handling         | Raw integer lookup in a dict     | `build_can_id / parse_can_id`       |
| Shared code                | None (STM32 + Pi duplicate IDs)  | Same constants on both sides        |

---

## 1. CAN ID Scheme

### Production (`src/stm32/joint-controller/main.c` + `src/pi/pi_can_buffer.py`)

Hardcoded numeric IDs, one per sensor channel, defined separately on each side:

```c
// STM32: (defined in lsm6ds3tr.h / main.h, referenced in CAN_Send_IMU_Data)
TxHeader.StdId = ACCEL_HIP_R;  // 0x125
TxHeader.StdId = GYRO_HIP_R;   // 0x126
```

```python
# Pi: (pi_can_buffer.py line 12-26)
imu1_a_id = 0x123
imu1_g_id = 0x124
imu2_a_id = 0x125
imu2_g_id = 0x126
# ... one constant per message
```

**Problems:**
- If you change an ID on one side, you must remember to change the other.
- No structure — 0x125 doesn't tell you "right hip accel" unless you look it up.
- No room for new message types without manually picking a free ID range.

### Torque-controller (new API)

IDs are *computed* from a message type + source + destination:

```c
// 11-bit ID = (type << 7) | (src << 4) | dest
uint16_t id = CAN_BUILD_ID(CAN_MSG_TORQUE_CMD, CAN_NODE_PI, CAN_NODE_LEFT_HIP);
// → 0x081
```

```python
# Pi: same construction, identical constants
can_id = build_can_id(MSG_TORQUE_CMD, NODE_PI, NODE_LEFT_HIP)
# → 0x081
```

**Benefits:**
- One source of truth (the constants in `can_common.h` / `can_common.py`).
- Given any ID, you can extract `type`, `src`, `dest` in one line.
- Adding a new message type = one constant, no ID-range management.

---

## 2. STM32 Filter Configuration

### Production

```c
// src/stm32/joint-controller/main.c line 118
HAL_CAN_Start(&hcan1);
```

That's it. **No `HAL_CAN_ConfigFilter()` call anywhere.** On STM32F4, if no
filter is configured, the peripheral rejects all incoming frames — which is
fine *for this file* because it only transmits IMU data. But it means the
joint controller physically cannot receive a torque command.

### Torque-controller

```c
// Filter 0 (FIFO0): ESTOP from any source
// Filter 1 (FIFO0): TORQUE_CMD addressed to MY_NODE_ID
// Filter 2 (FIFO1): all extended frames (VESC feedback)
can_common_init(&hcan1, MY_NODE_ID);
```

Three filter banks, split across two FIFOs. The important detail:

- **FIFO0** receives Pi commands (ESTOP + TORQUE_CMD)
- **FIFO1** receives motor feedback (extended frames)

This separation is what **solves the bug** you saw originally ("after one
motor command all other commands wouldn't go through"). With a single
accept-all filter + single FIFO, motor feedback at ~500 Hz flooded the FIFO
and blocked Pi commands. Splitting them means the two streams can't starve
each other.

---

## 3. RX Path on the STM32

### Production

There is **no RX path** in `src/stm32/joint-controller/main.c`. The file:
- Enables CAN1 and starts the peripheral
- Transmits accel/gyro every 2 ms
- Has no `HAL_CAN_RxFifo0MsgPendingCallback` or similar

If you add a torque receive path here, you'd be adding it from scratch.

### Torque-controller

```c
// can_common.c — hooked into HAL weak callbacks
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    can_rx_handler(hcan, CAN_RX_FIFO0);  // push to ring buffer
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    can_rx_handler(hcan, CAN_RX_FIFO1);
}
```

Main loop just drains the buffer:

```c
CanFrame rx;
while (can_recv(&rx)) { /* handle */ }
```

ISR push, main loop pop. Clean separation.

---

## 4. Error & Recovery Settings

### Production

```c
hcan1.Init.AutoBusOff = DISABLE;           // no auto-recovery from bus-off
hcan1.Init.AutoRetransmission = DISABLE;   // fire-and-forget
```

- If the bus has transient errors, the CAN controller eventually goes
  bus-off and **stays there forever** until reset.
- A single missed ACK drops the frame — no retry.
- No SCE (Status Change Error) handler.

### Torque-controller

```c
hcan1.Init.AutoBusOff = ENABLE;
hcan1.Init.AutoRetransmission = ENABLE;
```

Plus an SCE IRQ handler in `stm32f4xx_it.c`:
```c
void CAN1_SCE_IRQHandler(void) { HAL_CAN_IRQHandler(&hcan1); }
```

And error notifications enabled in `can_common_init`:
```c
uint32_t notif = CAN_IT_RX_FIFO0_MSG_PENDING |
                 CAN_IT_RX_FIFO1_MSG_PENDING |
                 CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE;
```

The bus-off state we hit during debugging earlier was recoverable because of
this — the STM32 auto-restarted, and on the Pi we added `restart-ms 100` for
the same reason. Without these, one bad cable disconnect = dead node until
power cycle.

---

## 5. Pi-side CAN Handling

### Production (`src/pi/pi_can_buffer.py`)

```python
# No filter on the socket — receives everything
bus = can.interface.Bus(channel='can1', interface='socketcan')
for msg in bus:
    process_msg(msg)

# Software filtering via dict lookup
can_id_map = {
    imu1_a_id: ("imu1", "accel"),
    imu1_g_id: ("imu1", "gyro"),
    ...
}
lookup = can_id_map.get(can_id)
if lookup is None: return   # silently drop
```

- Every CAN frame on the bus wakes up the Python thread, even unrelated ones.
- Unknown IDs are silently dropped after being decoded, parsed, and looked up.
- Pi bears the cost of filtering in userspace.

### Torque-controller (`scripts/`)

```python
# Same bus creation, no filter yet (future work)
bus = can_common.create_bus('can1')

# Sending uses shared constants:
send_torque_cmd(bus, NODE_LEFT_HIP, 1.0)
# internally: build_can_id(MSG_TORQUE_CMD, NODE_PI, NODE_LEFT_HIP)
```

Receive side would use `parse_can_id()` (not needed yet since the Pi only
transmits in this test). For the production joint controller, the Pi should
also apply **socketCAN kernel filters** so only relevant frames cross the
userspace boundary:

```python
# (Recommended, not implemented yet)
bus.set_filters([
    {"can_id": build_can_id(MSG_MOTOR_STATUS, 0, NODE_PI),
     "can_mask": 0x78F,  # type + dest
     "extended": False},
    # ... + ESTOP, IMU, etc.
])
```

---

## 6. Data Encoding

Both approaches pack int16 * scale-factor, but they differ in framing.

### Production (IMU, 6 bytes)

```c
TxData[0] = raw_ax & 0xFF;         // little-endian int16
TxData[1] = (raw_ax >> 8) & 0xFF;
TxData[2] = raw_ay & 0xFF;
TxData[3] = (raw_ay >> 8) & 0xFF;
TxData[4] = raw_az & 0xFF;
TxData[5] = (raw_az >> 8) & 0xFF;
TxHeader.DLC = 6;
```

```python
x_raw, y_raw, z_raw = struct.unpack('<hhh', msg.data[0:6])
```

Manual byte packing on STM32, matched `struct.unpack('<hhh')` on Pi. Works,
but the encoding details are duplicated in both places.

### Torque-controller (TORQUE_CMD, 2 bytes)

```c
// can_motor.c
int16_t raw = (int16_t)(torque_nm * 1000.0f);
memcpy(&data[0], &raw, 2);
can_send_std(id, data, 2);
```

```python
# can_motor.py — exactly mirrors the C code
data = struct.pack('<h', int(torque_nm * 1000))
```

The encoding is wrapped in a single named function per message type
(`can_send_torque_cmd`, `send_torque_cmd`), so callers never deal with raw
bytes. Same for parsing.

---

## 7. Transmit Pattern

### Production

Direct HAL calls in application code:
```c
TxHeader.StdId = ACCEL_HIP_R;
TxHeader.IDE = CAN_ID_STD;
TxHeader.RTR = CAN_RTR_DATA;
TxHeader.DLC = 6;
TxData[0] = ...;
HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
```

Every call site repeats the header setup. Easy to miss a field.

### Torque-controller

Application code calls a one-liner:
```c
can_send_torque_cmd(CAN_NODE_LEFT_HIP, 1.5f);
// internally builds the header, packs the data, calls HAL
```

---

## Summary — Why the New Pattern Matters

| Problem in production code              | New-pattern fix                       |
|-----------------------------------------|---------------------------------------|
| Hardcoded IDs duplicated STM32 ↔ Pi    | Shared `build_can_id` with constants  |
| No filter → can't receive anything      | 3 filter banks, 2 FIFOs               |
| Single FIFO overflow blocks commands    | FIFO0 for commands, FIFO1 for motor   |
| Bus-off is terminal                     | Auto-recovery + SCE handler           |
| Manual byte packing repeated everywhere | One encode/decode pair per message    |
| Silent frame drops on Pi                | socketCAN kernel filter (future)      |

## Migration Path for the Joint Controller

When you're ready to merge this into `src/stm32/joint-controller/`:

1. Copy `can_common.h/.c` and `can_motor.h/.c` from `torque-controller`
2. Add `can_imu.h/.c` (already in `apis/can/`) to replace the manual
   `CAN_Send_IMU_Data` in `main.c`
3. Replace `HAL_CAN_Start(&hcan1)` with `can_common_init(&hcan1, MY_NODE_ID)`
4. Add `CAN1_RX0_IRQHandler`, `CAN1_RX1_IRQHandler`, `CAN1_SCE_IRQHandler` in
   `stm32f4xx_it.c`
5. Flip `AutoBusOff` and `AutoRetransmission` to `ENABLE` in `MX_CAN1_Init`
6. Replace `CAN_Send_IMU_Data()` with `can_send_imu_accel(...)` and
   `can_send_imu_gyro(...)`
7. On the Pi (`pi_can_buffer.py`), replace the hardcoded `imu1_a_id` etc.
   with calls to `build_can_id(MSG_IMU_ACCEL, NODE_LEFT_HIP, NODE_PI)`
