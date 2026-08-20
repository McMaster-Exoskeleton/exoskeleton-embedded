# Two-Joint Bring-Up Guide

End-to-end procedure for testing the exoskeleton with **both hips + Raspberry Pi**
running the merged joint-controller firmware (IMU streaming + motor control) and
the `pi_imu_torque.py` test/template script.

---

## 1. Overview

| Node | Hardware | Role |
|---|---|---|
| Pi | Raspberry Pi + CAN HAT (or USB-CAN) | Sends `TORQUE_CMD` and `ESTOP`, buffers IMU, runs `pi_imu_torque.py` |
| Left hip | STM32F446RE Nucleo + LSM6DS3TR-C IMU + (optional) AK70-9 motor via VESC | `MY_NODE_ID = CAN_NODE_LEFT_HIP` |
| Right hip | STM32F446RE Nucleo + LSM6DS3TR-C IMU + (optional) AK70-9 motor via VESC | `MY_NODE_ID = CAN_NODE_RIGHT_HIP` |

Same firmware project (`src/stm32/joint-controller/`) flashes both boards;
only the per-board `#define`s change.

---

## 2. Pre-flight checklist

- [ ] 2× STM32F446RE Nucleo boards
- [ ] 2× LSM6DS3TR-C breakouts wired to I²C3 (PB0 = SCL, PB4 = SDA per the
      project's `.ioc`), SA0 = HIGH (3.3 V) so the I²C address is `0x6B`
- [ ] 2× CAN transceivers (TJA1050, SN65HVD230, MCP2551, etc.) wired to
      `PB8 = CAN1_RX`, `PB9 = CAN1_TX`
- [ ] Raspberry Pi with CAN interface (HAT or USB adapter), enumerating as
      `can0` or `can1` (note which — you'll need it later)
- [ ] 2× 120 Ω termination resistors (one for each end of the CAN bus)
- [ ] (Optional, for full motor test) 2× AK70-9 KV60 motors with VESC
      firmware, each programmed to a unique controller ID
- [ ] STM32CubeIDE installed for flashing
- [ ] Serial monitor (PuTTY, screen, minicom, or CubeIDE's built-in) at
      **115200 8N1** for each STM32

---

## 3. Per-board firmware configuration

Edit `src/stm32/joint-controller/Core/Src/main.c` between flashes:

| Setting | Line | Left hip value | Right hip value |
|---|---|---|---|
| `MY_NODE_ID` | ~47 | `CAN_NODE_LEFT_HIP` | `CAN_NODE_RIGHT_HIP` |
| `MY_MOTOR_CAN_ID` | ~54 | `104` (or your left motor's VESC ID) | `106` (or your right motor's VESC ID) |

**If you have no motor wired**, leave both at `104` — the joint will emit
unanswered VESC frames into the void and you'll see
`[MOTOR] no feedback for >500 ms` in the logs (expected).

**If both hips have motors on the same bus**, the controller IDs **must
differ** — program them via the VESC tool before connecting.

---

## 4. Flash both STM32s (CubeIDE)

For each board, in order:

1. `File → Open Projects from File System… → src/stm32/joint-controller`
2. Edit `MY_NODE_ID` (and `MY_MOTOR_CAN_ID` if motors are wired) for the
   board you're about to flash.
3. **Plug in only that board's ST-Link** so CubeIDE doesn't pick the wrong one.
4. `Project → Build All` → green Debug arrow → flashes and halts at `main()`.
   Click **Resume**, then disconnect.
5. Edit the `#define`s for the other board, plug it in, repeat.

After flashing, both boards should boot independently and immediately start
streaming IMU on their next power-up.

---

## 5. Pi setup (one-time)

```bash
sudo apt update
sudo apt install -y can-utils python3-pip
pip3 install python-can
```

Bring up the CAN interface at 1 Mbit/s. Substitute `can0` if that's what your
HAT enumerates as:

```bash
sudo ip link set can1 up type can bitrate 1000000
```

To make this persistent across reboots, add it to `/etc/network/interfaces`
or a systemd unit; otherwise re-run after each boot.

---

## 6. Get the script onto the Pi

The script imports from `apis/can/python/`, so you need that tree present too.
Easiest: clone the repo on the Pi.

```bash
# on the Pi
cd ~
git clone <your-repo-url> exoskeleton-embedded
cd exoskeleton-embedded
git checkout embedded-system-v1
```

Or `scp` only the two trees you need (preserve structure):

```bash
# from your dev machine
scp -r src/pi    pi@<pi-ip>:~/exoskeleton-embedded/src/
scp -r apis/can  pi@<pi-ip>:~/exoskeleton-embedded/apis/can/
```

---

## 7. Wiring

Each STM32 needs a CAN transceiver — the F446RE only outputs logic-level
`CAN_TX`/`CAN_RX`, not differential `CAN_H`/`CAN_L`. Default Nucleo pins:
`PB8 = CAN1_RX`, `PB9 = CAN1_TX` (verify against the project's `.ioc`).

```
[ Pi CAN HAT ]──CAN_H/CAN_L/GND──[ Left Hip transceiver ]──CAN_H/CAN_L/GND──[ Right Hip transceiver ]
       │                                                                                  │
       └──── 120 Ω across H/L ─────── (no termination here) ─────── 120 Ω across H/L ─────┘
```

Critical:

- **120 Ω termination at the two physical ends of the bus only** (not at
  every node).
- **GND tied across all three boards** — without a common ground reference
  the differential signaling drifts and you'll see persistent CAN errors.
- Keep the CAN_H/CAN_L pair as a twisted pair on stub wires; impedance
  mismatch on long stubs degrades signal integrity at 1 Mbit/s.

---

## 8. Run and verify

### 8.1 Sniff first

Before launching the Python script, check that both hips are talking on the
bus using `candump`:

```bash
candump can1
```

Expected IDs at ~500 Hz per hip:

| Frame | ID (hex) | Source |
|---|---|---|
| `IMU_ACCEL` from left hip  | `0x190` | LEFT_HIP=1 |
| `IMU_GYRO`  from left hip  | `0x210` | LEFT_HIP=1 |
| `IMU_ACCEL` from right hip | `0x1A0` | RIGHT_HIP=2 |
| `IMU_GYRO`  from right hip | `0x220` | RIGHT_HIP=2 |

**If you only see one set of IDs:** only one board is up, or both boards
were flashed with the same `MY_NODE_ID`.

### 8.2 Open serial consoles

Open a serial monitor on each STM32's USB-VCP at **115200 8N1** *before*
powering up so you catch the boot lines:

```bash
# Linux example, adjust device path
screen /dev/ttyACM0 115200      # left hip
screen /dev/ttyACM1 115200      # right hip   (different terminal)
```

### 8.3 Launch the test script

```bash
cd ~/exoskeleton-embedded
python3 src/pi/pi_imu_torque.py can1
```

### 8.4 Smoke tests

| Type | Effect |
|---|---|
| `l1` / `l2` | Latest IMU sample from left / right hip |
| `q1` / `q2` | 187-sample queue snapshot (downsampled from 500 Hz) |
| `m1` / `m2` | Latest motor status — currently always says "no motor status received yet" because the joint-controller doesn't republish `MOTOR_STATUS` on the bus yet |
| `t 1 0.5` | Send 0.5 Nm to left hip → STM32 emits VESC `SET_CURRENT` extended frames; if a motor is wired and powered, **it will move** |
| `tall 0` | Zero torque on every joint |
| `tstop` | Same as `tall 0` |
| `estop` | Broadcast ESTOP — latches both STM32s until power-cycle |

---

## 9. Real-time logging — what to expect

Both UART consoles and the Pi terminal stream events independently.
Logging is **edge-triggered** (state changes only) and **rate-limited**
(aggregated counters drained at 1 Hz) so the UART doesn't drown out the
real signals at 500 Hz IMU + 20 Hz motor refresh.

### 9.1 STM32 boot (per board, on each power-up)

```
[BOOT] joint-controller starting
[BOOT] node=2 motor=104
[INIT] CAN ok
[INIT] IMU connected
[INIT] calibrating IMU (~0.3 s, keep sensor still)
[INIT] ready
```

### 9.2 STM32 events (logged once per state change)

| Line | Meaning |
|---|---|
| `[FATAL] CAN init failed` | Halt. Check `.ioc` CAN1 enable, transceiver wiring. |
| `[FATAL] IMU not detected (WHO_AM_I mismatch)` | Halt. Check I²C wiring, SA0, power. |
| `[FATAL] Error_Handler invoked — halting` | Halt from any HAL `_Init` failure. |
| `[ESTOP] latched src=0 reason=0` | Pi sent the kill switch; ignored until power-cycle. |
| `[CAN] first TORQUE_CMD: 500 mNm -> 349 mA` | Confirmation the dispatch path actually fired. |
| `[CAN] TORQUE_CMD parse failed (DLC=N)` | Malformed torque command — should be 2 bytes. |
| `[MOTOR] error 0 -> 2 (OVER_CURRENT)` | VESC fault transition. Only logs on changes. |
| `[MOTOR] no feedback for >500 ms — check power/wiring/CAN ID` | Sent torque, no response. |
| `[MOTOR] feedback resumed` | Motor came back after a stale period. |
| `[IMU] state LOST` / `[IMU] state CONNECTED` | Sensor disconnected/reconnected mid-run. |
| `[CAN] bus_off=1 (TEC=255 REC=0)` | Bus collapsed; AutoBusOff recovers automatically. |
| `[CAN] error_passive=1 (...)` | TEC > 127, recoverable. |

### 9.3 STM32 aggregated counters (1 Hz, only when non-zero)

```
[CAN] RX ring overflow +3 (total 7)      <- main loop stalled or burst above drain rate
[CAN] TX dropped +12 (total 412)         <- mailbox full at send time; expected on busy bus
[CAN] error ISR +1 (total 8)             <- single-bit / bit-stuff / form errors
[I2C] error ISR +1 (total 1)             <- LSM6DS3TR NACK / arbitration loss
```

A handful of `TX dropped` per second on the IMU 500 Hz path is **normal**
under load — AutoRetransmission absorbs the loss. Investigate if the
counter climbs >50/s sustained.

### 9.4 Pi console events

```
[NODE] first IMU_ACCEL from left_hip          <- once per (node, msg_type) pair
[NODE] first IMU_GYRO  from left_hip
[NODE] first IMU_ACCEL from right_hip
...
[STALE] left_hip silent for 1.2s              <- once when threshold crossed
[ALIVE] left_hip traffic resumed              <- once when traffic returns
[MOTOR] right_hip error 0 -> 2                <- transition only
[ESTOP] from node 0 (reason 0)
[CAN] bus recv error: <details> (retry in 0.5 s)
[CAN] cannot unpack frame 0x190: ...          <- malformed frame
```

A bus-open failure quits cleanly with the exact `ip link set` hint instead
of a stack trace.

---

## 10. Triage cheat sheet

| Symptom | Likely cause | First thing to try |
|---|---|---|
| `[FATAL] IMU not detected` | I²C wiring, SA0 pin, no power | Probe SCL/SDA on scope; check WHO_AM_I on bus with logic analyzer |
| `[FATAL] CAN init failed` | `hcan1` not enabled in `.ioc`, filter bank conflict | Re-generate code from CubeMX |
| `[CAN] RX ring overflow +N` rising | Main loop stalled (UART blocking?), traffic > drain rate | Check UART consumer; reduce log frequency |
| `[CAN] error ISR` climbing fast | Termination missing/wrong, mismatched bitrate | Verify 120 Ω at both ends; verify all nodes at 1 Mbit/s |
| `[CAN] bus_off=1` repeating | One node out of sync — usually wrong prescaler / different bitrate | Confirm PCLK1 = 42 MHz on F446 → prescaler 3, BS1 10TQ, BS2 3TQ |
| `[I2C] error ISR` climbing | LSM6DS3TR cabling or pull-up issue, NACKs | Check SCL/SDA pull-ups (4.7 kΩ to 3.3 V) |
| `[MOTOR] no feedback` after `t 1 0.5` | Motor power off, wrong `MY_MOTOR_CAN_ID`, VESC asleep | Verify VESC controller ID matches `MY_MOTOR_CAN_ID`; check 48 V supply |
| Pi `[STALE]` for one hip | That hip stopped sending | Check that hip's UART for `[ESTOP]`, `[IMU] state LOST`, or rising error counts |
| Pi `[STALE]` for both hips | Bus issue, not joint issue | Check Pi CAN HAT, termination, transceiver power |
| Pi: nothing arrives at all | Bus down or HAT mis-configured | `candump can1` first; if also empty, the problem is below the script |
| `MOTOR error 0 -> 2 (OVER_CURRENT)` | Actual motor fault — current draw exceeded limit | Reduce commanded torque; check mechanical load |
| `[ESTOP] latched` then no further response | Working as intended — power-cycle to clear | Send `tstop` first, then power-cycle the joint |

---

## 11. Safety notes

- **Until both `MY_NODE_ID`s and `MY_MOTOR_CAN_ID`s are confirmed correct,
  leave motor power off.** The full CAN dispatch path (TORQUE_CMD → VESC
  command) is exercised on STM32 power alone — you don't need a motor to
  validate the firmware.
- **The `t <node> <Nm>` command will spin a connected motor immediately.**
  No ramp-up, no soft-start. Keep clear of moving parts.
- **`Ctrl+C` and `quit` both broadcast ESTOP** before closing the bus.
  This latches every joint to zero current until you power-cycle each STM32.
- **The current limit (`CURRENT_LIMIT = 5.0 A` in `main.c`) is a software
  clamp**, not a hardware fuse. Don't rely on it as your only over-current
  protection during a real test.
- **A `[STALE]` warning on a joint while motor is active is critical** —
  the joint may have dropped off the bus and stopped responding to ESTOP.
  Cut motor power immediately if this happens unexpectedly.

---

## 12. Known gaps / next steps

- The joint-controller receives motor feedback locally into `motor_status`
  but doesn't re-publish `MOTOR_STATUS` over CAN. The Pi's `m1`/`m2`
  commands therefore always report "no motor status received yet" until a
  ~100 Hz publisher is wired in.
- `can_system.h` is included and compiled, but `can_parse_estop()` is
  currently called only to extract the reason byte for the log line — no
  reason-specific handling exists yet (e.g., distinguishing `MANUAL` from
  `MOTOR_ERROR`).
- No heartbeat (`MSG_HEARTBEAT`) is emitted by either side yet, so
  comm-loss detection on the STM32 relies on TORQUE_CMD silence rather
  than an explicit Pi liveness signal.
