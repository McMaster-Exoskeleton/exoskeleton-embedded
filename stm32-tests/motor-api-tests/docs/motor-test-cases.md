# AK70-9 Motor API Test Cases

Manual test procedures for verifying each motor control function.
Run these tests using `python scripts/motor_test.py` with the motor powered and connected.

**Motor CAN ID:** 104

**Safety notes:**
- The STM32 firmware clamps all values to test-safe limits before sending to the motor.
- The firmware continuously re-sends the last motor command every 50ms to keep the VESC firmware's internal timeout from stopping the motor. Send `ESTOP` to stop.
- The Python script sends ESTOP automatically on quit or Ctrl+C.
- Always have physical access to the motor power supply as a last-resort kill switch.

**How motor commands work:**
- Motor control commands (SET_DUTY, SET_CURRENT, etc.) are sent once AND refreshed every 50ms automatically by the STM32.
- The motor runs indefinitely until you send `ESTOP`, `SET_DUTY 0`, or another command that replaces it.
- `SET_ORIGIN` is a one-shot configuration command and is not refreshed.

---

## Prerequisites

Before running any tests:

| Step | Action | Expected |
|------|--------|----------|
| P1 | Wire CAN transceiver between Nucleo and motor (see testing-instructions.md) | Hardware connected |
| P2 | Power the AK70-9 motor | Motor powered, no error LEDs |
| P3 | Flash firmware to Nucleo via STM32CubeIDE | Flash successful |
| P4 | Run `python scripts/motor_test.py` | "Connected to COMx at 115200 baud" |
| P5 | Send `PING` | Response: `PONG` |
| P6 | Send `READ_ERR` | Response: `ERR:0:NONE` |

If P5 or P6 fail, do not proceed. Debug the connection first.

---

## Test 1: ESTOP (Emergency Stop)

**Purpose:** Verify the emergency stop command immediately halts the motor and stops command refresh.

| ID | Procedure | Command | Expected UART Response | Expected Motor Behavior | Pass/Fail |
|----|-----------|---------|----------------------|------------------------|-----------|
| 1.1 | Send ESTOP with motor idle | `ESTOP` | `OK:ESTOP` | No change (already stopped) | |
| 1.2 | Start motor with SET_DUTY 0.05, wait 5s to confirm it keeps running, then ESTOP | `SET_DUTY 0.05` then wait 5s then `ESTOP` | `OK:ESTOP` | Motor spins continuously for 5s, then stops immediately on ESTOP | |
| 1.3 | Start motor with SET_RPM 1000, wait 5s, then ESTOP | `SET_RPM 1000` then wait 5s then `ESTOP` | `OK:ESTOP` | Motor spins continuously for 5s, then stops immediately on ESTOP | |

---

## Test 2: Command Persistence

**Purpose:** Verify that motor commands run indefinitely until replaced or stopped.

| ID | Procedure | Command | Expected UART Response | Expected Motor Behavior | Pass/Fail |
|----|-----------|---------|----------------------|------------------------|-----------|
| 2.1 | Send SET_DUTY 0.05, wait 30 seconds | `SET_DUTY 0.05` | `OK:SET_DUTY:0.0500` | Motor spins continuously for the entire 30 seconds | |
| 2.2 | While motor is running from 2.1, send READ_ALL | `READ_ALL` | `ALL:POS=...,SPD=<non-zero>,...` | Motor keeps spinning while read commands are processed | |
| 2.3 | Replace command: send SET_DUTY 0.10 | `SET_DUTY 0.10` | `OK:SET_DUTY:0.1000` | Motor speeds up (new duty replaces old) | |
| 2.4 | Replace with different mode: send SET_RPM 1000 | `SET_RPM 1000` | `OK:SET_RPM:1000.0` | Motor switches to RPM control seamlessly | |
| 2.5 | Stop with zero value | `SET_DUTY 0` | `OK:SET_DUTY:0.0000` | Motor stops, command refresh stops | |
| 2.6 | ESTOP after test | `ESTOP` | `OK:ESTOP` | Motor stopped | |

---

## Test 3: SET_DUTY (Duty Cycle Control)

**Purpose:** Verify duty cycle control with forward, reverse, and clamping.

| ID | Procedure | Command | Expected UART Response | Expected Motor Behavior | Pass/Fail |
|----|-----------|---------|----------------------|------------------------|-----------|
| 3.1 | Small positive duty | `SET_DUTY 0.05` | `OK:SET_DUTY:0.0500` | Motor spins slowly clockwise, continues indefinitely | |
| 3.2 | Medium positive duty | `SET_DUTY 0.10` | `OK:SET_DUTY:0.1000` | Motor spins faster clockwise | |
| 3.3 | Maximum test duty | `SET_DUTY 0.15` | `OK:SET_DUTY:0.1500` | Motor spins at max test speed CW | |
| 3.4 | Small negative duty (reverse) | `SET_DUTY -0.05` | `OK:SET_DUTY:-0.0500` | Motor spins slowly counter-clockwise | |
| 3.5 | Over-limit clamping (positive) | `SET_DUTY 0.50` | `OK:SET_DUTY:0.1500` | Motor spins at max test speed (clamped to 0.15) | |
| 3.6 | Over-limit clamping (negative) | `SET_DUTY -0.50` | `OK:SET_DUTY:-0.1500` | Motor spins at max test speed CCW (clamped to -0.15) | |
| 3.7 | Zero duty (stop) | `SET_DUTY 0` | `OK:SET_DUTY:0.0000` | Motor stops | |
| 3.8 | ESTOP after test | `ESTOP` | `OK:ESTOP` | Motor stopped | |

---

## Test 4: SET_CURRENT (Torque Control)

**Purpose:** Verify current/torque control.

| ID | Procedure | Command | Expected UART Response | Expected Motor Behavior | Pass/Fail |
|----|-----------|---------|----------------------|------------------------|-----------|
| 4.1 | Small positive current | `SET_CURRENT 1.0` | `OK:SET_CURRENT:1.00` | Motor applies torque CW continuously; if unloaded, shaft spins | |
| 4.2 | Medium current | `SET_CURRENT 3.0` | `OK:SET_CURRENT:3.00` | More torque, faster spin if unloaded | |
| 4.3 | Negative current (reverse torque) | `SET_CURRENT -1.0` | `OK:SET_CURRENT:-1.00` | Motor applies torque CCW | |
| 4.4 | Over-limit clamping | `SET_CURRENT 20.0` | `OK:SET_CURRENT:5.00` | Torque clamped to 5A | |
| 4.5 | Verify READ_CUR during torque | `READ_CUR` | `CUR:<non-zero value>` | Current reading should approximate commanded value | |
| 4.6 | Zero current (stop) | `SET_CURRENT 0` | `OK:SET_CURRENT:0.00` | Motor stops producing torque | |
| 4.7 | ESTOP after test | `ESTOP` | `OK:ESTOP` | Motor stopped | |

---

## Test 5: SET_BRAKE (Current Brake)

**Purpose:** Verify brake current holds the motor in position.

| ID | Procedure | Command | Expected UART Response | Expected Motor Behavior | Pass/Fail |
|----|-----------|---------|----------------------|------------------------|-----------|
| 5.1 | Apply brake current | `SET_BRAKE 2.0` | `OK:SET_BRAKE:2.00` | Motor resists being turned by hand | |
| 5.2 | Increase brake current | `SET_BRAKE 5.0` | `OK:SET_BRAKE:5.00` | Motor strongly resists being turned | |
| 5.3 | Negative value (clamped to 0) | `SET_BRAKE -1.0` | `OK:SET_BRAKE:0.00` | No braking (clamped to 0) | |
| 5.4 | Over-limit clamping | `SET_BRAKE 30.0` | `OK:SET_BRAKE:5.00` | Brake clamped to 5A | |
| 5.5 | Release brake | `SET_BRAKE 0` | `OK:SET_BRAKE:0.00` | Motor can be turned freely by hand | |
| 5.6 | ESTOP after test | `ESTOP` | `OK:ESTOP` | Motor stopped | |

---

## Test 6: SET_RPM (Velocity Control)

**Purpose:** Verify velocity (RPM) control.

| ID | Procedure | Command | Expected UART Response | Expected Motor Behavior | Pass/Fail |
|----|-----------|---------|----------------------|------------------------|-----------|
| 6.1 | Low positive RPM | `SET_RPM 1000` | `OK:SET_RPM:1000.0` | Motor spins CW at ~1000 ERPM continuously | |
| 6.2 | Medium positive RPM | `SET_RPM 3000` | `OK:SET_RPM:3000.0` | Motor spins faster CW | |
| 6.3 | Negative RPM (reverse) | `SET_RPM -1000` | `OK:SET_RPM:-1000.0` | Motor spins CCW | |
| 6.4 | Over-limit clamping | `SET_RPM 50000` | `OK:SET_RPM:5000.0` | Speed clamped to 5000 ERPM | |
| 6.5 | Verify READ_SPD during spin | `READ_SPD` | `SPD:<value near commanded RPM>` | Speed reading should approximate commanded RPM | |
| 6.6 | Zero RPM (stop) | `SET_RPM 0` | `OK:SET_RPM:0.0` | Motor decelerates to stop | |
| 6.7 | ESTOP after test | `ESTOP` | `OK:ESTOP` | Motor stopped | |

---

## Test 7: SET_POS (Position Control)

**Purpose:** Verify absolute position control. Position commands are also refreshed to maintain the hold.

| ID | Procedure | Command | Expected UART Response | Expected Motor Behavior | Pass/Fail |
|----|-----------|---------|----------------------|------------------------|-----------|
| 7.1 | Set origin first | `SET_ORIGIN 0` | `OK:SET_ORIGIN:0` | Current position becomes 0 degrees | |
| 7.2 | Move to +90 degrees | `SET_POS 90` | `OK:SET_POS:90.00` | Motor rotates ~90 degrees CW from origin and holds | |
| 7.3 | Verify position | `READ_POS` | `POS:<value near 90.00>` | Position reading should be ~90 degrees | |
| 7.4 | Move to -90 degrees | `SET_POS -90` | `OK:SET_POS:-90.00` | Motor rotates to -90 degrees (180 degrees from test 7.2) | |
| 7.5 | Move to +360 degrees (full revolution) | `SET_POS 360` | `OK:SET_POS:360.00` | Motor rotates to +360 degrees (1 full CW revolution from origin) | |
| 7.6 | Over-limit clamping | `SET_POS 1000` | `OK:SET_POS:360.00` | Position clamped to 360 degrees | |
| 7.7 | Return to origin | `SET_POS 0` | `OK:SET_POS:0.00` | Motor returns to origin position | |
| 7.8 | ESTOP after test | `ESTOP` | `OK:ESTOP` | Motor stopped | |

---

## Test 8: SET_ORIGIN (Set Zero Position)

**Purpose:** Verify setting the motor origin/zero position. This is a one-shot command (not refreshed).

| ID | Procedure | Command | Expected UART Response | Expected Motor Behavior | Pass/Fail |
|----|-----------|---------|----------------------|------------------------|-----------|
| 8.1 | Move motor to arbitrary position by hand | (manual) | N/A | Motor at some non-zero position | |
| 8.2 | Read current position | `READ_POS` | `POS:<non-zero value>` | Shows current position | |
| 8.3 | Set temporary origin | `SET_ORIGIN 0` | `OK:SET_ORIGIN:0` | Position resets to 0 | |
| 8.4 | Read position after origin set | `READ_POS` | `POS:0.00` (or near 0) | Position should now read ~0 | |
| 8.5 | Move to 45 degrees from new origin | `SET_POS 45` | `OK:SET_POS:45.00` | Motor moves 45 degrees from new origin | |
| 8.6 | Invalid mode value | `SET_ORIGIN 5` | `ERR:PARSE:SET_ORIGIN` | No change, error response | |

**Warning:** `SET_ORIGIN 1` writes to motor flash memory. Only use if you want a permanent origin. Use `SET_ORIGIN 0` for testing.

---

## Test 9: SET_POS_SPD (Position + Velocity + Acceleration)

**Purpose:** Verify smooth trajectory control with speed and acceleration limits.

| ID | Procedure | Command | Expected UART Response | Expected Motor Behavior | Pass/Fail |
|----|-----------|---------|----------------------|------------------------|-----------|
| 9.1 | Set origin | `SET_ORIGIN 0` | `OK:SET_ORIGIN:0` | Position zeroed | |
| 9.2 | Slow move to +180 degrees (spd=1000, acc=500) | `SET_POS_SPD 180 1000 500` | `OK:SET_POS_SPD:180.00,1000,500` | Motor slowly accelerates, moves to 180 degrees, decelerates, holds | |
| 9.3 | Fast move to -180 degrees (spd=5000, acc=5000) | `SET_POS_SPD -180 5000 5000` | `OK:SET_POS_SPD:-180.00,5000,5000` | Motor moves faster to -180 degrees | |
| 9.4 | Position over-limit clamping | `SET_POS_SPD 1000 1000 500` | `OK:SET_POS_SPD:360.00,1000,500` | Position clamped to 360 degrees | |
| 9.5 | Return to origin slowly | `SET_POS_SPD 0 500 200` | `OK:SET_POS_SPD:0.00,500,200` | Motor returns smoothly to origin | |
| 9.6 | ESTOP after test | `ESTOP` | `OK:ESTOP` | Motor stopped | |

---

## Test 10: SET_MIT (MIT Force/Impedance Control)

**Purpose:** Verify MIT mode impedance control. This is the most complex mode -- test carefully.

**Parameter reference:**
- `p` = desired position (rad), `v` = desired velocity (rad/s)
- `kp` = position gain, `kd` = damping gain, `t` = torque feedforward (Nm)
- Motor control law: `torque = kp * (p_des - p_actual) + kd * (v_des - v_actual) + t_ff`

| ID | Procedure | Command | Expected UART Response | Expected Motor Behavior | Pass/Fail |
|----|-----------|---------|----------------------|------------------------|-----------|
| 10.1 | Zero command (safe start) | `SET_MIT 0 0 0 0 0` | `OK:SET_MIT:0.000,0.000,0.0,0.00,0.000` | No movement (all zeros) | |
| 10.2 | Position hold with spring (kp only) | `SET_MIT 0 0 10 0.5 0` | `OK:SET_MIT:0.000,0.000,10.0,0.50,0.000` | Motor holds at 0 rad; resists displacement like a spring | |
| 10.3 | Move to +1 radian with spring | `SET_MIT 1.0 0 10 0.5 0` | `OK:SET_MIT:1.000,0.000,10.0,0.50,0.000` | Motor moves to ~1 radian (~57 degrees) CW and holds | |
| 10.4 | Move to -1 radian | `SET_MIT -1.0 0 10 0.5 0` | `OK:SET_MIT:-1.000,0.000,10.0,0.50,0.000` | Motor moves to ~-1 radian CCW and holds | |
| 10.5 | Velocity command only | `SET_MIT 0 2.0 0 1.0 0` | `OK:SET_MIT:0.000,2.000,0.0,1.00,0.000` | Motor spins continuously at ~2 rad/s | |
| 10.6 | Torque feedforward only | `SET_MIT 0 0 0 0 1.0` | `OK:SET_MIT:0.000,0.000,0.0,0.00,1.000` | Motor applies 1 Nm torque continuously (spins if unloaded) | |
| 10.7 | High stiffness hold | `SET_MIT 0 0 50 2.5 0` | `OK:SET_MIT:0.000,0.000,50.0,2.50,0.000` | Motor strongly holds at 0 rad, very stiff to turn by hand | |
| 10.8 | Over-limit clamping (position) | `SET_MIT 10 0 10 0.5 0` | `OK:SET_MIT:3.140,0.000,10.0,0.50,0.000` | Position clamped to pi radians | |
| 10.9 | Over-limit clamping (kp) | `SET_MIT 0 0 200 0.5 0` | `OK:SET_MIT:0.000,0.000,50.0,0.50,0.000` | Kp clamped to 50 | |
| 10.10 | Zero command to release | `SET_MIT 0 0 0 0 0` | `OK:SET_MIT:0.000,0.000,0.0,0.00,0.000` | Motor releases, can be turned freely | |
| 10.11 | ESTOP after test | `ESTOP` | `OK:ESTOP` | Motor stopped | |

---

## Test 11: Error Handling

**Purpose:** Verify parse error responses for malformed commands.

| ID | Procedure | Command | Expected UART Response | Expected Motor Behavior | Pass/Fail |
|----|-----------|---------|----------------------|------------------------|-----------|
| 11.1 | SET_DUTY with no parameter | `SET_DUTY` (note: no space after) | `UNKNOWN_CMD:SET_DUTY` | No movement | |
| 11.2 | SET_DUTY with non-numeric parameter | `SET_DUTY abc` | `ERR:PARSE:SET_DUTY` | No movement | |
| 11.3 | SET_RPM with no parameter | `SET_RPM ` | `ERR:PARSE:SET_RPM` | No movement | |
| 11.4 | SET_POS_SPD with missing parameters | `SET_POS_SPD 90` | `ERR:PARSE:SET_POS_SPD` | No movement | |
| 11.5 | SET_MIT with missing parameters | `SET_MIT 1.0 0` | `ERR:PARSE:SET_MIT` | No movement | |
| 11.6 | SET_ORIGIN with invalid mode | `SET_ORIGIN 5` | `ERR:PARSE:SET_ORIGIN` | No movement | |
| 11.7 | Unknown command | `FOOBAR` | `UNKNOWN_CMD:FOOBAR` | No movement | |

---

## Test 12: Combined Workflow

**Purpose:** Verify a realistic test workflow combining multiple functions.

| ID | Procedure | Command | Expected UART Response | Expected Motor Behavior | Pass/Fail |
|----|-----------|---------|----------------------|------------------------|-----------|
| 12.1 | Verify connection | `PING` | `PONG` | N/A | |
| 12.2 | Check for errors | `READ_ERR` | `ERR:0:NONE` | N/A | |
| 12.3 | Set temporary origin | `SET_ORIGIN 0` | `OK:SET_ORIGIN:0` | Position zeroed | |
| 12.4 | Read position (should be ~0) | `READ_POS` | `POS:0.00` | N/A | |
| 12.5 | Move to 90 degrees | `SET_POS 90` | `OK:SET_POS:90.00` | Motor rotates to 90 degrees and holds | |
| 12.6 | Wait 10 seconds, read position | `READ_POS` | `POS:<~90.00>` | Motor still holding at 90 degrees | |
| 12.7 | Read all values | `READ_ALL` | `ALL:POS=~90,SPD=~0,...` | Position ~90, speed ~0 (settled) | |
| 12.8 | Move back to origin | `SET_POS 0` | `OK:SET_POS:0.00` | Motor returns to origin | |
| 12.9 | Apply gentle torque | `SET_CURRENT 1.0` | `OK:SET_CURRENT:1.00` | Motor applies small torque continuously | |
| 12.10 | Wait 10 seconds, read current | `READ_CUR` | `CUR:<~1.00>` | Motor still applying torque after 10s | |
| 12.11 | Emergency stop | `ESTOP` | `OK:ESTOP` | Motor stops immediately | |
| 12.12 | Verify stopped | `READ_ALL` | `ALL:POS=...,SPD=0.0,...` | Speed should be 0 | |
