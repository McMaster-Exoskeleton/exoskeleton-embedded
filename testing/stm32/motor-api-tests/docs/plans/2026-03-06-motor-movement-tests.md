# Motor Movement Tests Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add UART commands for all AK70-9 motor control functions with safety limits, a software watchdog, and a test cases document.

**Architecture:** All safety limits are `#define` constants in `ak70_9.h`, clamped in `uart_cmd.c` before calling motor functions. A software watchdog in `main.c` auto-stops the motor if no UART command arrives within 2 seconds. The Python script adds confirmation prompts for motor commands.

**Tech Stack:** STM32 HAL (C), Python 3 + pyserial

---

### Task 1: Add motor CAN ID and test-safe limits to ak70_9.h

**Files:**
- Modify: `Core/Inc/can/ak70_9.h:32-60`

**Step 1: Add constants after the existing motor parameter limits block**

Add `MOTOR_CAN_ID` and test-safe limit constants:
```c
#define MOTOR_CAN_ID               104

/* Test-safe limits (conservative ceilings for bench testing) */
#define TEST_DUTY_MAX              (0.15f)
#define TEST_CURRENT_MAX           (5.0f)
#define TEST_BRAKE_CURRENT_MAX     (5.0f)
#define TEST_RPM_MAX               (5000.0f)
#define TEST_POS_DEG_MAX           (360.0f)
#define TEST_MIT_P_MAX             (3.14f)
#define TEST_MIT_V_MAX             (5.0f)
#define TEST_MIT_T_MAX             (5.0f)
#define TEST_MIT_KP_MAX            (50.0f)
#define TEST_MIT_KD_MAX            (2.5f)
```

**Step 2: Commit**
```
feat: add motor CAN ID (104) and test-safe limits to ak70_9.h
```

---

### Task 2: Update uart_cmd.h with new command declarations

**Files:**
- Modify: `Core/Inc/can/uart_cmd.h`

**Step 1: Update header comment to document new commands and add watchdog tick declaration**

---

### Task 3: Implement motor control commands + ESTOP + watchdog in uart_cmd.c

**Files:**
- Modify: `Core/Src/can/uart_cmd.c`

**Step 1: Add command parsing for all motor control functions**

New commands: ESTOP, SET_DUTY, SET_CURRENT, SET_BRAKE, SET_RPM, SET_POS, SET_ORIGIN, SET_POS_SPD, SET_MIT

Each command:
1. Parses parameters from the command string (space-separated)
2. Validates against test-safe limits, returns ERR:RANGE if exceeded
3. Clamps to test-safe limits
4. Calls the corresponding motor API function with `MOTOR_CAN_ID`
5. Sends OK response with the actual values sent

**Step 2: Add watchdog tick function**

`uart_cmd_watchdog_tick()` called from main loop — checks if 2 seconds have elapsed since last command, sends zero-duty to stop motor.

---

### Task 4: Add watchdog to main.c main loop

**Files:**
- Modify: `Core/Src/main.c:109-126`

**Step 1: Add watchdog tick call in main loop**

---

### Task 5: Update motor_test.py with motor control commands

**Files:**
- Modify: `scripts/motor_test.py`

**Step 1: Add motor control commands with confirmation prompts**

New commands in the interactive loop. Motor control commands print values and require 'y' confirmation. ESTOP sends immediately.

---

### Task 6: Create test cases document

**Files:**
- Create: `docs/motor-test-cases.md`

**Step 1: Write test procedures for all motor functions with expected results**
