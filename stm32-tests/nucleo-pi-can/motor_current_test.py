#!/usr/bin/env python3
"""
Motor Current Test — Raspberry Pi UART script for testing current commands.

Sends varying SET_CUR commands to the Nucleo-F446RE over UART for 2 minutes.
The STM32 forwards the current command to the CubeMars AK70-9 motor over CAN.
The motor shaft speed should visibly change as the current varies.

Pattern: ramps from 0 → +1.5A → 0 → -1.5A → 0 in 10-second cycles.

Usage:
  python3 motor_current_test.py                   # auto-detect port
  python3 motor_current_test.py /dev/ttyACM0      # specify port
"""

import serial
import serial.tools.list_ports
import time
import sys
import math
import signal

BAUD_RATE = 115200
TEST_DURATION_S = 120       # 2 minutes
COMMAND_INTERVAL_S = 0.25   # send a new current every 250 ms
CYCLE_PERIOD_S = 10.0       # one full sine cycle = 10 seconds
MAX_CURRENT_A = 1.5         # peak current in amps (safe for AK70-9)


def auto_detect_port():
    """Return the first ST-Link / ACM / USB-serial port, or None."""
    for port in serial.tools.list_ports.comports():
        d = (port.description or "").lower()
        dev = port.device.lower()
        if any(k in d for k in ("stlink", "stm32", "virtual com")):
            return port.device
        if "ttyacm" in dev or "usbmodem" in dev:
            return port.device
    return None


def send_and_recv(ser, cmd):
    """Send a command and return the response string."""
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode("utf-8"))
    time.sleep(0.1)
    if ser.in_waiting > 0:
        return ser.read(ser.in_waiting).decode("utf-8", errors="ignore").strip()
    return None


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else auto_detect_port()
    if port is None:
        port = "/dev/ttyACM0"
        print(f"No ST-Link port found; defaulting to {port}")
    else:
        print(f"Using port: {port}")

    print(f"Motor Current Test — {TEST_DURATION_S}s @ peak {MAX_CURRENT_A}A")
    print("=" * 55)
    print("Press Ctrl+C at any time to stop (motor will be set to 0A)")
    print("=" * 55)

    ser = serial.Serial(port, BAUD_RATE, timeout=2)
    print(f"Connected to {port}")
    time.sleep(2)  # wait for board reset after serial open

    # Pre-flight check: READ_ALL to verify motor connection
    print("\n--- Pre-flight READ_ALL check ---")
    ser.reset_input_buffer()
    ser.write(b"READALL\n")
    time.sleep(1.0)
    lines = []
    while ser.in_waiting > 0:
        raw = ser.readline().decode("utf-8", errors="ignore").strip()
        if raw:
            lines.append(raw)

    if not lines:
        print("WARNING: No response from READALL")
    else:
        for line in lines:
            print(f"  {line}")

    # Check if all motor position values are 0.00
    all_zero = all("MP:0.00" in line for line in lines if "MP:" in line)
    if all_zero and lines:
        print("\n*** ALL motor position readings are 0.00 ***")
        print("*** Motor connection may not be working.  ***")
        print("*** Verify CAN wiring and motor power.    ***")
        try:
            answer = input("\nContinue anyway? (y/n): ").strip().lower()
        except EOFError:
            answer = "n"
        if answer != "y":
            ser.close()
            print("Aborted. Serial port closed.")
            return
    else:
        print("Motor position data detected — connection looks good.")

    print("-" * 55)

    # Ensure motor starts at zero
    resp = send_and_recv(ser, "SET_CUR:0.00")
    print(f"Initial SET_CUR:0.00 -> {resp}")

    start_time = time.time()

    try:
        while True:
            elapsed = time.time() - start_time
            if elapsed >= TEST_DURATION_S:
                print(f"\n{TEST_DURATION_S}s elapsed — test complete.")
                break

            # Sine wave: smoothly varies current from -MAX to +MAX
            current = MAX_CURRENT_A * math.sin(2 * math.pi * elapsed / CYCLE_PERIOD_S)

            cmd = f"SET_CUR:{current:.2f}"
            resp = send_and_recv(ser, cmd)

            remaining = TEST_DURATION_S - elapsed
            print(f"[{elapsed:6.1f}s / {remaining:5.1f}s left]  {cmd:20s}  ->  {resp}")

            time.sleep(COMMAND_INTERVAL_S)

    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")

    finally:
        # Always set current to zero on exit
        print("Setting motor current to 0A...")
        send_and_recv(ser, "SET_CUR:0.00")
        time.sleep(0.1)
        ser.close()
        print("Motor stopped. Serial port closed.")


if __name__ == "__main__":
    main()
