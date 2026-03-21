#!/usr/bin/env python3
"""
UART IMU Test Script for STM32 Nucleo F446RE + LSM6DS3TR-C
Sends commands to the microcontroller and displays IMU data responses.

Commands:
  READ       - Latest filtered accel + gyro (single reading from imu_data struct)
  READLATEST - Latest reading from the 100-sample circular buffer
  READALL    - All 100 stored readings (oldest first) from the circular buffer
  STATUS     - IMU I2C connectivity status
  REGISTER   - I2C device address
  CONFIG     - Gyro and accel control register values
  POWER      - Power configuration register value
"""

import serial
import serial.tools.list_ports
import time
import sys

COMMANDS = ["READ", "READLATEST", "READALL", "STATUS", "REGISTER", "CONFIG", "POWER"]

# READALL streams up to 100 lines; give it more time than single-line commands.
READALL_TIMEOUT = 15.0  # seconds


def auto_detect_port():
    """Return the first ST-Link virtual COM port found, or None."""
    for port in serial.tools.list_ports.comports():
        desc = (port.description or "").lower()
        if "stlink" in desc or "stm32" in desc or "virtual com" in desc:
            return port.device
    return None


def recv_lines(ser, timeout_s):
    """
    Read lines from *ser* until *timeout_s* seconds pass with no new data.
    Returns a list of decoded lines (stripped).
    """
    lines = []
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if ser.in_waiting > 0:
            raw = ser.readline()
            line = raw.decode("utf-8", errors="ignore").strip()
            if line:
                lines.append(line)
                deadline = time.time() + timeout_s  # reset on activity
        else:
            time.sleep(0.01)
    return lines


def main():
    # Port selection: command-line arg > auto-detect > fallback
    if len(sys.argv) > 1:
        PORT = sys.argv[1]
    else:
        PORT = auto_detect_port()
        if PORT:
            print(f"Auto-detected port: {PORT}")
        else:
            PORT = "COM3"
            print(f"No ST-Link port found; defaulting to {PORT}")

    BAUD_RATE = 115200

    print("STM32 Nucleo F446RE - IMU UART Test")
    print("=" * 50)
    print(f"Port:      {PORT}")
    print(f"Baud Rate: {BAUD_RATE}")
    print(f"Commands:  {', '.join(COMMANDS)}")
    print("Type 'quit' to exit")
    print("=" * 50)

    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=2)
        print(f"Connected to {PORT}\n")
        time.sleep(2)  # Wait for board reset after connection

        print("Enter commands (Ctrl+C to exit):")
        print("-" * 50)

        while True:
            try:
                message = input("\nCommand: ").strip().upper()
            except EOFError:
                break

            if not message:
                continue

            if message == "QUIT":
                break

            if message not in COMMANDS:
                print(f"Unknown command '{message}'")
                print(f"Valid commands: {', '.join(COMMANDS)}")
                continue

            ser.write((message + "\n").encode("utf-8"))
            print(f"Sent: {message}")

            if message == "READALL":
                # READALL sends a header line then up to 100 data lines.
                # Read until the stream goes quiet for 0.5 s after the last line.
                lines = recv_lines(ser, timeout_s=0.5)
                if not lines:
                    print("No response received")
                else:
                    for line in lines:
                        print(f"  {line}")
                    print(f"  ({len(lines)} line(s) received)")
            else:
                time.sleep(0.15)
                if ser.in_waiting > 0:
                    response = ser.read(ser.in_waiting).decode("utf-8", errors="ignore")
                    print(f"Response: {response.strip()}")
                else:
                    print("No response received")

    except serial.SerialException as e:
        print(f"\nError: {e}")
        print("\nTroubleshooting:")
        print("  1. Check that the correct COM port is specified")
        print("  2. Ensure the microcontroller is connected via USB")
        print("  3. Verify no other program is using the port")
        print("  4. On Windows, check Device Manager for the COM port number")
        sys.exit(1)

    except KeyboardInterrupt:
        print("\n\nExiting...")

    finally:
        if "ser" in locals() and ser.is_open:
            ser.close()
            print("Serial port closed")


if __name__ == "__main__":
    main()
