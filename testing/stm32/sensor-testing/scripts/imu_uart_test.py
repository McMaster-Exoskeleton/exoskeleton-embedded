#!/usr/bin/env python3
"""
UART IMU Test Script for STM32 Nucleo F446RE + LSM6DS3TR-C
Sends commands to the microcontroller and displays IMU data responses.

Usage:
    python imu_uart_test.py              # Auto-detect COM port
    python imu_uart_test.py COM5         # Specify COM port (Windows)
    python imu_uart_test.py /dev/ttyACM0 # Specify port (Linux)

Requirements:
    pip install pyserial

Commands:
    READ     - Get filtered accelerometer and gyroscope measurements
    STATUS   - Get IMU connectivity status
    REGISTER - Get the I2C device address
    CONFIG   - Get gyro and accel configuration register values
    POWER    - Get power configuration register value
"""

import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial is required. Install with: pip install pyserial")
    sys.exit(1)

COMMANDS = ["READ", "STATUS", "REGISTER", "CONFIG", "POWER"]
BAUD_RATE = 115200
TIMEOUT = 2


def find_stlink_port():
    """Try to auto-detect an ST-Link virtual COM port."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        desc = (port.description or "").lower()
        mfr = (port.manufacturer or "").lower()
        if "stlink" in desc or "stm" in desc or "stlink" in mfr or "stm" in mfr:
            return port.device
    # Fallback: return the first available port
    if ports:
        return ports[0].device
    return None


def main():
    # Determine COM port
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = find_stlink_port()
        if port is None:
            print("ERROR: No serial ports found. Connect the Nucleo board and retry.")
            print("       Or specify the port: python imu_uart_test.py COM5")
            sys.exit(1)
        print(f"Auto-detected port: {port}")

    print("STM32 Nucleo F446RE - IMU UART Test")
    print("=" * 50)
    print(f"Port: {port}")
    print(f"Baud Rate: {BAUD_RATE}")
    print(f"Available commands: {', '.join(COMMANDS)}")
    print("Type 'quit' to exit")
    print("=" * 50)

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=TIMEOUT)
        print(f"Connected to {port}")
        time.sleep(2)

        print("\nEnter commands:")
        print("-" * 50)

        while True:
            message = input("\nCommand: ").strip()

            if message.lower() == "quit":
                break

            message = message.upper()

            if not message:
                continue

            if message not in COMMANDS:
                print(f"Unknown command '{message}'")
                print(f"Valid commands: {', '.join(COMMANDS)}")
                continue

            ser.write((message + '\n').encode('utf-8'))
            print(f"Sent: {message}")

            time.sleep(0.1)

            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                print(f"Response: {response.strip()}")
            else:
                print("No response received")

    except serial.SerialException as e:
        print(f"\nError: {e}")
        print("\nTroubleshooting:")
        print("1. Check that the correct COM port is specified")
        print("2. Ensure the Nucleo board is connected via USB")
        print("3. Verify no other program is using the port (close CubeIDE serial monitor)")
        print("4. On Windows, check Device Manager for the COM port number")
        sys.exit(1)

    except KeyboardInterrupt:
        print("\n\nExiting...")

    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed")


if __name__ == "__main__":
    main()
