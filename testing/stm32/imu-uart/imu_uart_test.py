#!/usr/bin/env python3
"""
UART IMU Test Script for STM32 Nucleo F446RE + MPU9250
Sends commands to the microcontroller and displays IMU data responses

Commands:
  READ     - Get accelerometer and gyroscope measurements
  STATUS   - Get IMU connectivity status
  REGISTER - Get the I2C device address
  CONFIG   - Get gyro and accel configuration values
  POWER    - Get power configuration value
"""

import serial
import time
import sys

COMMANDS = ["READ", "STATUS", "REGISTER", "CONFIG", "POWER"]

def main():
    PORT = 'COM3'  # Change this to match your COM port
    BAUD_RATE = 115200
    TIMEOUT = 2

    print("STM32 Nucleo F446RE - IMU UART Test")
    print("=" * 50)
    print(f"Port: {PORT}")
    print(f"Baud Rate: {BAUD_RATE}")
    print(f"Available commands: {', '.join(COMMANDS)}")
    print("=" * 50)

    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=TIMEOUT)
        print(f"Connected to {PORT}")
        time.sleep(2)

        print("\nEnter commands (Ctrl+C to exit):")
        print("-" * 50)

        while True:
            message = input("\nCommand: ").strip().upper()

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
        print("2. Ensure the microcontroller is connected")
        print("3. Verify no other program is using the port")
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
