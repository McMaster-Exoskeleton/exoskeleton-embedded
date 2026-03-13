#!/usr/bin/env python3
"""
UART Test Script for STM32 Character Counter
Sends messages to the microcontroller and displays the character count response
"""

import serial
import time
import sys

def main():
    # Configuration
    PORT = 'COM4'  # Change this to match your COM port
    BAUD_RATE = 115200
    TIMEOUT = 2

    print("STM32 UART Character Counter Test")
    print("=" * 50)
    print(f"Port: {PORT}")
    print(f"Baud Rate: {BAUD_RATE}")
    print("=" * 50)

    try:
        # Open serial connection
        ser = serial.Serial(PORT, BAUD_RATE, timeout=TIMEOUT)
        print(f"Connected to {PORT}")
        time.sleep(2)  # Give time for connection to stabilize

        # Interactive mode
        print("\nEnter messages to send (Ctrl+C to exit):")
        print("-" * 50)

        while True:
            # Get user input
            message = input("\nYour message: ")

            if not message:
                continue

            # Send message with newline
            ser.write((message + '\n').encode('utf-8'))
            print(f"Sent: '{message}' ({len(message)} characters)")

            # Wait for response
            time.sleep(0.1)

            # Read response
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
        print("5. On Linux, you may need: sudo usermod -a -G dialout $USER")
        sys.exit(1)

    except KeyboardInterrupt:
        print("\n\nExiting...")

    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed")

if __name__ == "__main__":
    main()
