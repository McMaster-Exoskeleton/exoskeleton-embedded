"""
motor_test.py

Interactive serial console for testing the AK70-9 motor API over UART.
Connects to the STM32 Nucleo board via the ST-Link virtual COM port
and sends text-based commands.

Usage:
    python motor_test.py              # Auto-detect COM port
    python motor_test.py COM5         # Specify COM port
    python motor_test.py /dev/ttyACM0 # Linux

Requirements:
    pip install pyserial

Commands:
    PING       - Test connection (expects PONG)
    READ_ALL   - Read all motor feedback values
    READ_POS   - Read motor position (degrees)
    READ_SPD   - Read motor speed (ERPM)
    READ_CUR   - Read motor current (amps)
    READ_TEMP  - Read driver board temperature (C)
    READ_ERR   - Read motor error code with description
    quit       - Exit the script
"""

import sys
import threading
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial is required. Install with: pip install pyserial")
    sys.exit(1)

BAUD_RATE = 115200
TIMEOUT = 1.0  # Serial read timeout in seconds


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


def reader_thread(ser, stop_event):
    """
    Background thread that continuously reads lines from the serial port.
    Displays motor error notifications (lines starting with '!') prominently.
    """
    while not stop_event.is_set():
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode("ascii", errors="replace").strip()
                if not line:
                    continue
                if line.startswith("!ERR:"):
                    # Proactive motor error notification
                    parts = line[5:].split(":", 1)
                    code = parts[0]
                    desc = parts[1] if len(parts) > 1 else "UNKNOWN"
                    print(f"\n*** MOTOR ERROR [{code}]: {desc} ***")
                    print("> ", end="", flush=True)
                else:
                    print(f"  <- {line}")
            else:
                time.sleep(0.01)
        except (serial.SerialException, OSError):
            if not stop_event.is_set():
                print("\n[Serial connection lost]")
            break


def main():
    # Determine COM port
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = find_stlink_port()
        if port is None:
            print("ERROR: No serial ports found. Connect the Nucleo board and retry.")
            print("       Or specify the port: python motor_test.py COM5")
            sys.exit(1)
        print(f"Auto-detected port: {port}")

    # Open serial connection
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=TIMEOUT)
    except serial.SerialException as e:
        print(f"ERROR: Could not open {port}: {e}")
        sys.exit(1)

    print(f"Connected to {port} at {BAUD_RATE} baud")
    print()
    print("Available commands:")
    print("  PING       - Test connection")
    print("  READ_ALL   - Read all motor values")
    print("  READ_POS   - Read position (degrees)")
    print("  READ_SPD   - Read speed (ERPM)")
    print("  READ_CUR   - Read current (amps)")
    print("  READ_TEMP  - Read temperature (C)")
    print("  READ_ERR   - Read error code")
    print("  quit       - Exit")
    print()

    # Start the reader thread
    stop_event = threading.Event()
    reader = threading.Thread(target=reader_thread, args=(ser, stop_event), daemon=True)
    reader.start()

    # Interactive command loop
    try:
        while True:
            try:
                cmd = input("> ").strip()
            except EOFError:
                break

            if cmd.lower() == "quit":
                break
            if not cmd:
                continue

            # Send command to the STM32
            try:
                ser.write((cmd + "\n").encode("ascii"))
            except serial.SerialException:
                print("[Serial write failed]")
                break

            # Brief pause to let the response arrive
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nInterrupted")

    # Cleanup
    stop_event.set()
    reader.join(timeout=1.0)
    ser.close()
    print("Disconnected.")


if __name__ == "__main__":
    main()
