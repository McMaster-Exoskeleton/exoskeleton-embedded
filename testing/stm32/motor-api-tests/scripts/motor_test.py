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

Read Commands:
    PING       - Test connection (expects PONG)
    READ_ALL   - Read all motor feedback values
    READ_POS   - Read motor position (degrees)
    READ_SPD   - Read motor speed (ERPM)
    READ_CUR   - Read motor current (amps)
    READ_TEMP  - Read driver board temperature (C)
    READ_ERR   - Read motor error code with description

Motor Control Commands (require confirmation):
    ESTOP                          - Emergency stop (immediate, no confirmation)
    SET_DUTY <duty>                - Set duty cycle (-0.15 to 0.15)
    SET_CURRENT <amps>             - Set current/torque (-5.0 to 5.0 A)
    SET_BRAKE <amps>               - Set brake current (0 to 5.0 A)
    SET_RPM <rpm>                  - Set velocity (-5000 to 5000 ERPM)
    SET_POS <degrees>              - Set position (-360 to 360 deg)
    SET_ORIGIN <0|1>               - Set zero position (0=temp, 1=permanent)
    SET_POS_SPD <deg> <spd> <acc>  - Position with velocity/accel limits
    SET_MIT <p> <v> <kp> <kd> <t>  - MIT force control

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

# Commands that move the motor and require confirmation
MOTOR_COMMANDS = {
    "SET_DUTY", "SET_CURRENT", "SET_BRAKE", "SET_RPM",
    "SET_POS", "SET_ORIGIN", "SET_POS_SPD", "SET_MIT",
}


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


def is_motor_command(cmd):
    """Check if a command is a motor control command that needs confirmation."""
    cmd_upper = cmd.strip().upper()
    for prefix in MOTOR_COMMANDS:
        if cmd_upper.startswith(prefix):
            return True
    return False


def reader_thread(ser, stop_event):
    """
    Background thread that continuously reads lines from the serial port.
    Displays motor error notifications and watchdog alerts prominently.
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
    print("=== READ COMMANDS ===")
    print("  PING       - Test connection")
    print("  READ_ALL   - Read all motor values")
    print("  READ_POS   - Read position (degrees)")
    print("  READ_SPD   - Read speed (ERPM)")
    print("  READ_CUR   - Read current (amps)")
    print("  READ_TEMP  - Read temperature (C)")
    print("  READ_ERR   - Read error code")
    print()
    print("=== MOTOR CONTROL (requires confirmation) ===")
    print("  ESTOP                          - EMERGENCY STOP (immediate)")
    print("  SET_DUTY <duty>                - Duty cycle (-0.15 to 0.15)")
    print("  SET_CURRENT <amps>             - Current (-5 to 5 A)")
    print("  SET_BRAKE <amps>               - Brake current (0 to 5 A)")
    print("  SET_RPM <rpm>                  - Velocity (-5000 to 5000 ERPM)")
    print("  SET_POS <degrees>              - Position (-360 to 360 deg)")
    print("  SET_ORIGIN <0|1>               - Set zero (0=temp, 1=perm)")
    print("  SET_POS_SPD <deg> <spd> <acc>  - Position + velocity + accel")
    print("  SET_MIT <p> <v> <kp> <kd> <t>  - MIT force control")
    print()
    print("  quit / Ctrl+C - Exit")
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
                # Send ESTOP before quitting
                try:
                    ser.write(b"ESTOP\n")
                    time.sleep(0.1)
                except serial.SerialException:
                    pass
                break
            if not cmd:
                continue

            # ESTOP is always sent immediately without confirmation
            if cmd.strip().upper() == "ESTOP":
                try:
                    ser.write(b"ESTOP\n")
                except serial.SerialException:
                    print("[Serial write failed]")
                    break
                time.sleep(0.05)
                continue

            # Motor control commands require confirmation
            if is_motor_command(cmd):
                print(f"  >> Will send: {cmd}")
                try:
                    confirm = input("  >> Confirm? (y/n): ").strip().lower()
                except EOFError:
                    break
                if confirm != "y":
                    print("  >> Cancelled.")
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
        print("\nInterrupted - sending ESTOP...")
        try:
            ser.write(b"ESTOP\n")
            time.sleep(0.1)
        except serial.SerialException:
            pass

    # Cleanup
    stop_event.set()
    reader.join(timeout=1.0)
    ser.close()
    print("Disconnected.")


if __name__ == "__main__":
    main()
