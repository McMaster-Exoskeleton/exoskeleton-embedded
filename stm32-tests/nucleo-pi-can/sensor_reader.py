#!/usr/bin/env python3
"""
Sensor Reader — Raspberry Pi UART client for the Nucleo-F446RE sensor board.

Retrieves timestamped IMU (accel + gyro) and motor encoder (position) data
from the STM32 over the ST-Link virtual COM port.

Commands:
  READ       - Single live reading (timestamp, motor pos, accel, gyro)
  READLATEST - Latest entry from the 100-sample circular buffer
  READALL    - All stored readings, oldest first (up to 100 lines)
  STATUS     - IMU connection state + motor position + motor error
  STREAM     - Continuously send READ every STREAM_INTERVAL_S (local command)
  STREAMALL  - Continuously send READALL every STREAMALL_INTERVAL_S (local)
  REGISTER   - I2C device address
  CONFIG     - Gyro and accel control register values
  POWER      - Power configuration register value

Usage:
  python3 sensor_reader.py                   # auto-detect port
  python3 sensor_reader.py /dev/ttyACM0      # specify port
  python3 sensor_reader.py /dev/ttyACM0 -o data.csv   # log READALL to CSV
"""

import serial
import serial.tools.list_ports
import time
import sys
import re
import csv
import argparse
import signal

BAUD_RATE = 115200

MCU_COMMANDS = [
    "READ", "READLATEST", "READALL",
    "STATUS", "REGISTER", "CONFIG", "POWER",
]

LOCAL_COMMANDS = ["STREAM", "STREAMALL", "QUIT", "HELP"]

ALL_COMMANDS = MCU_COMMANDS + LOCAL_COMMANDS

STREAM_INTERVAL_S = 0.05   # 50 ms  — matches the 20 ms loop on the MCU
STREAMALL_INTERVAL_S = 2.5  # seconds between READALL bursts

# Field names in the order they appear in the MCU response
SENSOR_FIELDS = ["T", "MP", "AX", "AY", "AZ", "GX", "GY", "GZ"]

# Regex to pull key:value pairs from a response line
KV_RE = re.compile(r"([A-Z]+):([-\d.]+)")


# ── helpers ──────────────────────────────────────────────────────────────────

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


def recv_lines(ser, quiet_timeout_s=0.5):
    """Read lines until *quiet_timeout_s* passes with no new data."""
    lines = []
    deadline = time.time() + quiet_timeout_s
    while time.time() < deadline:
        if ser.in_waiting > 0:
            raw = ser.readline()
            line = raw.decode("utf-8", errors="ignore").strip()
            if line:
                lines.append(line)
                deadline = time.time() + quiet_timeout_s
        else:
            time.sleep(0.005)
    return lines


def send_cmd(ser, cmd):
    """Flush RX, send command, return None."""
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode("utf-8"))


def send_and_recv(ser, cmd):
    """Send a single-line MCU command and return the response string."""
    send_cmd(ser, cmd)
    time.sleep(0.15)
    if ser.in_waiting > 0:
        return ser.read(ser.in_waiting).decode("utf-8", errors="ignore").strip()
    return None


def send_and_recv_multi(ser, cmd, quiet_s=0.5):
    """Send a multi-line MCU command (READALL) and return list of lines."""
    send_cmd(ser, cmd)
    return recv_lines(ser, quiet_timeout_s=quiet_s)


def parse_sensor_line(line):
    """
    Parse a response line like
      T:12345 MP:10.50 AX:0.12 AY:-0.05 AZ:9.78 GX:0.03 GY:-0.01 GZ:0.02
    into a dict {T: '12345', MP: '10.50', ...}.
    Returns None if the line doesn't contain sensor data.
    """
    pairs = KV_RE.findall(line)
    if not pairs:
        return None
    return {k: v for k, v in pairs}


def print_sensor(parsed, prefix=""):
    """Pretty-print a parsed sensor dict."""
    if parsed is None:
        print(f"{prefix}(no data)")
        return
    t   = parsed.get("T", "?")
    mp  = parsed.get("MP", "?")
    ax  = parsed.get("AX", "?")
    ay  = parsed.get("AY", "?")
    az  = parsed.get("AZ", "?")
    gx  = parsed.get("GX", "?")
    gy  = parsed.get("GY", "?")
    gz  = parsed.get("GZ", "?")
    print(f"{prefix}t={t}ms  motor={mp}deg  "
          f"accel=({ax}, {ay}, {az}) m/s^2  "
          f"gyro=({gx}, {gy}, {gz}) dps")


# ── CSV logging ──────────────────────────────────────────────────────────────

class CsvLogger:
    """Optional CSV writer.  Pass None for *path* to disable."""

    def __init__(self, path):
        self._f = None
        self._w = None
        if path:
            self._f = open(path, "w", newline="")
            self._w = csv.writer(self._f)
            self._w.writerow(["timestamp_ms", "motor_pos_deg",
                              "ax", "ay", "az", "gx", "gy", "gz"])

    def write(self, parsed):
        if self._w is None or parsed is None:
            return
        row = [parsed.get(f, "") for f in SENSOR_FIELDS]
        self._w.writerow(row)

    def close(self):
        if self._f:
            self._f.close()


# ── interactive loop ─────────────────────────────────────────────────────────

def run_stream(ser, logger, interval_s):
    """Continuously send READ and display results until Ctrl+C."""
    print("Streaming (Ctrl+C to stop)...\n")
    try:
        while True:
            resp = send_and_recv(ser, "READ")
            if resp:
                parsed = parse_sensor_line(resp)
                print_sensor(parsed)
                logger.write(parsed)
            else:
                print("(no response)")
            time.sleep(interval_s)
    except KeyboardInterrupt:
        print("\nStream stopped.")


def run_stream_all(ser, logger, interval_s):
    """Continuously send READALL, print + log all lines, repeat."""
    print("Streaming READALL (Ctrl+C to stop)...\n")
    try:
        while True:
            lines = send_and_recv_multi(ser, "READALL", quiet_s=0.5)
            if not lines:
                print("(no response)")
            else:
                for line in lines:
                    parsed = parse_sensor_line(line)
                    if parsed:
                        print_sensor(parsed, prefix="  ")
                        logger.write(parsed)
                    else:
                        print(f"  {line}")
                print(f"  ({len(lines)} line(s))\n")
            time.sleep(interval_s)
    except KeyboardInterrupt:
        print("\nStream stopped.")


def print_help():
    print("\nAvailable commands:")
    print("  MCU commands  : " + ", ".join(MCU_COMMANDS))
    print("  Local commands:")
    print("    STREAM    - Continuously poll READ (~50 ms)")
    print("    STREAMALL - Continuously poll READALL (~2.5 s)")
    print("    QUIT      - Exit the program")
    print("    HELP      - Show this message")
    print()


def main():
    # ── argument parsing ─────────────────────────────────────────────────
    ap = argparse.ArgumentParser(
        description="Read sensor + motor data from Nucleo-F446RE over UART")
    ap.add_argument("port", nargs="?", default=None,
                    help="Serial port (e.g. /dev/ttyACM0). Auto-detected if omitted.")
    ap.add_argument("-o", "--output", default=None,
                    help="Path to CSV file for logging sensor data")
    args = ap.parse_args()

    port = args.port
    if port is None:
        port = auto_detect_port()
        if port:
            print(f"Auto-detected port: {port}")
        else:
            port = "/dev/ttyACM0"
            print(f"No ST-Link port found; defaulting to {port}")

    logger = CsvLogger(args.output)

    print("Nucleo-F446RE Sensor Reader (Raspberry Pi)")
    print("=" * 55)
    print(f"Port:      {port}")
    print(f"Baud Rate: {BAUD_RATE}")
    print(f"Commands:  {', '.join(ALL_COMMANDS)}")
    if args.output:
        print(f"CSV log:   {args.output}")
    print("Type HELP for details, QUIT or Ctrl+C to exit")
    print("=" * 55)

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=2)
        print(f"Connected to {port}\n")
        time.sleep(2)  # wait for board reset after serial open

        print("Enter commands:")
        print("-" * 55)

        while True:
            try:
                message = input("\nCommand: ").strip().upper()
            except EOFError:
                break

            if not message:
                continue

            if message == "QUIT":
                break

            if message == "HELP":
                print_help()
                continue

            if message == "STREAM":
                run_stream(ser, logger, STREAM_INTERVAL_S)
                continue

            if message == "STREAMALL":
                run_stream_all(ser, logger, STREAMALL_INTERVAL_S)
                continue

            if message not in MCU_COMMANDS:
                print(f"Unknown command '{message}'")
                print(f"Valid: {', '.join(ALL_COMMANDS)}")
                continue

            # ── MCU command dispatch ─────────────────────────────────────
            if message == "READALL":
                lines = send_and_recv_multi(ser, "READALL", quiet_s=0.5)
                if not lines:
                    print("No response received")
                else:
                    for line in lines:
                        parsed = parse_sensor_line(line)
                        if parsed:
                            print_sensor(parsed, prefix="  ")
                            logger.write(parsed)
                        else:
                            print(f"  {line}")
                    print(f"  ({len(lines)} line(s) received)")
            else:
                resp = send_and_recv(ser, message)
                if resp:
                    # For READ / READLATEST, pretty-print if sensor data
                    parsed = parse_sensor_line(resp)
                    if parsed:
                        print_sensor(parsed)
                        logger.write(parsed)
                    else:
                        print(f"Response: {resp}")
                else:
                    print("No response received")

    except serial.SerialException as e:
        print(f"\nError: {e}")
        print("\nTroubleshooting:")
        print("  1. Check that the correct serial port is specified")
        print("  2. Ensure the Nucleo is connected via USB")
        print("  3. Verify no other program is using the port")
        print("  4. Run: ls /dev/ttyACM*  to find the port")
        sys.exit(1)

    except KeyboardInterrupt:
        print("\n\nExiting...")

    finally:
        logger.close()
        if "ser" in locals() and ser.is_open:
            ser.close()
            print("Serial port closed")


if __name__ == "__main__":
    main()
