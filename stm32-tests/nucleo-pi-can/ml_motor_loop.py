#!/usr/bin/env python3
"""
ML Motor Control Loop — Raspberry Pi

Reads IMU + motor position from the Nucleo over UART, sends a sliding window
to the TCN inference server, and commands motor current based on the predicted
torque.

Flow:
  1. Connect to Nucleo via UART
  2. SET_ORIGIN to zero the motor position
  3. Continuously READ sensor data (AX,AY,AZ,GX,GY,GZ,MP)
  4. Maintain a sliding window of 187 timesteps × 7 features
  5. POST the window to the inference server (msgpack)
  6. Convert predicted torque → current (current = torque / TORQUE_CONSTANT)
  7. Send SET_CUR:<current> to the Nucleo

Usage:
  python3 ml_motor_loop.py                          # auto-detect port
  python3 ml_motor_loop.py /dev/ttyACM0             # specify port
  python3 ml_motor_loop.py --server http://HOST:PORT # custom server
"""

import serial
import serial.tools.list_ports
import time
import sys
import re
import argparse
import collections
import struct

try:
    import msgpack
except ImportError:
    print("ERROR: msgpack is required. Install with: pip install msgpack")
    sys.exit(1)

try:
    import requests
except ImportError:
    print("ERROR: requests is required. Install with: pip install requests")
    sys.exit(1)

# ── Constants ────────────────────────────────────────────────────────────────

BAUD_RATE = 115200
TORQUE_CONSTANT = 0.159         # Nm/A for AK70-9
WINDOW_SIZE = 187               # TCN model window
FEATURE_COUNT = 7               # AX, AY, AZ, GX, GY, GZ, MP
CURRENT_CLAMP = 5.0             # max current magnitude (A)
LOOP_INTERVAL_S = 0.02          # 50 Hz control loop (matches 20ms MCU loop)

PREDICT_URL_TEMPLATE = "{}/predict_msgpack?model=single_joint"
HEALTH_URL_TEMPLATE = "{}/health"

# Regex to pull key:value pairs from a sensor response line
KV_RE = re.compile(r"([A-Z]+):([-\d.]+)")


# ── Helpers ──────────────────────────────────────────────────────────────────

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
    """Send a UART command and return the response string."""
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode("utf-8"))
    time.sleep(0.1)
    if ser.in_waiting > 0:
        return ser.read(ser.in_waiting).decode("utf-8", errors="ignore").strip()
    return None


def parse_sensor_line(line):
    """Parse 'T:123 MP:1.5 AX:0.1 ...' into a dict."""
    pairs = KV_RE.findall(line)
    if not pairs:
        return None
    return {k: float(v) for k, v in pairs}


def reading_to_features(parsed):
    """Extract the 7 model features in order: AX,AY,AZ,GX,GY,GZ,MP."""
    return [
        parsed.get("AX", 0.0),
        parsed.get("AY", 0.0),
        parsed.get("AZ", 0.0),
        parsed.get("GX", 0.0),
        parsed.get("GY", 0.0),
        parsed.get("GZ", 0.0),
        parsed.get("MP", 0.0),
    ]


def build_msgpack_request(window):
    """Pack the sliding window as a flat float32 array for the server."""
    flat = []
    for row in window:
        flat.extend(row)
    return msgpack.packb(flat, use_single_float=True)


def torque_to_current(torque_nm):
    """Convert torque (Nm) to current (A) and clamp."""
    current = torque_nm / TORQUE_CONSTANT
    if current > CURRENT_CLAMP:
        current = CURRENT_CLAMP
    elif current < -CURRENT_CLAMP:
        current = -CURRENT_CLAMP
    return current


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description="ML motor control loop: sensor → inference → motor current")
    ap.add_argument("port", nargs="?", default=None,
                    help="Serial port (e.g. /dev/ttyACM0). Auto-detected if omitted.")
    ap.add_argument("--server", default="http://127.0.0.1:8000",
                    help="Inference server base URL (default: http://127.0.0.1:8000)")
    args = ap.parse_args()

    port = args.port or auto_detect_port()
    if port is None:
        port = "/dev/ttyACM0"
        print(f"No ST-Link port found; defaulting to {port}")
    else:
        print(f"Using port: {port}")

    predict_url = PREDICT_URL_TEMPLATE.format(args.server)
    health_url = HEALTH_URL_TEMPLATE.format(args.server)

    # ── Check inference server health ────────────────────────────────────
    print(f"Checking inference server at {args.server} ...")
    try:
        resp = requests.get(health_url, timeout=5)
        if resp.status_code == 200:
            print("Inference server: OK")
        else:
            print(f"WARNING: Server returned HTTP {resp.status_code}")
    except requests.exceptions.ConnectionError:
        print(f"ERROR: Cannot reach inference server at {args.server}")
        print("Make sure the server is running:")
        print("  uvicorn scripts.server:app --host 127.0.0.1 --port 8000")
        sys.exit(1)

    # ── Connect to Nucleo ────────────────────────────────────────────────
    print(f"\nConnecting to Nucleo on {port} @ {BAUD_RATE} baud ...")
    ser = serial.Serial(port, BAUD_RATE, timeout=2)
    print(f"Connected to {port}")
    time.sleep(2)  # wait for board reset

    # ── Set motor origin to 0 ───────────────────────────────────────────
    # Use the SET_ORIGIN command from the motor API (mode 1 = set current position as origin)
    # We send this as a raw UART command that the firmware handles
    print("\nSetting motor origin to 0 ...")
    resp = send_and_recv(ser, "SET_ORIGIN")
    print(f"  SET_ORIGIN -> {resp}")

    # Ensure motor starts at zero current
    resp = send_and_recv(ser, "SET_CUR:0.00")
    print(f"  SET_CUR:0.00 -> {resp}")

    # ── Initialize sliding window (zero-padded) ─────────────────────────
    zero_row = [0.0] * FEATURE_COUNT
    window = collections.deque([zero_row[:] for _ in range(WINDOW_SIZE)], maxlen=WINDOW_SIZE)

    print(f"\nML Motor Control Loop")
    print("=" * 60)
    print(f"  Window size:      {WINDOW_SIZE} timesteps × {FEATURE_COUNT} features")
    print(f"  Torque constant:  {TORQUE_CONSTANT} Nm/A")
    print(f"  Current clamp:    ±{CURRENT_CLAMP} A")
    print(f"  Loop interval:    {LOOP_INTERVAL_S * 1000:.0f} ms")
    print(f"  Inference server: {predict_url}")
    print("=" * 60)
    print("Press Ctrl+C to stop (motor will be set to 0A)\n")

    cycle_count = 0
    inference_errors = 0

    try:
        while True:
            loop_start = time.time()

            # 1. Read latest sensor data from Nucleo
            resp = send_and_recv(ser, "READ")
            if resp:
                parsed = parse_sensor_line(resp)
                if parsed:
                    features = reading_to_features(parsed)
                    window.append(features)
                else:
                    # Non-sensor response, keep window as-is
                    pass
            # If no response, keep the previous window state

            # 2. Build msgpack request and call inference server
            payload = build_msgpack_request(window)
            torque = 0.0
            inference_ms = 0.0
            try:
                r = requests.post(
                    predict_url,
                    data=payload,
                    headers={"Content-Type": "application/x-msgpack"},
                    timeout=1.0,
                )
                if r.status_code == 200:
                    result = msgpack.unpackb(r.content, raw=False)
                    torque = result.get("hip_left", 0.0) * 0.2
                    inference_ms = result.get("inference_ms", 0.0)
                else:
                    inference_errors += 1
            except (requests.exceptions.RequestException, msgpack.exceptions.UnpackException):
                inference_errors += 1
                # On inference failure, use zero torque (safe fallback)

            # 3. Convert torque → current
            current = torque_to_current(torque)

            # 4. Send current command to motor
            cmd = f"SET_CUR:{current:.3f}"
            motor_resp = send_and_recv(ser, cmd)

            # 5. Print status
            cycle_count += 1
            mp = parsed.get("MP", 0.0) if resp and parsed else 0.0
            print(
                f"[{cycle_count:5d}]  "
                f"MP={mp:7.2f}deg  "
                f"torque={torque:7.4f}Nm  "
                f"current={current:7.3f}A  "
                f"infer={inference_ms:5.1f}ms  "
                f"errors={inference_errors}"
            )

            # 6. Maintain loop timing
            elapsed = time.time() - loop_start
            sleep_time = LOOP_INTERVAL_S - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")

    finally:
        print("Setting motor current to 0A ...")
        send_and_recv(ser, "SET_CUR:0.00")
        time.sleep(0.1)
        ser.close()
        print(f"Motor stopped. Serial port closed. ({cycle_count} cycles, {inference_errors} inference errors)")


if __name__ == "__main__":
    main()
