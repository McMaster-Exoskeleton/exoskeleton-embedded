#!/usr/bin/env python3
"""
exo_system_test.py

Single-file Raspberry Pi / SocketCAN test tool for the exoskeleton network.

What it does:
- Receives and decodes IMU accel frames, IMU gyro frames, and motor status frames
- Lets you send torque commands to one node or all nodes
- Supports quick pulse and sweep tests
- Sends zero torque to all joints on exit by default

Assumptions from current firmware/protocol:
- SocketCAN interface defaults to can1
- Standard 11-bit CAN IDs using the project's bit-packed format:
    bits 10:7 -> msg_type
    bits  6:4 -> src_node
    bits  3:0 -> dest_node
- IMU accel payload: <hhhH   -> ax, ay, az, timestamp_ms
- IMU gyro payload:  <hhhH   -> gx, gy, gz, timestamp_ms
- Motor status payload: <hhhbB -> position*10 deg, speed/10 ERPM, current*100 A, temp C, error
- Torque command payload: <h -> torque_nm * 1000
"""

from __future__ import annotations

import argparse
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Optional

import can

# ---------------- CAN protocol constants ----------------
NODE_PI = 0
NODE_LEFT_HIP = 1
NODE_RIGHT_HIP = 2
NODE_LEFT_KNEE = 3
NODE_RIGHT_KNEE = 4

NODES = {
    NODE_LEFT_HIP: "Left Hip",
    NODE_RIGHT_HIP: "Right Hip",
    NODE_LEFT_KNEE: "Left Knee",
    NODE_RIGHT_KNEE: "Right Knee",
}

MSG_ESTOP = 0x0
MSG_TORQUE_CMD = 0x1
MSG_MOTOR_STATUS = 0x2
MSG_IMU_ACCEL = 0x3
MSG_IMU_GYRO = 0x4
MSG_HEARTBEAT = 0x5

KT_EFFECTIVE = 1.431  # Nm/A from your STM32 comments
DEFAULT_SAFE_TORQUE_LIMIT = 7.0  # conservative bench limit


def build_can_id(msg_type: int, src_node: int, dest_node: int) -> int:
    return ((msg_type & 0x0F) << 7) | ((src_node & 0x07) << 4) | (dest_node & 0x0F)


def parse_can_id(can_id: int) -> tuple[int, int, int]:
    msg_type = (can_id >> 7) & 0x0F
    src_node = (can_id >> 4) & 0x07
    dest_node = can_id & 0x0F
    return msg_type, src_node, dest_node


# ---------------- decoding helpers ----------------

def parse_imu_frame(data: bytes) -> tuple[float, float, float, int]:
    raw_x, raw_y, raw_z, ts = struct.unpack("<hhhH", bytes(data[:8]))
    return raw_x / 100.0, raw_y / 100.0, raw_z / 100.0, ts


def parse_motor_status(data: bytes) -> tuple[float, float, float, int, int]:
    raw_pos, raw_spd, raw_cur, temp, err = struct.unpack("<hhhbB", bytes(data[:8]))
    return raw_pos / 10.0, raw_spd * 10.0, raw_cur / 100.0, temp, err


# ---------------- RX state ----------------
@dataclass
class NodeTelemetry:
    last_accel: Optional[tuple[float, float, float, int]] = None
    last_gyro: Optional[tuple[float, float, float, int]] = None
    last_motor: Optional[tuple[float, float, float, int, int]] = None
    last_rx_wall_time: float = 0.0
    accel_count: int = 0
    gyro_count: int = 0
    motor_count: int = 0


@dataclass
class RxState:
    nodes: Dict[int, NodeTelemetry] = field(default_factory=lambda: {nid: NodeTelemetry() for nid in NODES})
    lock: threading.Lock = field(default_factory=threading.Lock)

    def update_accel(self, node: int, payload: tuple[float, float, float, int]) -> None:
        with self.lock:
            st = self.nodes.setdefault(node, NodeTelemetry())
            st.last_accel = payload
            st.accel_count += 1
            st.last_rx_wall_time = time.time()

    def update_gyro(self, node: int, payload: tuple[float, float, float, int]) -> None:
        with self.lock:
            st = self.nodes.setdefault(node, NodeTelemetry())
            st.last_gyro = payload
            st.gyro_count += 1
            st.last_rx_wall_time = time.time()

    def update_motor(self, node: int, payload: tuple[float, float, float, int, int]) -> None:
        with self.lock:
            st = self.nodes.setdefault(node, NodeTelemetry())
            st.last_motor = payload
            st.motor_count += 1
            st.last_rx_wall_time = time.time()

    def snapshot(self) -> Dict[int, NodeTelemetry]:
        with self.lock:
            out: Dict[int, NodeTelemetry] = {}
            for nid, st in self.nodes.items():
                out[nid] = NodeTelemetry(
                    last_accel=st.last_accel,
                    last_gyro=st.last_gyro,
                    last_motor=st.last_motor,
                    last_rx_wall_time=st.last_rx_wall_time,
                    accel_count=st.accel_count,
                    gyro_count=st.gyro_count,
                    motor_count=st.motor_count,
                )
            return out


# ---------------- sender helpers ----------------
def send_torque_cmd(bus: can.BusABC, dest_node: int, torque_nm: float, src_node: int = NODE_PI) -> None:
    raw = int(round(torque_nm * 1000.0))
    data = struct.pack("<h", raw)
    can_id = build_can_id(MSG_TORQUE_CMD, src_node, dest_node)
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    bus.send(msg)


def send_estop(bus: can.BusABC, reason: int = 0, src_node: int = NODE_PI) -> None:
    can_id = build_can_id(MSG_ESTOP, src_node, 0)
    msg = can.Message(arbitration_id=can_id, data=struct.pack("<B", reason), is_extended_id=False)
    bus.send(msg)


def zero_all(bus: can.BusABC) -> None:
    for node in NODES:
        send_torque_cmd(bus, node, 0.0)


# ---------------- threads ----------------
def rx_loop(bus: can.BusABC, state: RxState, stop_evt: threading.Event, verbose: bool) -> None:
    while not stop_evt.is_set():
        msg = bus.recv(timeout=0.1)
        if msg is None:
            continue
        if msg.is_error_frame:
            print("[RX] error frame")
            continue
        if msg.is_extended_id:
            if verbose:
                print(f"[RX] EXT id=0x{msg.arbitration_id:X} dlc={msg.dlc} data={msg.data.hex()}")
            continue

        msg_type, src, dest = parse_can_id(msg.arbitration_id)
        try:
            if msg_type == MSG_IMU_ACCEL and msg.dlc >= 8:
                payload = parse_imu_frame(msg.data)
                state.update_accel(src, payload)
                if verbose:
                    ax, ay, az, ts = payload
                    print(f"[ACC] node={src} ax={ax:.2f} ay={ay:.2f} az={az:.2f} ts={ts}")

            elif msg_type == MSG_IMU_GYRO and msg.dlc >= 8:
                payload = parse_imu_frame(msg.data)
                state.update_gyro(src, payload)
                if verbose:
                    gx, gy, gz, ts = payload
                    print(f"[GYR] node={src} gx={gx:.2f} gy={gy:.2f} gz={gz:.2f} ts={ts}")

            elif msg_type == MSG_MOTOR_STATUS and msg.dlc >= 8:
                payload = parse_motor_status(msg.data)
                state.update_motor(src, payload)
                if verbose:
                    pos, spd, cur, temp, err = payload
                    print(f"[MTR] node={src} pos={pos:.1f}deg spd={spd:.0f}erpm cur={cur:.2f}A temp={temp}C err={err}")

            elif msg_type == MSG_ESTOP:
                reason = msg.data[0] if msg.dlc else 0
                print(f"[ESTOP] src={src} reason={reason}")

            elif verbose:
                print(f"[RX] id=0x{msg.arbitration_id:03X} type={msg_type} src={src} dst={dest} dlc={msg.dlc} data={msg.data.hex()}")
        except Exception as exc:
            print(f"[RX] decode error for id=0x{msg.arbitration_id:03X}: {exc}")


def monitor_loop(state: RxState, stop_evt: threading.Event, period_s: float = 1.0) -> None:
    while not stop_evt.wait(period_s):
        snap = state.snapshot()
        print("\n--- Telemetry snapshot ---")
        now = time.time()
        for nid, name in NODES.items():
            st = snap.get(nid)
            if st is None:
                continue
            age = (now - st.last_rx_wall_time) if st.last_rx_wall_time else None
            age_str = f"{age:.2f}s" if age is not None else "never"
            line = f"[{nid}] {name:<10} age={age_str:>7} accel={st.accel_count:4d} gyro={st.gyro_count:4d} motor={st.motor_count:4d}"
            if st.last_accel:
                ax, ay, az, _ = st.last_accel
                line += f" | a=({ax:6.2f},{ay:6.2f},{az:6.2f})"
            if st.last_gyro:
                gx, gy, gz, _ = st.last_gyro
                line += f" | g=({gx:6.2f},{gy:6.2f},{gz:6.2f})"
            if st.last_motor:
                pos, spd, cur, temp, err = st.last_motor
                line += f" | m=(pos {pos:6.1f}, spd {spd:7.0f}, cur {cur:5.2f}, T {temp:2d}, e {err})"
            print(line)
        print("--------------------------\n")


# ---------------- interactive commands ----------------
def print_help() -> None:
    print(
        "Commands:\n"
        "  help                          show this help\n"
        "  status                        print current telemetry snapshot\n"
        "  <node> <torque_nm>            send torque to one node\n"
        "  all <torque_nm>               send same torque to all nodes\n"
        "  pulse <node> <torque> <sec>   apply torque for sec, then zero\n"
        "  sweep <node> <start> <stop> <step> <dt>\n"
        "                                stepped torque sweep, dt in seconds\n"
        "  stop                          send zero torque to all nodes\n"
        "  estop                         send ESTOP broadcast\n"
        "  quit                          zero all and exit\n"
    )


def print_snapshot(state: RxState) -> None:
    snap = state.snapshot()
    now = time.time()
    for nid, name in NODES.items():
        st = snap[nid]
        age = (now - st.last_rx_wall_time) if st.last_rx_wall_time else None
        age_str = f"{age:.2f}s" if age is not None else "never"
        print(f"node={nid} {name}: age={age_str}, accel_count={st.accel_count}, gyro_count={st.gyro_count}, motor_count={st.motor_count}")
        if st.last_accel:
            print(f"  accel={st.last_accel}")
        if st.last_gyro:
            print(f"  gyro ={st.last_gyro}")
        if st.last_motor:
            print(f"  motor={st.last_motor}")


def run_sweep(bus: can.BusABC, node: int, start: float, stop: float, step: float, dt: float, limit: float) -> None:
    if step == 0:
        raise ValueError("step cannot be 0")
    torque = start
    if start <= stop and step < 0:
        step = abs(step)
    if start >= stop and step > 0:
        step = -abs(step)

    def done(x: float) -> bool:
        return x > stop if step > 0 else x < stop

    try:
        while not done(torque):
            bounded = max(-limit, min(limit, torque))
            send_torque_cmd(bus, node, bounded)
            print(f"sent node={node} torque={bounded:.3f} Nm (~{bounded / KT_EFFECTIVE:.3f} A)")
            time.sleep(dt)
            torque += step
    finally:
        send_torque_cmd(bus, node, 0.0)
        print(f"node {node} zeroed")


# ---------------- main ----------------
def main() -> None:
    parser = argparse.ArgumentParser(description="Exoskeleton CAN network test tool")
    parser.add_argument("--channel", default="can1", help="SocketCAN interface, default can1")
    parser.add_argument("--safe-limit", type=float, default=DEFAULT_SAFE_TORQUE_LIMIT, help="Software clamp in Nm for torque commands")
    parser.add_argument("--verbose-rx", action="store_true", help="Print every received frame as it arrives")
    parser.add_argument("--no-auto-zero", action="store_true", help="Do not zero all joints on exit")
    args = parser.parse_args()

    print(f"Opening {args.channel}...")
    bus = can.interface.Bus(channel=args.channel, interface="socketcan")
    state = RxState()
    stop_evt = threading.Event()

    rx_thread = threading.Thread(target=rx_loop, args=(bus, state, stop_evt, args.verbose_rx), daemon=True)
    mon_thread = threading.Thread(target=monitor_loop, args=(state, stop_evt), daemon=True)
    rx_thread.start()
    mon_thread.start()

    print("Exoskeleton CAN test tool")
    print(f"Channel: {args.channel}")
    print(f"Torque safety clamp: +/-{args.safe_limit:.2f} Nm")
    print_help()

    try:
        while True:
            try:
                raw = input("Test> ").strip()
            except EOFError:
                break

            if not raw:
                continue

            parts = raw.split()
            cmd = parts[0].lower()

            if cmd == "help":
                print_help()
            elif cmd == "status":
                print_snapshot(state)
            elif cmd == "stop":
                zero_all(bus)
                print("all nodes commanded to 0 Nm")
            elif cmd == "estop":
                send_estop(bus)
                print("ESTOP broadcast sent")
            elif cmd == "quit":
                break
            elif cmd == "all":
                if len(parts) != 2:
                    print("usage: all <torque_nm>")
                    continue
                torque = max(-args.safe_limit, min(args.safe_limit, float(parts[1])))
                for node, name in NODES.items():
                    send_torque_cmd(bus, node, torque)
                    print(f"sent {torque:.3f} Nm to {name}")
            elif cmd == "pulse":
                if len(parts) != 4:
                    print("usage: pulse <node> <torque_nm> <seconds>")
                    continue
                node = int(parts[1])
                torque = max(-args.safe_limit, min(args.safe_limit, float(parts[2])))
                seconds = float(parts[3])
                send_torque_cmd(bus, node, torque)
                print(f"sent {torque:.3f} Nm to node {node} for {seconds:.2f}s")
                time.sleep(seconds)
                send_torque_cmd(bus, node, 0.0)
                print(f"node {node} zeroed")
            elif cmd == "sweep":
                if len(parts) != 6:
                    print("usage: sweep <node> <start> <stop> <step> <dt>")
                    continue
                node = int(parts[1])
                start = float(parts[2])
                stop = float(parts[3])
                step = float(parts[4])
                dt = float(parts[5])
                run_sweep(bus, node, start, stop, step, dt, args.safe_limit)
            else:
                if len(parts) != 2:
                    print("usage: <node> <torque_nm>")
                    continue
                node = int(parts[0])
                if node not in NODES:
                    print(f"unknown node {node}; valid nodes: {sorted(NODES.keys())}")
                    continue
                torque = max(-args.safe_limit, min(args.safe_limit, float(parts[1])))
                send_torque_cmd(bus, node, torque)
                print(f"sent {torque:.3f} Nm to {NODES[node]} (~{torque / KT_EFFECTIVE:.3f} A command on STM32)")

    except KeyboardInterrupt:
        print("\nCtrl+C received")
    finally:
        stop_evt.set()
        if not args.no_auto_zero:
            try:
                zero_all(bus)
                print("all nodes zeroed on exit")
            except Exception as exc:
                print(f"failed to zero on exit: {exc}")
        bus.shutdown()
        print("CAN bus closed")


if __name__ == "__main__":
    main()
