import struct
import sys
import threading
from collections import deque
from pathlib import Path
from time import sleep

import can

try:
    import can_common
    import can_imu
except ImportError:
    REPO_ROOT = Path(__file__).resolve().parents[2]
    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))
    from apis.can.python import can_common, can_imu

# STM32 streams accel+gyro pairs at 500 Hz. The ML model wants 187 Hz, so we
# downsample with a Bresenham-style accumulator below: every paired sample adds
# `target` to an accumulator and emits one entry whenever it crosses `hz`. This
# spreads the kept samples uniformly without timer jitter.
hz = 500
target = 187

NODE_TO_IMU = {
    can_common.NODE_LEFT_HIP: "imu1",
    can_common.NODE_RIGHT_HIP: "imu2",
    can_common.NODE_LEFT_KNEE: "imu3",
    can_common.NODE_RIGHT_KNEE: "imu4",
}

IMU_LABELS = {
    "imu1": "left_hip",
    "imu2": "right_hip",
    "imu3": "left_knee",
    "imu4": "right_knee",
}

def make_imu_state():
    return {
        "accumulator": 0,
        "queue": deque(maxlen=target),
        "temp_accel": None,
        "temp_gyro": None,
        "timestamp": 0.0,
    }


imus = {imu_key: make_imu_state() for imu_key in IMU_LABELS}
queue_lock = threading.Lock()

print("buffering joint data...")


def process_msg(msg: can.Message):
    if msg.is_extended_id:
        return

    msg_type, src_node, dest = can_common.parse_can_id(msg.arbitration_id)
    if dest != can_common.NODE_PI:
        return

    imu_key = NODE_TO_IMU.get(src_node)
    if imu_key is None:
        return

    stream_state = imus[imu_key]

    try:
        if msg_type == can_common.MSG_IMU_ACCEL:
            stream_state["temp_accel"] = can_imu.parse_imu_accel(msg)
            stream_state["timestamp"] = msg.timestamp * 1000.0
        elif msg_type == can_common.MSG_IMU_GYRO:
            stream_state["temp_gyro"] = can_imu.parse_imu_gyro(msg)
        else:
            return
    except struct.error as e:
        print(f"cannot unpack IMU frame 0x{msg.arbitration_id:03X}: {e}")
        return

    # Keep accel and gyro paired before adding a sample to the public queue.
    if stream_state["temp_accel"] is not None and stream_state["temp_gyro"] is not None:
        stream_state["accumulator"] += target

        if stream_state["accumulator"] >= hz:
            data_tuple = (
                stream_state["timestamp"],
                stream_state["temp_accel"],
                stream_state["temp_gyro"],
            )

            with queue_lock:
                stream_state["queue"].append(data_tuple)

            stream_state["accumulator"] -= hz

        stream_state["temp_accel"] = None
        stream_state["temp_gyro"] = None


def can_listener(channel="can1"):
    try:
        bus = can_common.create_bus(channel=channel)
        for msg in bus:
            process_msg(msg)
    except can.CanError as e:
        print(f"CAN error: {e}")
    finally:
        if "bus" in locals():
            bus.shutdown()


def get_imu_data(imu_key):
    if imu_key in imus:
        with queue_lock:
            return list(imus[imu_key]["queue"])

    return []


def print_full_queue(imu_key):
    data = get_imu_data(imu_key)

    if len(data) == 0:
        print(f"[{imu_key}] empty queue, wait a second")
        return

    print(f"\n [{imu_key} / {IMU_LABELS[imu_key]}] full queue snapshot ({len(data)}) samples")

    for i, data_point in enumerate(data):
        timestamp, accel, gyro = data_point
        print(
            f"[{i + 1:3}] Time: {timestamp:.4f} | "
            f"Accel: {accel[0]:6.2f}, {accel[1]:6.2f}, {accel[2]:6.2f} | "
            f"Gyro: {gyro[0]:6.2f}, {gyro[1]:6.2f}, {gyro[2]:6.2f}"
        )

    print()


def print_latest(imu_key):
    data = get_imu_data(imu_key)

    if len(data) == 0:
        print(f"[{imu_key}] empty queue, wait a second")
        return

    latest_time, latest_accel, latest_gyro = data[-1]

    print(f"\n [{imu_key} / {IMU_LABELS[imu_key]}] snapshot taken")
    print(f"total samples in queue: {len(data)}")
    print(f"UNIX time (ms): {latest_time}")
    print(f"accel (x,y,z): {latest_accel}")
    print(f"gyro (x,y,z): {latest_gyro}")
    print()


def print_help():
    print("\n commands:")
    for index, imu_key in enumerate(IMU_LABELS, start=1):
        print(f"  q{index}  - full queue for {imu_key} / {IMU_LABELS[imu_key]}")
    for index, imu_key in enumerate(IMU_LABELS, start=1):
        print(f"  l{index}  - latest value for {imu_key} / {IMU_LABELS[imu_key]}")
    print("  h   - help")
    print()


def main():
    print("exo telemetry, i choose you")

    can_thread = threading.Thread(target=can_listener, daemon=True)
    can_thread.start()

    print("initially filling queues")
    sleep(1)

    print_help()

    try:
        while True:
            user_input = input("> ").strip().lower()

            if user_input == "h":
                print_help()
            elif len(user_input) == 2 and user_input[0] in {"q", "l"} and user_input[1].isdigit():
                imu_key = f"imu{user_input[1]}"
                if imu_key not in imus:
                    print("unknown imu, type 'h' for help")
                elif user_input[0] == "q":
                    print_full_queue(imu_key)
                else:
                    print_latest(imu_key)
            else:
                print("unknown command, type 'h' for help")

    except KeyboardInterrupt:
        print("\n stopping telemetry")


if __name__ == "__main__":
    main()
