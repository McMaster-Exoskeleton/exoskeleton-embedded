import can
import struct
import threading # // get data (here), send data, process data (ML)
from collections import deque
from time import sleep

hz = 500
target = 187 # // request by dilly

# format: IMU#_GYRO/ACCEL_ID
# LEFT HIP
imu1_a_id = 0x123
imu1_g_id = 0x124

# RIGHT HIP
imu2_a_id = 0x125
imu2_g_id = 0x126

''' for the future
# LEFT KNEE
imu3_a_id = 0x127
imu3_g_id = 0x128

# RIGHT KNEE
imu4_a_id = 0x129
imu4_g_id = 0x130
'''

# // map CAN IDs to (imu_key, sensor_type) for cleaner lookup
can_id_map = {
    imu1_a_id: ("imu1", "accel"),
    imu1_g_id: ("imu1", "gyro"),
    imu2_a_id: ("imu2", "accel"),
    imu2_g_id: ("imu2", "gyro"),
}

def make_imu_state():
    return {
        'accumulator': 0,
        'queue': deque(maxlen=target),
        'temp_accel': None,
        'temp_gyro': None,
        'timestamp': 0.0
        # // encoder reading to be added?
    }

imus = {
    "imu1": make_imu_state(),
    "imu2": make_imu_state(),
    # // future IMUS, add below
}

queue_lock = threading.Lock()

print("buffering joint data...")

def process_msg(msg: can.Message):
    can_id = msg.arbitration_id

    # // lookup imu key and sensor type from the CAN ID map
    lookup = can_id_map.get(can_id)
    if lookup is None:
        return

    imu_key, sensor_type = lookup

    # // access a specific imus data for appending
    stream_state = imus[imu_key]

    try:
        #  seq = msg.data[0]
        x_raw, y_raw, z_raw = struct.unpack('<hhh', msg.data[0:6])
        val = (x_raw / 100.0, y_raw / 100.0, z_raw / 100.0)

        # // id's are contigous
        if sensor_type == "accel":
            stream_state['temp_accel'] = val
            stream_state['timestamp'] = msg.timestamp * 1000 # // s -> ms
        else:
            stream_state['temp_gyro'] = val
    
    except Exception as e:
        print(f"noo, cannot unpack! {e}")
        
        return

    # // its crucial that the contiguous data (accel + gyro) is a snychronized pair
    if stream_state['temp_accel'] is not None and stream_state['temp_gyro'] is not None:

        # // accumulate :p
        stream_state['accumulator'] += target

        if stream_state['accumulator'] >= hz:
            # // mettre la donnees dans le file d'attente (+horodatage)
            data_tuple = (
                stream_state['timestamp'], 
                stream_state['temp_accel'], 
                stream_state['temp_gyro']
            )
            
            # Lock once data_tuple is built once
            with queue_lock:
                stream_state['queue'].append(data_tuple)
                        
            # // reset accumulator :p
            stream_state['accumulator'] -= hz

        # // clear the temporary slots to wait for the next pair from the bus
        stream_state['temp_accel'] = None
        stream_state['temp_gyro'] = None
    
# // hardware thread to be running in background
def can_listener():
    try:
        bus = can.interface.Bus(channel='can1', interface='socketcan')
        for msg in bus:
            process_msg(msg)
    except can.CanError as e:
        print(f"CAN error: {e}")
    finally:
        # // gets rid of annoying wall of text
        if 'bus' in locals():
            bus.shutdown()

# // for ML -> get data of any joint
def get_imu_data(imu_key):
    if imu_key in imus:
        with queue_lock:
            return list(imus[imu_key]['queue'])
    
    return []

def print_full_queue(imu_key):
    data = get_imu_data(imu_key)

    if len(data) == 0:
        print(f"[{imu_key}] empty queue, wait a second")
        return
    
    print(f"\n [{imu_key}] full queue snapshot ({len(data)}) samples")

    for i, data_point in enumerate(data):
        timestamp, accel, gyro = data_point
        print(f"[{i+1:3}] Time: {timestamp:.4f} | "
              f"Accel: {accel[0]:6.2f}, {accel[1]:6.2f}, {accel[2]:6.2f} | "
              f"Gyro: {gyro[0]:6.2f}, {gyro[1]:6.2f}, {gyro[2]:6.2f}")
        
    print()

def print_latest(imu_key):
    data = get_imu_data(imu_key)

    if len(data) == 0:
        print(f"[{imu_key}] empty queue, wait a second")
        return

    latest_time, latest_accel, latest_gyro = data[-1]

    print(f"\n [{imu_key}] snapshot taken")
    print(f"total samples in queueueue: {len(data)}")
    print(f"UNIX time (ms): {latest_time}")
    print(f"accel (x,y,z): {latest_accel}")
    print(f"gyro (x,y,z): {latest_gyro}")
    
    print()

def print_help():
    print("\n commands:")
    print("  q1  - full queue for imu1")
    print("  q2  - full queue for imu2")
    print("  l1  - latest value for imu1")
    print("  l2  - latest value for imu2")
    print()

def main():
    print("exo telemetry, i choose you")

    # // start accumulating queues in background
    can_thread = threading.Thread(target=can_listener, daemon=True)
    can_thread.start()

    print("initially filling queues")
    sleep(1)

    print_help()

    try:
        # // ML stuff here
        while True:
            user_input = input("> ").strip().lower()

            if user_input == 'q1':
                print_full_queue("imu1")
            elif user_input == 'q2':
                print_full_queue("imu2")
            elif user_input == 'l1':
                print_latest("imu1")
            elif user_input == 'l2':
                print_latest("imu2")
            else:
                print("unknown command, type 'h' for help")

    except KeyboardInterrupt:
        print("\n stopping telemetry")

if __name__ == "__main__":
    main()