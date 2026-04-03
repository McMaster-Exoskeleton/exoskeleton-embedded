import can
from collections import deque

hz = 500
target = 187 # // request by dilly

# format: IMU#_GYRO/ACCEL_ID
imu1_a_id = 0x123
imu1_g_id = 0x124
'''
imu2_a_id = 0x125
imu2_g_id = 0x126

imu3_a_id = 0x127
imu3_g_id = 0x128

imu4_a_id = 0x129
imu4_g_id = 0x130
'''

# // hold state (queue, time) for each IMU
can_streams = {
    imu1_a_id: {'accumulator': 0, 'queue': deque(maxlen=target)},
    imu1_g_id: {'accumulator': 0, 'queue': deque(maxlen=target)}
    # add rest
}

bus = can.Bus(interface='socketcan', channel='can1')

print("buffering joint data...")


'''
while True:
    msg = bus.recv()
    
    if msg.arbitration_id == 0x123:
        ax = int.from_bytes(msg.data[1:3], byteorder='little', signed=True)
        ay = int.from_bytes(msg.data[3:5], byteorder='little', signed=True)
        az = int.from_bytes(msg.data[5:7], byteorder='little', signed=True)
        
        accel_history.append((ax, ay, az))
'''