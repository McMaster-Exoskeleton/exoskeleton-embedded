import can
import collections

accel_history = collections.deque(maxlen=500)
gyro_history = collections.deque(maxlen=500)

bus = can.Bus(interface='socketcan', channel='can1')

print("Buffering IMU data...")

while True:
    msg = bus.recv()
    
    if msg.arbitration_id == 0x123:
        ax = int.from_bytes(msg.data[1:3], byteorder='little', signed=True)
        ay = int.from_bytes(msg.data[3:5], byteorder='little', signed=True)
        az = int.from_bytes(msg.data[5:7], byteorder='little', signed=True)
        
        accel_history.append((ax, ay, az))