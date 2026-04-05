import can

print("Initializing CAN bus...")

bus = can.Bus(interface='socketcan', channel='can1')

expected_seq = None
dropped_count = 0
total_received = 0

print("Listening on can1...")

try:
    while True:
        msg = bus.recv(timeout = 1.0)
        
        if msg is None:
            print("Timeout")
            
            continue
            
        total_received += 1
        
        if total_received <= 5:
            hex_data = ', '.join([hex(b) for b in msg.data])
            
            print(f"DEBUG - Rx Frame: ID={hex(msg.arbitration_id)} | Extended={msg.is_extended_id} | Data=[{hex_data}]")

        if msg.arbitration_id == 0x123:
            current_seq = msg.data[0]
            
            if expected_seq is not None:
                if current_seq != expected_seq:
                    if current_seq > expected_seq:
                        missed = current_seq - expected_seq
                    else:
                        missed = (256 - expected_seq) + current_seq
                        
                    dropped_count += missed
                    
                    print(f"{missed} frames dropped. (Expected {expected_seq}, Got {current_seq})")
            
            expected_seq = (current_seq + 1) % 256 # Half of 512 byte
            
            if current_seq == 0:
                print(f"Total Accel frames: {total_received}. Dropped Count: {dropped_count}")

except KeyboardInterrupt:
    print(f"\nTotal messages recieved: {total_received}. Total dropped: {dropped_count}")