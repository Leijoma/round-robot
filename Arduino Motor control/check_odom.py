#!/usr/bin/env python3
import socket, time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(0.5)
print('Listening for ODOM for 10 seconds...')
start = time.time()
count = 0
try:
    while time.time() - start < 10:
        try:
            data, _ = sock.recvfrom(1024)
            if len(data) >= 4 and data[0] == 0xAA and data[1] == 0x55:
                count += 1
                if count % 10 == 1:
                    print(f'Got #{count}: Type=0x{data[2]:02x}')
        except: pass
finally:
    print(f'Total messages: {count}')
    if count > 0:
        print(f'Rate: {count/10:.1f} Hz')
    else:
        print('NO DATA - ESP32 may need reset!')
    sock.close()
