#!/usr/bin/env python3
"""Emergency stop - sends stop command to robot via SocketIO"""

import socketio
import time

sio = socketio.Client()

@sio.on('connect')
def on_connect():
    print("Connected to server")
    print("Sending STOP command...")
    sio.emit('motor_command', {'type': 'stop'})
    time.sleep(0.5)
    print("Stop command sent!")
    sio.disconnect()

@sio.on('disconnect')
def on_disconnect():
    print("Disconnected from server")

try:
    sio.connect('http://localhost:5001')
    time.sleep(1)
except Exception as e:
    print(f"Error: {e}")
