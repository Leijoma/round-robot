#!/usr/bin/env python3
"""
Send MSG_SET_DEADBAND command to update deadband values
"""
import socket
import sys
import os

# Add server directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../robot-ui/server'))
from robotlink import RobotLink

# ESP32 robot bridge IP and port
ESP32_IP = "192.168.68.52"
ESP32_PORT = 5000

def main():
    print("Setting deadband to PWM 40...")
    print(f"Connecting to robot at {ESP32_IP}:{ESP32_PORT}")

    # Create RobotLink instance
    link = RobotLink(ESP32_IP, ESP32_PORT)

    # Set deadband: all values to 40.0
    print("Sending MSG_SET_DEADBAND (left_fwd=40, left_rev=40, right_fwd=40, right_rev=40)")
    if link.set_deadband(40.0, 40.0, 40.0, 40.0):
        print("✓ Deadband command sent successfully")
    else:
        print("✗ Failed to send deadband command")
        return 1

    # Save to EEPROM (MSG_SAVE_CONFIG = 0x17)
    print("\nSaving configuration to EEPROM...")
    if link.send_frame(0x17):  # MSG_SAVE_CONFIG
        print("✓ Configuration saved successfully")
    else:
        print("✗ Failed to save configuration")
        return 1

    print("\nDone! Deadband updated to 40 and saved to EEPROM.")
    return 0

if __name__ == "__main__":
    sys.exit(main())
