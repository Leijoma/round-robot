#!/usr/bin/env python3
"""
Read Arduino Firmware Configuration

Reads and displays current firmware settings via RobotLink protocol:
- Robot configuration (wheelbase, diameter, ticks_per_rev)
- PID parameters (Kp, Ki, Kd)
- Deadband values (per motor, per direction)

Usage:
    python3 read_firmware_config.py --host 192.168.68.52
"""

import argparse
import time
import sys
from robotlink import RobotLink, MessageType, ConfigPayload, StatusPayload

class FirmwareConfigReader:
    """Read and display Arduino firmware configuration"""

    def __init__(self, host: str, port: int = 5000):
        self.host = host
        self.port = port
        self.robot = RobotLink(host=host, port=port)

        # Configuration data
        self.config = None
        self.status = None
        self.config_received = False
        self.status_received = False

        # Register callbacks
        self.robot.register_callback(MessageType.MSG_CONFIG_RESP, self._on_config)
        self.robot.register_callback(MessageType.MSG_STATUS, self._on_status)

    def _on_config(self, payload: bytes):
        """Handle CONFIG_RESP message"""
        try:
            self.config = ConfigPayload.unpack(payload)
            self.config_received = True
        except Exception as e:
            print(f"Error unpacking config: {e}")

    def _on_status(self, payload: bytes):
        """Handle STATUS message"""
        try:
            self.status = StatusPayload.unpack(payload)
            self.status_received = True
        except Exception as e:
            print(f"Error unpacking status: {e}")

    def connect(self) -> bool:
        """Connect to robot"""
        print(f"Connecting to robot at {self.host}:{self.port}...", end=" ", flush=True)

        # Test connection with ping
        self.robot.ping(int(time.time() * 1000) & 0xFFFFFFFF)
        time.sleep(0.1)

        # Process any incoming messages
        count = self.robot.process_messages(timeout=0.5)

        print("✓ Connected")
        return True

    def request_config(self) -> bool:
        """Request robot configuration"""
        print("Requesting configuration...", end=" ", flush=True)

        self.robot.get_config()

        # Wait for response
        start = time.time()
        while time.time() - start < 2.0:
            self.robot.process_messages(timeout=0.1)
            if self.config_received:
                print("✓ Received")
                return True

        print("✗ Timeout")
        return False

    def request_status(self) -> bool:
        """Request robot status (if supported)"""
        print("Requesting status...", end=" ", flush=True)

        # Try sending MSG_STATUS request (may not be implemented in Arduino)
        # Note: Arduino firmware may not have MSG_STATUS implemented yet
        self.robot.send_frame(MessageType.MSG_STATUS)

        # Wait for response
        start = time.time()
        while time.time() - start < 2.0:
            self.robot.process_messages(timeout=0.1)
            if self.status_received:
                print("✓ Received")
                return True

        print("✗ Not available (not implemented in Arduino)")
        return False

    def display_config(self):
        """Display configuration in formatted output"""
        print("\n" + "="*70)
        print("ARDUINO FIRMWARE CONFIGURATION")
        print("="*70)

        if self.config:
            print("\nROBOT CONFIGURATION:")
            print(f"  Wheel Diameter:    {self.config.wheel_diameter:.3f} m")
            print(f"  Wheelbase:         {self.config.wheelbase:.3f} m")
            print(f"  Ticks per Rev:     {self.config.ticks_per_rev:.1f}")
            print(f"  Left Inverted:     {'Yes' if self.config.invert_left else 'No'}")
            print(f"  Right Inverted:    {'Yes' if self.config.invert_right else 'No'}")

            if self.config.balance_enable:
                print(f"  Balance Mode:      Enabled (gain: {self.config.balance_gain:.2f})")
            else:
                print(f"  Balance Mode:      Disabled")
        else:
            print("\n✗ Robot configuration not available")

        if self.status:
            print("\nPID PARAMETERS:")
            print(f"  Kp: {self.status.kp:.1f}")
            print(f"  Ki: {self.status.ki:.1f}")
            print(f"  Kd: {self.status.kd:.1f}")

            print("\nDEADBAND VALUES (PWM):")
            left_fwd, left_rev, right_fwd, right_rev = self.status.deadband
            print(f"  Left Motor:")
            print(f"    Forward:  {left_fwd:.1f}")
            print(f"    Reverse:  {left_rev:.1f}")
            print(f"  Right Motor:")
            print(f"    Forward:  {right_fwd:.1f}")
            print(f"    Reverse:  {right_rev:.1f}")

            print("\nSTATUS:")
            print(f"  PID Enabled:       {'Yes' if self.status.pid_enabled else 'No'}")
            print(f"  Stream Enabled:    {'Yes' if self.status.stream_enabled else 'No'}")
            print(f"  Stream Interval:   {self.status.stream_interval} ms")
            print(f"  Frames Received:   {self.status.frames_received}")
            print(f"  Frames Sent:       {self.status.frames_sent}")
            print(f"  Uptime:            {self.status.uptime} seconds ({self.status.uptime/60:.1f} min)")
        else:
            print("\n⚠️  STATUS INFO NOT AVAILABLE")
            print("    MSG_STATUS not implemented in current Arduino firmware")
            print("    PID and deadband values cannot be read remotely")
            print()
            print("    WORKAROUND: Check values via Arduino USB Serial debug output")
            print("    OR: Add MSG_STATUS handler to Arduino firmware")

        # Show default values from code for reference
        print("\n" + "-"*70)
        print("DEFAULT VALUES (from Arduino source code):")
        print("  PID:      Kp=10.0, Ki=5.0, Kd=0.1")
        print("  Deadband: 30.0 PWM (all motors/directions)")
        print()
        print("NOTE: If EEPROM has been written, actual values may differ!")
        print("      Expected EEPROM deadband: 40.0 PWM (per Magnus)")
        print("="*70 + "\n")

    def run(self):
        """Run configuration read sequence"""
        try:
            # Connect
            if not self.connect():
                print("✗ Failed to connect to robot")
                return False

            # Request configuration
            config_ok = self.request_config()

            # Try to request status (may not be implemented)
            status_ok = self.request_status()

            # Display results
            self.display_config()

            # Close connection
            self.robot.close()

            return config_ok

        except Exception as e:
            print(f"\n✗ Error: {e}")
            import traceback
            traceback.print_exc()
            self.robot.close()
            return False


def main():
    parser = argparse.ArgumentParser(description='Read Arduino firmware configuration')
    parser.add_argument('--host', type=str, default='192.168.68.52',
                        help='ESP32 IP address (default: 192.168.68.52)')
    parser.add_argument('--port', type=int, default=5000,
                        help='ESP32 UDP port (default: 5000)')

    args = parser.parse_args()

    reader = FirmwareConfigReader(host=args.host, port=args.port)

    success = reader.run()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
