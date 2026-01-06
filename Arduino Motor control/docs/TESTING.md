# Testing Guide for Arduino Motor Control Firmware

This guide helps you verify that the firmware is working correctly after uploading to your Arduino Uno.

## Pre-Flight Checklist

Before powering up:

- [ ] Arduino Uno is securely mounted
- [ ] Monster Moto Shield is properly seated on Arduino
- [ ] Left motor connected to M1 terminals
- [ ] Right motor connected to M2 terminals
- [ ] Left encoder connected: A→Pin 2, B→Pin 10, GND, VCC
- [ ] Right encoder connected: A→Pin 3, B→Pin 11, GND, VCC
- [ ] Motor power supply connected (7-30V depending on motors)
- [ ] USB cable connected for serial communication
- [ ] Wheels can spin freely (robot lifted or on low-friction surface)

## Test 1: Upload and Boot

1. **Upload firmware**:
   ```bash
   cd "/Users/magnus/Documents/PlatformIO/Projects/Arduino Motor control"
   pio run --target upload
   ```

2. **Expected output**:
   ```
   Linking .pio/build/uno/firmware.elf
   Checking size .pio/build/uno/firmware.elf
   RAM:   [==        ]  16.0% (used 327 bytes from 2048 bytes)
   Flash: [===       ]  29.9% (used 9644 bytes from 32256 bytes)
   ...
   SUCCESS
   ```

3. **Verify boot**:
   - Arduino LED should blink briefly
   - No smoke or strange smells
   - Motors should be stopped

## Test 2: Serial Communication

1. **Open serial monitor**:
   ```bash
   pio device monitor -b 115200
   ```

2. **Test with Python script** (recommended):

   Create `test_connection.py`:
   ```python
   import serial
   import struct
   import time

   # CRC16-CCITT-FALSE implementation
   def crc16_ccitt_false(data):
       crc = 0xFFFF
       for byte in data:
           crc ^= byte << 8
           for _ in range(8):
               if crc & 0x8000:
                   crc = (crc << 1) ^ 0x1021
               else:
                   crc = crc << 1
           crc &= 0xFFFF
       return crc

   # Send a frame
   def send_frame(ser, msg_type, payload):
       frame = bytearray()
       frame.append(0xAA)  # SOF0
       frame.append(0x55)  # SOF1
       frame.append(msg_type)
       frame.append(len(payload))
       frame.extend(payload)

       # Calculate CRC over TYPE, LEN, PAYLOAD
       crc_data = bytes([msg_type, len(payload)]) + payload
       crc = crc16_ccitt_false(crc_data)
       frame.append(crc & 0xFF)        # CRC low
       frame.append((crc >> 8) & 0xFF) # CRC high

       ser.write(frame)
       print(f"Sent: {frame.hex()}")

   # Test connection
   ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Adjust port for Windows/Mac
   time.sleep(2)  # Wait for Arduino to boot

   # Send PING message
   print("Sending PING...")
   ping_payload = struct.pack('<I', int(time.time() * 1000))
   send_frame(ser, 0x7E, ping_payload)

   # Wait for PONG
   time.sleep(0.1)
   if ser.in_waiting > 0:
       response = ser.read(ser.in_waiting)
       print(f"Received: {response.hex()}")
       print("✓ Communication working!")
   else:
       print("✗ No response - check connections")

   ser.close()
   ```

   Run: `python test_connection.py`

3. **Expected result**:
   - Should receive PONG message (0x7F)
   - Format: `AA 55 7F 04 [timestamp] [CRC]`

## Test 3: Encoder Reading

1. **Enable odometry streaming**:

   ```python
   # test_encoders.py
   import serial
   import struct
   import time

   # ... (use crc16 and send_frame from above)

   ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
   time.sleep(2)

   # Enable streaming at 50ms interval
   print("Enabling odometry stream...")
   payload = struct.pack('<BH', 1, 50)  # enable=1, interval=50ms
   send_frame(ser, 0x14, payload)  # MSG_ENABLE_STREAM

   # Read odometry for 5 seconds
   print("\nReading odometry (manually rotate wheels)...\n")
   start_time = time.time()

   while time.time() - start_time < 5:
       if ser.in_waiting >= 24:  # Minimum frame size for ODOM
           # Parse frame (simplified - just check for SOF)
           data = ser.read(ser.in_waiting)
           if len(data) >= 2 and data[0] == 0xAA and data[1] == 0x55:
               msg_type = data[2]
               if msg_type == 0x01:  # ODOM
                   # Parse payload (you'll need to implement full frame parsing)
                   print(f"ODOM received: {data.hex()}")
       time.sleep(0.05)

   ser.close()
   ```

2. **Manual test**:
   - Slowly rotate left wheel forward
   - Should see ODOM messages with positive dL_ticks
   - Rotate right wheel forward
   - Should see ODOM messages with positive dR_ticks

3. **Expected result**:
   - Encoder counts should increase/decrease with wheel rotation
   - Direction should match physical rotation
   - If backwards, check encoder wiring or inversion settings

## Test 4: Motor Control (No Load)

**WARNING**: Ensure robot is lifted off the ground or wheels can spin freely!

1. **Test basic movement**:

   ```python
   # test_motors.py
   import serial
   import struct
   import time

   # ... (use send_frame from above)

   ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
   time.sleep(2)

   # Test 1: Very slow forward
   print("Test 1: Slow forward (0.05 m/s)...")
   payload = struct.pack('<ff', 0.05, 0.05)  # left=0.05, right=0.05
   send_frame(ser, 0x10, payload)  # MSG_SET_VEL
   time.sleep(3)

   # Stop
   print("Stopping...")
   payload = struct.pack('<ff', 0.0, 0.0)
   send_frame(ser, 0x10, payload)
   time.sleep(1)

   # Test 2: Medium forward
   print("Test 2: Medium forward (0.15 m/s)...")
   payload = struct.pack('<ff', 0.15, 0.15)
   send_frame(ser, 0x10, payload)
   time.sleep(3)

   # Stop
   send_frame(ser, 0x1A, b'')  # MSG_STOP
   time.sleep(1)

   # Test 3: Differential (turn in place)
   print("Test 3: Turn in place...")
   payload = struct.pack('<ff', 0.1, -0.1)  # left forward, right backward
   send_frame(ser, 0x10, payload)
   time.sleep(2)

   # Stop
   send_frame(ser, 0x1A, b'')

   print("Tests complete!")
   ser.close()
   ```

2. **Observe**:
   - Motors should start smoothly (no jerking)
   - Speed should be proportional to commanded velocity
   - Motors should stop when commanded
   - No excessive noise or vibration

3. **Expected result**:
   - ✓ Smooth acceleration and deceleration
   - ✓ Motors respond to commands within ~100ms
   - ✓ No oscillation at low speeds (deadband working)

## Test 5: PID Performance

1. **Enable odometry streaming** to monitor velocity:

   ```python
   # test_pid.py
   import serial
   import struct
   import time

   # ... setup serial and functions ...

   # Enable streaming
   payload = struct.pack('<BH', 1, 50)
   send_frame(ser, 0x14, payload)
   time.sleep(0.5)

   # Command velocity
   print("Testing PID response...")
   payload = struct.pack('<ff', 0.1, 0.1)
   send_frame(ser, 0x10, payload)

   # Monitor for 5 seconds
   # You'll need to parse ODOM messages and extract velocities
   # Look for stable velocity around 0.1 m/s after ~1 second

   time.sleep(5)
   send_frame(ser, 0x1A, b'')  # Stop
   ```

2. **Good PID behavior**:
   - Velocity reaches target within 1-2 seconds
   - Minimal overshoot (<10%)
   - Stable at target (±5%)
   - No continuous oscillation

3. **Poor PID behavior** (needs tuning):
   - Large overshoot (>20%) → Reduce Kp
   - Slow response (>3 seconds) → Increase Kp
   - Oscillation → Reduce Kp, increase Kd
   - Steady-state error → Increase Ki

## Test 6: Configuration Persistence

1. **Change PID values and save**:

   ```python
   # test_eeprom.py
   import serial
   import struct
   import time

   # ... setup ...

   # Set new PID values
   print("Setting new PID values...")
   payload = struct.pack('<fff', 15.0, 8.0, 0.2)
   send_frame(ser, 0x11, payload)  # MSG_SET_PID
   time.sleep(0.1)

   # Save to EEPROM
   print("Saving to EEPROM...")
   send_frame(ser, 0x17, b'')  # MSG_SAVE_CONFIG
   time.sleep(0.5)

   print("Power cycle Arduino and verify values are loaded on boot")
   ser.close()
   ```

2. **Power cycle Arduino**

3. **Read configuration**:
   ```python
   # Request configuration
   send_frame(ser, 0x15, b'')  # MSG_GET_CONFIG
   time.sleep(0.1)

   # Parse response (MSG_CONFIG_RESP = 0x16)
   # Should contain saved values
   ```

4. **Expected result**:
   - Configuration survives power cycle
   - Values match what was saved

## Verification Checklist

After completing all tests:

- [ ] Serial communication works (PING/PONG)
- [ ] Encoders count correctly in both directions
- [ ] Motors respond to velocity commands
- [ ] Left and right motors turn in correct direction
- [ ] PID controller maintains target velocity
- [ ] No oscillation at low speeds
- [ ] STOP command works immediately
- [ ] Configuration saves to EEPROM
- [ ] Odometry streaming works

## Common Issues

### Issue: Motors run backwards
**Fix**: Swap motor wires or change inversion in code (line 201 of main.cpp)

### Issue: Encoders count backwards
**Fix**: Swap encoder A/B wires or modify ISR logic (lines 159-175)

### Issue: Motors oscillate
**Fix**: Reduce Kp gain, increase deadband values

### Issue: Motors don't start at low speeds
**Fix**: Increase deadband forward/reverse values

### Issue: Velocity overshoots target
**Fix**: Reduce Kp, increase Kd

### Issue: Velocity has steady-state error
**Fix**: Increase Ki

### Issue: No odometry messages
**Fix**:
- Verify streaming enabled
- Check baud rate (115200)
- Verify frame parsing code

## Next Steps

Once all tests pass:

1. Lower robot to ground
2. Test with load (driving on floor)
3. Re-tune PID if needed (loaded behavior differs)
4. Calibrate odometry accuracy
5. Integrate with higher-level navigation stack

## Safety Notes

- Always test with motors free-spinning first
- Use a current-limited power supply during testing
- Have emergency stop ready (power switch or STOP command)
- Monitor motor temperature during extended testing
- Don't exceed 0.3 m/s without proper testing

Good luck with your testing!
