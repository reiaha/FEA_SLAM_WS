#!/usr/bin/env python3
import serial
import time
import sys

print("=== Arduino Motor Test ===\n")

try:
    print("Opening serial port /dev/ttyACM0...")
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=2)
    print("✓ Port opened\n")
    
    time.sleep(2)  # Wait for Arduino to initialize
    
    # Clear buffers
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    
    print("1️⃣  Sending START command...")
    ser.write(b'START\n')
    ser.flush()
    time.sleep(1)
    
    # Read response
    response = b""
    start_time = time.time()
    while time.time() - start_time < 1.0:
        if ser.in_waiting > 0:
            byte = ser.read(1)
            response += byte
            print(f"   Received: {repr(byte)}")
    
    if response:
        print(f"   Full Response: {response.decode('utf-8', errors='ignore')}\n")
    else:
        print("   ⚠️  No response from Arduino\n")
    
    time.sleep(0.5)
    
    print("2️⃣  Sending motor command (MOTOR:100,100)...")
    ser.write(b'MOTOR:100,100\n')
    ser.flush()
    time.sleep(1)
    
    response = b""
    start_time = time.time()
    while time.time() - start_time < 1.0:
        if ser.in_waiting > 0:
            byte = ser.read(1)
            response += byte
            print(f"   Received: {repr(byte)}")
    
    if response:
        print(f"   Full Response: {response.decode('utf-8', errors='ignore')}\n")
    else:
        print("   ⚠️  No response from Arduino\n")
    
    print("3️⃣  Stopping motors (MOTOR:0,0)...")
    ser.write(b'MOTOR:0,0\n')
    ser.flush()
    time.sleep(0.5)
    
    response = b""
    start_time = time.time()
    while time.time() - start_time < 1.0:
        if ser.in_waiting > 0:
            byte = ser.read(1)
            response += byte
    
    if response:
        print(f"   Response: {response.decode('utf-8', errors='ignore')}\n")
    
    ser.close()
    print("✅ Test complete")
    
except Exception as e:
    print(f"❌ Error: {e}", file=sys.stderr)
    import traceback
    traceback.print_exc()
