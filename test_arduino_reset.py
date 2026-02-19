#!/usr/bin/env python3
import serial
import time

print("=== Arduino Motor Diagnostics ===\n")

try:
    print("Step 1: Opening serial port with Arduino reset...")
    # Toggling DTR causes Arduino to reset
    ser = serial.Serial()
    ser.port = '/dev/ttyACM0'
    ser.baudrate = 115200
    ser.timeout = 2
    ser.open()
    
    # Trigger Arduino reset via DTR
    ser.dtr = False
    time.sleep(0.5)
    ser.dtr = True
    time.sleep(2)  # Wait for Arduino boot
    
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    
    print(f"✓ Port opened and Arduino reset\n")
    
    print("Step 2: Clearing any buffered data from Arduino...")
    time.sleep(1)
    while ser.in_waiting > 0:
        ser.read(1)
    print("✓ Buffer cleared\n")
    
    print("Step 3: Sending START command...")
    ser.write(b'START\n')
    ser.flush()
    
    # Read for 2 seconds looking for acknowledgment
    print("   Waiting for response...")
    response = b""
    start_time = time.time()
    while time.time() - start_time < 2.0:
        if ser.in_waiting > 0:
            chunk = ser.read(ser.in_waiting)
            response += chunk
            print(f"   Got: {chunk}")
            if b'ACK:START' in response or b'Motors ENABLED' in response:
                print(f"   ✅ START acknowledged!\n")
                break
        time.sleep(0.1)
    
    if b'IGNORED' in response:
        print(f"   ❌ Motors still disabled! Response: {response}\n")
    elif not response:
        print(f"   ⚠️  No acknowledgment received\n")
    else:
        print(f"   Response: {response}\n")
    
    print("Step 4: Sending motor test command (MOTOR:150,150)...")
    ser.write(b'MOTOR:150,150\n')
    ser.flush()
    
    response = b""
    start_time = time.time()
    while time.time() - start_time < 1.0:
        if ser.in_waiting > 0:
            chunk = ser.read(ser.in_waiting)
            response += chunk
            print(f"   Got: {chunk}")
    
    if b'IGNORED' in response:
        print("   ❌ Motors still disabled!\n")
    elif b'ACK:MOTOR' in response:
        print("   ✅ Motor command acknowledged!\n")
    else:
        print(f"   Response: {response}\n")
    
    print("Step 5: Stopping motors (MOTOR:0,0)...")
    ser.write(b'MOTOR:0,0\n')
    ser.flush()
    time.sleep(0.5)
    
    ser.close()
    print("✅ Test complete")
    
except Exception as e:
    print(f"❌ Error: {e}")
    import traceback
    traceback.print_exc()
