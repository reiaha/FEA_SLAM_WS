#!/usr/bin/env python3
"""
Direct Motor Test Script
Bypasses ROS to test Arduino motor commands via serial
"""
import serial
import time
import sys

def test_motors(port='/dev/ttyACM0', baud=115200):
    """Send test commands directly to Arduino"""
    try:
        print(f"Opening {port} at {baud} baud...")
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)  # Wait for Arduino reset
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print("✓ Serial port opened\n")
        
        tests = [
            ("Stop (both motors 0)", "MOTOR:0,0\n"),
            ("Forward slow (both 100)", "MOTOR:100,100\n"),
            ("Stop", "MOTOR:0,0\n"),
            ("Forward medium (both 150)", "MOTOR:150,150\n"),
            ("Stop", "MOTOR:0,0\n"),
            ("Rotate right (left 120, right -120)", "MOTOR:120,-120\n"),
            ("Stop", "MOTOR:0,0\n"),
            ("Reverse slow (both -100)", "MOTOR:-100,-100\n"),
            ("Stop", "MOTOR:0,0\n"),
        ]
        
        print("Starting motor tests...")
        print("=" * 60)
        
        for i, (desc, cmd) in enumerate(tests, 1):
            print(f"\n[{i}/{len(tests)}] {desc}")
            print(f"    Sending: {cmd.strip()}")
            ser.write(cmd.encode())
            ser.flush()
            
            # Read any response
            time.sleep(0.1)
            if ser.in_waiting:
                response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                if response.strip():
                    print(f"    Arduino: {response.strip()}")
            
            # Hold command for 2 seconds
            time.sleep(2)
        
        print("\n" + "=" * 60)
        print("✓ Test complete - motors should be stopped")
        ser.close()
        
    except serial.SerialException as e:
        print(f"✗ Serial error: {e}")
        print("\nTroubleshooting:")
        print("  1. Check Arduino is connected: ls -l /dev/ttyACM*")
        print("  2. Stop ROS bridge: pkill -f arduino_motor_bridge")
        print("  3. Check permissions: sudo usermod -a -G dialout $USER")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\n⚠ Interrupted - sending stop command")
        if 'ser' in locals() and ser.is_open:
            ser.write(b"MOTOR:0,0\n")
            ser.close()
        sys.exit(0)

if __name__ == '__main__':
    print("=" * 60)
    print("DIRECT MOTOR TEST")
    print("=" * 60)
    print("\nThis script sends motor commands directly to the Arduino.")
    print("Watch/listen for motor movement during each 2-second test.\n")
    
    # Stop any running bridge
    import subprocess
    subprocess.run(['pkill', '-f', 'arduino_motor_bridge'], 
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.5)
    
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    test_motors(port)
