#!/usr/bin/env python3
"""
Verbose serial debugging - see exactly what Arduino receives
"""
import serial
import time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)

print("=== Connected ===\n")

# Clear initial junk
time.sleep(0.5)
ser.reset_input_buffer()

print("=== Reading baseline data (2 seconds) ===")
for i in range(10):
    if ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        print(f"[RX] {line}")
    time.sleep(0.2)

print("\n=== SENDING: 'MOTOR:150,150\\n' ===")
command = b"MOTOR:150,150\n"
print(f"Bytes sent: {command}")
bytes_written = ser.write(command)
print(f"Wrote {bytes_written} bytes")
ser.flush()  # Ensure it's sent
print("Flushed serial buffer")

print("\n=== Reading for 5 seconds (looking for ACK and motor values) ===")
start_time = time.time()
while time.time() - start_time < 5:
    if ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        print(f"[RX] {line}")
    time.sleep(0.1)

print("\n=== SENDING: 'STOP\\n' ===")
ser.write(b"STOP\n")
ser.flush()

time.sleep(1)
print("\n=== Reading final data ===")
for i in range(5):
    if ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        print(f"[RX] {line}")
    time.sleep(0.2)

ser.close()
print("\n=== Test complete ===")
