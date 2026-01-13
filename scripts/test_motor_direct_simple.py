#!/usr/bin/env python3
"""
Simple direct motor test - bypasses ROS completely
"""
import serial
import time

# Open serial port
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)  # Wait for Arduino

print("Connected to Arduino")
print("Reading sensor data for 3 seconds...")

# Read some sensor data first
for i in range(15):
    if ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        print(f"  {line}")
    time.sleep(0.2)

print("\n=== SENDING MOTOR COMMAND: FORWARD ===")
command = "MOTOR:120,120\n"
ser.write(command.encode())
print(f"Sent: {command.strip()}")

print("Reading Arduino response for 5 seconds...")
time.sleep(0.5)

# Read responses
for i in range(25):
    if ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        print(f"  {line}")
    time.sleep(0.2)

print("\n=== SENDING STOP COMMAND ===")
ser.write(b"STOP\n")
print("Sent: STOP")

time.sleep(1)
ser.close()
print("\nTest complete")
