#!/usr/bin/env python3
"""Simple serial monitor for ESP32-S3"""
import sys
import time

try:
    import serial
except ImportError:
    print("Installing pyserial with --user flag...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "--user", "pyserial"])
    import serial

port = '/dev/cu.usbmodem2401'
baud = 115200

print(f"Connecting to {port} at {baud} baud...")
print("Press Ctrl+C to exit\n")

try:
    ser = serial.Serial(port, baud, timeout=1)
    print("Connected! Waiting for data...\n")
    
    while True:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(line)
        time.sleep(0.01)
except serial.SerialException as e:
    print(f"Error: {e}")
    print("\nMake sure:")
    print("1. ESP32-S3 is connected via USB")
    print("2. No other program is using the serial port")
    print("3. Try pressing RESET on the ESP32-S3")
except KeyboardInterrupt:
    print("\n\nExiting...")
    if 'ser' in locals():
        ser.close()

