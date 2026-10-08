#!/usr/bin/env python3
"""Find which /dev/ttyUSB* is the ESP32 (it sends 'ODM,' lines) and which is the LiDAR.
Works no matter which Pi USB socket each cable is in.
Prints shell assignments:  ESP=/dev/ttyUSBx  LIDAR=/dev/ttyUSBy
Usage:  eval "$(python3 ~/slam-robot/scripts/find_ports.py)"
"""
import glob
import sys
import time

import serial

ports = sorted(glob.glob('/dev/ttyUSB*'))
if len(ports) < 2:
    sys.stderr.write(f'Need 2 USB-serial devices (LiDAR + ESP32), found: {ports}\n')
    sys.exit(1)

esp = None
for p in ports:
    try:
        s = serial.Serial()
        s.port, s.baudrate, s.timeout = p, 115200, 0.2
        s.dtr = False
        s.rts = False
        s.open()
        end, buf = time.time() + 3.0, b''      # opening resets the ESP32: allow ~1 s to boot
        while time.time() < end and b'ODM,' not in buf:
            buf += s.read(512)
        s.close()
        if b'ODM,' in buf:
            esp = p
            break
    except Exception as e:  # busy / permission
        sys.stderr.write(f'{p}: {e}\n')

if not esp:
    sys.stderr.write('No port sends ODM lines. Is the ESP32 powered and is anything else (an old launch) holding the port?\n')
    sys.exit(1)
lidar = [p for p in ports if p != esp][0]
sys.stderr.write(f'ESP32 = {esp}   LiDAR = {lidar}\n')
print(f'ESP={esp} LIDAR={lidar}')
