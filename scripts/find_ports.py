#!/usr/bin/env python3
"""Find which port is the ESP32 (it sends 'ODM,' lines) and which is the LiDAR.
Works no matter which Pi USB socket each cable is in.
ESP32-S3 (native USB) shows up as /dev/ttyACM*, the LiDAR as /dev/ttyUSB*: then no probing is needed.
Prints shell assignments:  ESP=/dev/ttyXXX  LIDAR=/dev/ttyUSBy
Usage:  eval "$(python3 ~/slam-robot/scripts/find_ports.py)"
"""
import glob
import sys
import time

import serial

acm = sorted(glob.glob('/dev/ttyACM*'))
usb = sorted(glob.glob('/dev/ttyUSB*'))
if acm and usb:                       # ESP32-S3 board: ACM = ESP32, USB = LiDAR
    sys.stderr.write(f'ESP32 = {acm[0]}   LiDAR = {usb[0]}   (ESP32-S3 native USB)\n')
    print(f'ESP={acm[0]} LIDAR={usb[0]}')
    sys.exit(0)

ports = usb + acm
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
