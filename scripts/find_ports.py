#!/usr/bin/env python3
"""Find which port is the ESP32 (it sends 'ODM,' lines) and which is the LiDAR.
Works no matter which Pi USB socket each cable is in.
ESP32-S3 on the Pi's GPIO UART (firmware v2.6, pins 8/10) is /dev/ttyAMA0, the LiDAR /dev/ttyUSB*.
ESP32-S3 on native USB shows up as /dev/ttyACM*: then no probing is needed either.
Prints shell assignments:  ESP=/dev/ttyXXX  LIDAR=/dev/ttyUSBy
Usage:  eval "$(python3 ~/slam-robot/scripts/find_ports.py)"
"""
import glob
import os
import sys
import time

import serial


def sends_odm(p, secs):
    try:
        s = serial.Serial()
        s.port, s.baudrate, s.timeout = p, 115200, 0.2
        s.dtr = False
        s.rts = False
        s.open()
        end, buf = time.time() + secs, b''
        while time.time() < end and b'ODM,' not in buf:
            buf += s.read(512)
        s.close()
        return b'ODM,' in buf
    except Exception as e:  # busy / permission
        sys.stderr.write(f'{p}: {e}\n')
        return False


acm = sorted(glob.glob('/dev/ttyACM*'))
usb = sorted(glob.glob('/dev/ttyUSB*'))
uart = [os.path.realpath(p) for p in ('/dev/esp32', '/dev/ttyAMA0') if os.path.exists(p)]
uart = [p for p in dict.fromkeys(uart) if not p.startswith(('/dev/ttyUSB', '/dev/ttyACM'))]

for p in uart:                        # ESP32 on the GPIO UART: USB = LiDAR
    if sends_odm(p, 1.5):
        sys.stderr.write(f'ESP32 = {p}   LiDAR = {usb[0] if usb else "NOT FOUND"}   (Pi GPIO UART)\n')
        print(f'ESP={p} LIDAR={usb[0] if usb else "/dev/ttyUSB0"}')
        sys.exit(0)

if acm and usb:                       # ESP32-S3 board on USB: ACM = ESP32, USB = LiDAR
    sys.stderr.write(f'ESP32 = {acm[0]}   LiDAR = {usb[0]}   (ESP32-S3 native USB)\n')
    print(f'ESP={acm[0]} LIDAR={usb[0]}')
    sys.exit(0)

ports = usb + acm
if len(ports) < 2:
    sys.stderr.write(f'ESP32 not found. USB-serial devices: {ports}, GPIO UART: {uart or "none"}.\n'
                     'GPIO UART: check the 3 wires (ESP 47 -> Pi pin 10, ESP 8 <- Pi pin 8, GND -> pin 6), '
                     'the ESP32 5V, and run: sudo bash ~/slam-robot/scripts/setup_pi_uart.sh\n')
    sys.exit(1)

esp = next((p for p in ports if sends_odm(p, 3.0)), None)   # opening may reset the ESP32: allow ~1 s to boot
if not esp:
    sys.stderr.write('No port sends ODM lines. Is the ESP32 powered and is anything else (an old launch) holding the port?\n')
    sys.exit(1)
lidar = [p for p in ports if p != esp][0]
sys.stderr.write(f'ESP32 = {esp}   LiDAR = {lidar}\n')
print(f'ESP={esp} LIDAR={lidar}')
