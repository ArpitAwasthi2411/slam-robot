#!/usr/bin/env python3
"""Talk to the ESP32 directly (stop the ROS launch first — only one program can own the port).

  python3 serial_probe.py                       # watch raw output 5 s + summary  (is it alive?)
  python3 serial_probe.py --pwm 120 120         # robot ON BLOCKS: spin both wheels, report speed
  python3 serial_probe.py --vel 200 200         # closed-loop test: 200 mm/s both wheels
  python3 serial_probe.py --calibrate           # robot ON BLOCKS: sweep PWM, suggest firmware constants
  python3 serial_probe.py --push                # roll the robot 1 m by hand, prints ticks per wheel

Needs only pyserial:  sudo apt install python3-serial
"""
import argparse
import glob
import math
import sys
import time

import serial

TICKS_PER_REV = 4740
WHEEL_D_MM = 125.0
MM_PER_TICK = math.pi * WHEEL_D_MM / TICKS_PER_REV


def find_port(p):
    if p != 'auto':
        return p
    for pat in ('/dev/esp32', '/dev/ttyACM*'):
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[0]
    sys.exit('No ESP32 port found (/dev/esp32, /dev/ttyACM*). Is the USB cable plugged in?')


def open_port(port):
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, 115200, 0.1
    s.dtr = False          # don't reset / bootloader the ESP32-S3
    s.rts = False
    try:
        s.open()
    except serial.SerialException as e:
        sys.exit(f'Cannot open {port}: {e}\n-> Is the ROS launch still running? Stop it first.')
    s.reset_input_buffer()
    return s


def read_for(s, secs, on_line=None, send=None, send_every=0.1):
    end = time.time() + secs
    nxt = 0.0
    buf = b''
    while time.time() < end:
        if send and time.time() >= nxt:
            s.write(send)
            nxt = time.time() + send_every
        buf += s.read(256)
        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1)
            if on_line:
                on_line(line.decode('ascii', 'replace').strip())


class Stats:
    def __init__(self):
        self.n = 0
        self.dl = self.dr = 0
        self.dt_ms = 0
        self.modes = {}
        self.other = []

    def __call__(self, line):
        if line.startswith('ODM,'):
            p = line.split(',')
            try:
                self.dl += int(p[1]); self.dr += int(p[2]); self.dt_ms += int(p[3])
                m = p[4] if len(p) > 4 else 'v1'
                self.modes[m] = self.modes.get(m, 0) + 1
                self.n += 1
            except (ValueError, IndexError):
                self.other.append(line)
        elif line:
            self.other.append(line)

    def speeds(self):
        if self.dt_ms <= 0:
            return 0.0, 0.0
        return self.dl * MM_PER_TICK / (self.dt_ms / 1000), self.dr * MM_PER_TICK / (self.dt_ms / 1000)


def cmd_watch(s, secs):
    st = Stats()

    def show(line):
        st(line)
        if not line.startswith('ODM,') or st.n % 25 == 1:
            print('  <', line)
    s.write(b'?\n')
    read_for(s, secs, show)
    print(f'\n{st.n} ODM lines in {secs}s = {st.n / secs:.1f} Hz   modes={st.modes}')
    print(f'ticks: left {st.dl}  right {st.dr}')
    if st.n == 0:
        print('\nNOTHING RECEIVED. Check, in order:\n'
              ' 1. Firmware v2 flashed? (Arduino IDE serial monitor should show ODM lines)\n'
              ' 2. Arduino IDE: Tools > USB CDC On Boot, USB Mode = Hardware CDC and JTAG\n'
              ' 3. Press the ESP32 RESET (RST) button, run this again\n'
              ' 4. sudo systemctl stop ModemManager   (it grabs /dev/ttyACM* devices)\n'
              ' 5. Try the other USB socket on the ESP32 board (UART vs USB)')
    elif '3' in st.modes:
        print('-> ESP32 E-STOP is latched (mode 3). Release from the dashboard, or press the ESP32 RESET button.')
    elif not any(m in st.modes for m in ('0', '1', '2')):
        print('-> v1 firmware detected (no mode field). Flash firmware v2 for velocity control.')


def cmd_drive(s, secs, line_bytes, label):
    print(f'{label} for {secs}s — robot must be ON BLOCKS. Ctrl+C to abort.')
    read_for(s, 0.5, send=line_bytes)          # spin-up
    st = Stats()
    read_for(s, secs, st, send=line_bytes)
    s.write(b'S\n')
    l, r = st.speeds()
    print(f'measured wheel speed: left {l:7.1f} mm/s   right {r:7.1f} mm/s   modes={st.modes}')
    if l < 0 or r < 0:
        print('-> a wheel turned BACKWARD for a forward command: flip its MOTOR_DIR (or ENC_DIR '
              'if it physically turned forward) in the firmware.')
    return l, r


def cmd_calibrate(s):
    print('PWM sweep — robot ON BLOCKS, wheels free. Ctrl+C to abort.\n')
    rows = []
    for pwm in (20, 30, 40, 60, 80, 100, 130, 160, 200):
        st = Stats()
        read_for(s, 0.6, send=f'P,{pwm},{pwm}\n'.encode())
        read_for(s, 1.0, st, send=f'P,{pwm},{pwm}\n'.encode())
        l, r = st.speeds()
        rows.append((pwm, l, r))
        print(f'  PWM {pwm:3d}:  left {l:7.1f} mm/s   right {r:7.1f} mm/s')
    s.write(b'S\n')
    moving = [p for p, l, r in rows if min(abs(l), abs(r)) > 15]
    pwm_min = moving[0] if moving else None
    top = rows[-1]
    print()
    if pwm_min is None:
        print('Wheels never moved: check motor power / driver enable pins.')
        return
    print('Put these in firmware/robot_esp32/robot_esp32.ino:')
    print(f'  #define PWM_MIN              {max(10, pwm_min - 5)}.0f')
    print(f'  #define MAX_WHEEL_SPEED_MMS  {min(abs(top[1]), abs(top[2])):.0f}.0f   (speed at MAX_PWM={top[0]})')
    if abs(top[1] - top[2]) / max(abs(top[1]), abs(top[2]), 1) > 0.1:
        print('  NOTE: left/right differ by >10% at full PWM — the PI loop will correct it, '
              'but check for a dragging wheel or weak motor.')


def cmd_push(s):
    print('Mark the floor, then roll the robot STRAIGHT forward exactly 1.000 m by hand.')
    print('Press Enter when done...')
    st = Stats()
    import threading
    stop = threading.Event()

    def reader():
        while not stop.is_set():
            read_for(s, 0.2, st)
    t = threading.Thread(target=reader, daemon=True)
    t.start()
    input()
    stop.set()
    t.join()
    print(f'left ticks {st.dl}   right ticks {st.dr}')
    circ = 2 * math.pi * 0.0625
    for name, n in (('left', st.dl), ('right', st.dr)):
        if n:
            print(f'  ticks_per_rev_{name}: {abs(n) * circ:.1f}   (put in config/robot_params.yaml)')
    if st.dl < 0 or st.dr < 0:
        print('-> negative ticks while rolling forward: flip that wheel\'s ENC_DIR in the firmware.')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default='auto')
    ap.add_argument('--secs', type=float, default=5.0)
    g = ap.add_mutually_exclusive_group()
    g.add_argument('--pwm', nargs=2, type=int, metavar=('L', 'R'))
    g.add_argument('--vel', nargs=2, type=int, metavar=('L_MMS', 'R_MMS'))
    g.add_argument('--calibrate', action='store_true')
    g.add_argument('--push', action='store_true')
    a = ap.parse_args()

    port = find_port(a.port)
    print(f'Port: {port}')
    s = open_port(port)
    try:
        if a.pwm:
            cmd_drive(s, a.secs, f'P,{a.pwm[0]},{a.pwm[1]}\n'.encode(), f'Raw PWM {a.pwm}')
        elif a.vel:
            cmd_drive(s, a.secs, f'V,{a.vel[0]},{a.vel[1]}\n'.encode(), f'Velocity {a.vel} mm/s')
        elif a.calibrate:
            cmd_calibrate(s)
        elif a.push:
            cmd_push(s)
        else:
            cmd_watch(s, a.secs)
    except KeyboardInterrupt:
        print('\naborted')
    finally:
        try:
            s.write(b'S\n')
        finally:
            s.close()


if __name__ == '__main__':
    main()
