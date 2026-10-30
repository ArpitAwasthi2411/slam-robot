#!/usr/bin/env python3
"""Where does the Pi's CPU go? One report per robot program, plus temperature and slow-down flags.

    python3 ~/slam-robot/tools/cpu_report.py            # 10 s sample while the robot runs
    python3 ~/slam-robot/tools/cpu_report.py --secs 30  # longer = steadier numbers (e.g. while driving)

"% of 1 core": 100 = one core fully busy; a Pi 4 has 4 (400 in total). Reads /proc and /sys only:
no ROS, no root, nothing to install. Run it before and after a change and compare (docs/compute.md).
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'ros2_ws', 'src', 'lidar_robot'))

# cmdline contains -> name shown (first match wins; order matters)
GROUPS = [
    ('cartographer_occupancy_grid_node', 'map drawing (occupancy grid)'),
    ('cartographer_node', 'Cartographer SLAM'),
    ('sllidar_node', 'LiDAR driver'),
    ('esp32_bridge', 'ESP32 bridge'),
    ('lidar_robot/navigator', 'navigator'),
    ('lidar_robot/dashboard', 'app server (dashboard)'),
    ('lidar_robot/vision', 'camera + YOLO (vision)'),
    ('gst-launch-1.0', 'camera pipeline (GStreamer)'),
    ('lidar_robot/voice', 'voice'),
    ('robot_state_publisher', 'robot model (URDF)'),
    ('static_transform_publisher', 'static frames'),
    ('ros2 launch', 'launch'),
    ('ros2-daemon', 'ros2 daemon'),
    ('_ros2_daemon', 'ros2 daemon'),
    ('piper', 'speech (Piper)'),
    ('aplay', 'speech playback'),
]


def ticks_per_s():
    try:
        return os.sysconf('SC_CLK_TCK')
    except (ValueError, OSError):
        return 100


def proc_table():
    """pid -> (cmdline, cpu ticks, threads)."""
    out = {}
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            with open(f'/proc/{pid}/stat') as f:
                st = f.read()
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode(errors='replace').strip()
        except OSError:
            continue
        rest = st[st.rfind(')') + 2:].split()
        try:
            ticks = int(rest[11]) + int(rest[12])        # utime + stime
            threads = int(rest[17])
        except (IndexError, ValueError):
            continue
        out[pid] = (cmd, ticks, threads)
    return out


def group_of(cmd):
    for key, name in GROUPS:
        if key in cmd:
            return name
    return None


def read(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--secs', type=float, default=10.0)
    ap.add_argument('--all', action='store_true', help='also list the busiest other programs')
    a = ap.parse_args()

    from lidar_robot.pihealth import PiHealth
    health = PiHealth(start_thread=False)
    health.measure()                                       # first CPU sample
    t0, before = time.monotonic(), proc_table()
    print(f'measuring for {a.secs:.0f} s …', flush=True)
    time.sleep(a.secs)
    after, dt = proc_table(), time.monotonic() - t0
    h = health.measure()
    hz = ticks_per_s()

    groups, others = {}, []
    for pid, (cmd, ticks, threads) in after.items():
        if pid not in before:
            continue
        pct = 100.0 * (ticks - before[pid][1]) / hz / dt
        name = group_of(cmd)
        if name:
            g = groups.setdefault(name, [0.0, 0, 0])
            g[0] += pct
            g[1] += threads
            g[2] += 1
        elif pct >= 1.0:
            others.append((pct, cmd[:70]))

    total = sum(g[0] for g in groups.values())
    print(f'\n{"program":34s} {"% of 1 core":>12s} {"threads":>8s}')
    print('-' * 56)
    for name, (pct, thr, _) in sorted(groups.items(), key=lambda kv: -kv[1][0]):
        print(f'{name:34s} {pct:12.1f} {thr:8d}')
    print('-' * 56)
    print(f'{"robot programs together":34s} {total:12.1f}   (the Pi has {h["cores"] * 100})')
    if a.all and others:
        print('\nother busy programs:')
        for pct, cmd in sorted(others, reverse=True)[:8]:
            print(f'  {pct:5.1f}  {cmd}')

    freq = read('/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq')
    print(f'\nwhole CPU     {h["cpu_pct"]} % busy, load {h["load1"]} (4 = all cores busy)')
    print(f'temperature   {h["temp_c"]} °C' + (f', CPU clock {int(freq) // 1000} MHz' if freq else ''))
    print(f'throttled     {h["throttled"]}  ' + (', '.join(h['flags']) or 'never slowed down'))
    print(f'verdict       {h["level"].upper()}: {h["text"]}')


if __name__ == '__main__':
    main()
