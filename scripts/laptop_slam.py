#!/usr/bin/env python3
"""Build the map on the laptop riding on the robot (docs/laptop_slam.md).

    source /opt/ros/humble/setup.bash
    python3 ~/slam-robot/scripts/laptop_slam.py            # map + record + RViz, follows the app
    python3 ~/slam-robot/scripts/laptop_slam.py --check    # only check the link, clocks, tools, then exit
    python3 ~/slam-robot/scripts/laptop_slam.py --rviz-only   # only RViz (if you closed it), same settings
    options: --pi arpit@169.254.1.2  --no-rviz  --no-record  --quality pi   (Pi's own settings, for a comparison)
             --gpu auto|nvidia|intel|software   which graphics RViz uses (auto: the NVIDIA card if there is one)

Needs on the laptop: Ubuntu 22.04 + ROS 2 Humble, `sudo apt install ros-humble-cartographer-ros rsync`,
this repo (git clone), the Ethernet cable to the Pi (scripts/setup_laptop_link.sh), an SSH key on the Pi
(ssh-copy-id arpit@169.254.1.2). The Pi runs with `slam_host:=laptop` in ROBOT_ARGS (~/robot.env).

What it does, all on its own while it runs:
  * starts Cartographer + the map picture node when the app is in "mapping" (Pi says so on /slam_host),
    restarts them when the Pi restarts or "Continue mapping" picks a map, stops them when the map is frozen
  * records /scan /odom /imu /tf_static + the robot model to ~/bags/<time>: a drive is never lost; rebuild
    it any time with scripts/map_offline.sh build <name> (on this laptop: minutes, not half an hour)
  * copies every map the app saves (Freeze, autosave) to the Pi's ~/maps, keeps a copy in ~/pathik_maps
  * RViz with the map, the scan and the robot
If the cable comes out, the robot loses its map correction and stops by itself after map_tf_timeout (10 s).
"""
import argparse
import os
import shutil
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
PKG = os.path.join(REPO, 'ros2_ws', 'src', 'lidar_robot')
sys.path.insert(0, PKG)
from lidar_robot.laptop_slam import clock_note, decide, parse_status, ready_files, rviz_env, wanted  # noqa: E402

CONFIG = os.path.join(PKG, 'config')
KEEP = os.path.expanduser('~/pathik_maps')
BAGS = os.path.expanduser('~/bags')


def say(msg):
    print(time.strftime('%H:%M:%S'), msg, flush=True)


def run(cmd, timeout=20):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.returncode, (r.stdout + r.stderr).strip()
    except (OSError, subprocess.TimeoutExpired) as e:
        return 1, str(e)


def has_nvidia():
    return bool(shutil.which('nvidia-smi')) and run(['nvidia-smi', '-L'], timeout=5)[0] == 0


def start_rviz(gpu, config):
    """RViz in its own process group with the graphics fix (lidar_robot.laptop_slam.rviz_env); its output
    goes to ~/rviz_log.txt so a freeze or crash can be read afterwards."""
    nv = gpu in ('auto', 'nvidia') and has_nvidia()
    if gpu == 'nvidia' and not nv:
        say('no working NVIDIA driver (nvidia-smi failed): RViz uses the built-in graphics')
    env = rviz_env(os.environ, gpu if gpu != 'intel' else 'none', nv)
    say('start RViz on ' + ('the NVIDIA card' if '__NV_PRIME_RENDER_OFFLOAD' in env else
                           'software drawing' if 'LIBGL_ALWAYS_SOFTWARE' in env else 'the built-in graphics'))
    log = open(os.path.expanduser('~/rviz_log.txt'), 'a')
    cmd = ['rviz2'] + (['-d', config] if os.path.isfile(config) else [])
    return subprocess.Popen(cmd, env=env, start_new_session=True, stdout=log, stderr=subprocess.STDOUT)


def checks(pi):
    """Things that are cheap to check before driving. Returns a list of problems (empty = fine)."""
    bad = []
    if not os.environ.get('ROS_DISTRO'):
        bad.append('ROS is not sourced: source /opt/ros/humble/setup.bash')
    if os.environ.get('ROS_LOCALHOST_ONLY') == '1':
        bad.append('ROS_LOCALHOST_ONLY=1 hides the robot: export ROS_LOCALHOST_ONLY=0')
    dom = os.environ.get('ROS_DOMAIN_ID', '0')
    if dom != '0':
        bad.append(f'ROS_DOMAIN_ID={dom} but the robot uses 0: export ROS_DOMAIN_ID=0')
    for exe in ('cartographer_node', 'cartographer_occupancy_grid_node'):
        if not os.path.exists(f'/opt/ros/{os.environ.get("ROS_DISTRO", "humble")}/lib/cartographer_ros/{exe}'):
            bad.append('Cartographer missing: sudo apt install ros-humble-cartographer-ros')
            break
    if not shutil.which('rsync'):
        bad.append('rsync missing: sudo apt install rsync')
    for f in ('cartographer_odom.lua', 'cartographer_laptop.lua'):
        if not os.path.isfile(os.path.join(CONFIG, f)):
            bad.append(f'{f} not found in {CONFIG}: git pull the repo')
    host = pi.split('@')[-1]
    code, out = run(['ping', '-c', '2', '-W', '1', host], 6)
    if code != 0:
        bad.append(f'cannot reach the Pi at {host}: cable in? bash scripts/setup_laptop_link.sh')
    else:
        code, out = run(['ip', 'route', 'get', host], 5)
        if code == 0 and ' dev wl' in out:
            say(f'note: traffic to the Pi goes over Wi-Fi ({out.split(" dev ")[1].split()[0]}), not the cable')
        code, out = run(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5', pi, 'echo ok'], 10)
        if code != 0:
            bad.append(f'SSH to {pi} needs a password: run once  ssh-copy-id {pi}  (maps are copied with rsync)')
    if shutil.which('nmcli'):
        code, out = run(['nmcli', 'radio', 'wifi'], 5)
        if code == 0 and out.strip() == 'enabled':
            say('tip: laptop Wi-Fi is on. Fine, but if the robot data goes over Wi-Fi it can lag: '
                'nmcli radio wifi off while mapping')
    return bad


class Supervisor:
    def __init__(self, args):
        import rclpy
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        self.rclpy, self.args = rclpy, args
        rclpy.init()
        self.node = rclpy.create_node('laptop_slam')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.status, self.status_t = None, None
        self.offset, self.odom_t = None, None
        self.node.create_subscription(String, 'slam_host', self._on_status, latched)
        self.node.create_subscription(Odometry, 'odom', self._on_odom, qos_profile_sensor_data)
        self.procs, self.key, self.bag = [], None, None
        self.sent = {}
        self.save_dir = None
        self.rviz = None
        self.rviz_starts = 0

    def _on_status(self, msg):
        s = parse_status(msg.data)
        if s:
            self.status, self.status_t = s, time.monotonic()

    def _on_odom(self, msg):
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.offset, self.odom_t = time.time() - st, time.monotonic()

    # ------------------------------------------------------------ processes
    def _spawn(self, cmd, name, log=None):
        say(f'start {name}')
        out = None if self.args.verbose else subprocess.DEVNULL
        if log and not self.args.verbose:
            out = open(log, 'w')
        p = subprocess.Popen(cmd, start_new_session=True, stdout=out,
                             stderr=None if self.args.verbose else subprocess.STDOUT)
        p.log, p.name = log, name
        return p

    @staticmethod
    def _tail(path, n=8):
        try:
            with open(path, errors='replace') as f:
                lines = [ln.rstrip() for ln in f if ln.strip()]
        except OSError:
            return []
        return lines[-n:]

    def start_rviz(self):
        rv = os.path.join(PKG, 'rviz', 'robot.rviz')
        self.rviz = start_rviz(self.args.gpu, rv)
        self.rviz_starts += 1

    @staticmethod
    def _kill(p, wait=8.0):
        if p is None or p.poll() is not None:
            return
        try:
            os.killpg(p.pid, signal.SIGINT)
            p.wait(wait)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

    def _pull_map(self, name):
        """'Continue mapping' needs the saved map on this laptop: take ours, or fetch it from the Pi."""
        local = os.path.join(KEEP, name)
        if os.path.isfile(local):
            return local
        src = f'{self.args.pi}:{self.status.get("map_dir", "~/maps")}/{name}'
        code, out = run(['rsync', '-q', src, local + '.part'], 300)
        if code == 0:
            os.replace(local + '.part', local)
            return local
        say(f'could not fetch {name} from the Pi ({out}): starting a NEW map instead')
        return ''

    def start(self, w):
        lua = 'cartographer_laptop.lua' if self.args.quality == 'laptop' else self.status.get('lua', 'cartographer_odom.lua')
        if self.status.get('lua') == 'cartographer_lidar_only.lua':
            lua = 'cartographer_lidar_only.lua'          # no wheel odometry on the robot: same as the Pi would
        extra = []
        if w['load']:
            path = self._pull_map(w['load'])
            if path:
                extra = [f'-load_state_filename={path}', '-load_frozen_state=false']
                say(f'continuing the saved map {w["load"]}')
        self.save_dir = self.status.get('save_dir') or '/tmp/pathik_maps'
        os.makedirs(self.save_dir, exist_ok=True)
        self.procs = [
            self._spawn(['ros2', 'run', 'cartographer_ros', 'cartographer_node',
                         '-configuration_directory', CONFIG, '-configuration_basename', lua] + extra +
                        ['--ros-args', '-r', 'scan:=/scan', '-r', 'odom:=/odom'], f'Cartographer ({lua})',
                        os.path.expanduser('~/cartographer_log.txt')),
            self._spawn(['ros2', 'run', 'cartographer_ros', 'cartographer_occupancy_grid_node',
                         '-resolution', '0.05', '-publish_period_sec', '2.0'], 'map picture (/map every 2 s)',
                        os.path.expanduser('~/map_picture_log.txt')),
        ]
        if not self.args.no_record:
            os.makedirs(BAGS, exist_ok=True)
            name = time.strftime('laptop_%Y%m%d_%H%M%S')
            out = os.path.join(BAGS, name)
            self._save_urdf(out)
            self.bag = self._spawn(['ros2', 'bag', 'record', '-o', out, '/scan', '/odom', '/imu', '/tf_static'],
                                   f'recording to {out}')
            say(f'rebuild this drive later:  bash {REPO}/scripts/map_offline.sh build {name}')
        self.key = w['key']

    def _save_urdf(self, out):
        code, txt = run(['ros2', 'param', 'get', '/robot_state_publisher', 'robot_description', '--hide-type'], 15)
        if code == 0 and '<robot' in txt:
            self._urdf = (out, txt)               # written next to the bag once it exists
        else:
            self._urdf = None

    def stop(self, why):
        if self.key is None and not self.procs:
            return
        say(f'stop map building ({why})')
        for p in self.procs:
            self._kill(p)
        self._kill(self.bag, 15.0)
        self._write_urdf()
        self.procs, self.key, self.bag = [], None, None

    def _write_urdf(self):
        u = getattr(self, '_urdf', None)
        if u and os.path.isdir(u[0]):
            with open(os.path.join(u[0], 'robot.urdf'), 'w') as f:
                f.write(u[1])
            self._urdf = None

    # ------------------------------------------------------------ map copies to the Pi
    def sync(self):
        if not self.save_dir:
            return
        for name, sig in ready_files(self.save_dir, self.sent, time.time()):
            src = os.path.join(self.save_dir, name)
            dst = f'{self.args.pi}:{self.status.get("map_dir", "~/maps")}/'
            code, out = run(['rsync', '-q', src, dst], 600)   # temp file + rename on the Pi: never half a map
            if code == 0:
                os.makedirs(KEEP, exist_ok=True)
                shutil.copy2(src, os.path.join(KEEP, name))
                self.sent[name] = sig
                say(f'map {name} copied to the Pi (and {KEEP})')
            else:
                say(f'copy of {name} to the Pi failed: {out} (will retry)')

    # ------------------------------------------------------------ main loop
    def loop(self):
        say('waiting for the robot (/slam_host from the Pi dashboard)…')
        last_note, warned_pi = 0.0, False
        t0, hinted = time.monotonic(), False
        while self.rclpy.ok():
            self.rclpy.spin_once(self.node, timeout_sec=0.5)
            now = time.monotonic()
            if self.status is None and not hinted and now - t0 > 30:
                hinted = True
                say('still nothing from the Pi after 30 s: the robot service may be (re)starting or stuck. On the Pi: '
                    'systemctl is-active robot; journalctl -u robot -n 40 --no-pager | grep -i -E "dashboard|error"')
            if self.rviz is not None and self.rviz.poll() is not None:
                code = self.rviz.returncode
                if code != 0 and self.rviz_starts < 4:
                    say(f'RViz closed with an error ({code}): opening it again (log in ~/rviz_log.txt). '
                        'Mapping is not affected.')
                    self.start_rviz()
                else:
                    say('RViz closed (mapping goes on; reopen it: laptop_slam.py --rviz-only)')
                    self.rviz = None
            age = None if self.status_t is None else now - self.status_t
            act = decide(self.status, self.key, age)
            if self.status and self.status.get('host') != 'laptop' and not warned_pi:
                say('the Pi builds the map itself (slam_host is "pi"): add slam_host:=laptop to ROBOT_ARGS in '
                    '~/robot.env on the Pi and sudo systemctl restart robot')
                warned_pi = True
            if getattr(self, 'gave_up', False) and act in ('start', 'restart'):
                pass
            elif act == 'start':
                self.start(wanted(self.status))
            elif act == 'restart':
                self.stop('the robot restarted or a different map was chosen')
                self.start(wanted(self.status))
            elif act == 'stop':
                self.stop('robot not mapping any more' if age is not None and age <= 15 else 'robot silent for 15 s')
            for p in list(self.procs):
                if p.poll() is not None:
                    say(f'{p.name} stopped by itself (exit {p.returncode}). Its last lines:')
                    for ln in self._tail(p.log) if p.log else []:
                        print('    | ' + ln, flush=True)
                    self.crashes = [t for t in getattr(self, 'crashes', []) if now - t < 60] + [now]
                    self.stop('crashed')
                    if len(self.crashes) >= 3:
                        say('crashed 3 times in a minute: NOT restarting. Send the lines above. To try the Pi\'s '
                            'settings instead: Ctrl+C, then run again with --quality pi')
                        self.gave_up = True
                    break
            if self.bag and self.bag.poll() is None:
                self._write_urdf()
            self.sync()
            if now - last_note > 60 and self.key is not None:
                last_note = now
                say(clock_note(self.offset) + ('' if self.odom_t and now - self.odom_t < 2 else ' | NO /odom: check the cable'))

    def shutdown(self):
        self.stop('Ctrl+C')
        self.sync()
        self._kill(self.rviz)
        self.node.destroy_node()
        self.rclpy.try_shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--pi', default=os.environ.get('PATHIK_PI', 'arpit@169.254.1.2'))
    ap.add_argument('--check', action='store_true', help='check everything and exit')
    ap.add_argument('--no-rviz', action='store_true')
    ap.add_argument('--rviz-only', action='store_true', help='only open RViz (same graphics settings), then exit')
    ap.add_argument('--gpu', choices=('auto', 'nvidia', 'intel', 'software'), default='auto',
                    help='graphics for RViz: auto = NVIDIA card if present; software if RViz still freezes')
    ap.add_argument('--no-record', action='store_true')
    ap.add_argument('--quality', choices=('laptop', 'pi'), default='laptop',
                    help='laptop = finer nodes + more loop closure tries (default); pi = the Pi\'s own settings')
    ap.add_argument('--verbose', action='store_true', help='show Cartographer\'s own output')
    args = ap.parse_args()

    if args.rviz_only:
        p = start_rviz(args.gpu, os.path.join(PKG, 'rviz', 'robot.rviz'))
        try:
            p.wait()
        except KeyboardInterrupt:
            pass
        sys.exit(0)

    bad = checks(args.pi)
    for b in bad:
        say('PROBLEM: ' + b)
    if args.check or bad:
        if not bad:
            say('all good: start it without --check')
        sys.exit(1 if bad else 0)

    sup = Supervisor(args)
    if not args.no_rviz and shutil.which('rviz2'):
        sup.start_rviz()
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        sup.loop()
    except (KeyboardInterrupt, RuntimeError):     # Ctrl+C inside rclpy's spin shows up as a RuntimeError
        pass
    finally:
        sup.shutdown()
        say('done')


if __name__ == '__main__':
    main()
