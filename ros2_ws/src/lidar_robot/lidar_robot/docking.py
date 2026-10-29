"""Front docking: drive nose-first onto the charging contacts, guided by the dock marker (pure Python, tested).

The navigator first drives to the pre-dock pose (predock_m in front of the dock, facing it). Then:

    align    turn on the spot until the marker is in view and roughly centred (sweeps if it isn't seen)
    creep    drive at <= creep_speed (5 cm/s) along the dock line: steering corrects the sideways offset and
             the heading; the ESP32's front ultrasonic stop is relaxed (docking mode) only now
    contact  the last few cm, when the camera is too close to see the marker: straight on, slowly
    charging charging current (INA226) or a battery voltage rise, stable for a moment -> docked
    backoff  no contact / marker lost / took too long -> reverse backoff_m and try again (up to `tries`)

Fails with DOCK_NO_MARKER (never saw it) or DOCK_NO_CONTACT; the navigator hands that to error recovery.

Frames. The marker observation comes from marker_pose.detect() in the camera frame (x right, y down,
z ahead, yaw_deg). marker_in_robot() turns it into the robot frame (x forward, y left) using the camera mount.
"""
import math
from dataclasses import dataclass


@dataclass
class DockParams:
    predock_m: float = 0.60          # pre-dock pose: this far in front of the docked pose
    contact_dist_m: float = 0.10     # base_link to the marker plane when docked (front 0.07 + contacts)
    creep_speed: float = 0.04        # m/s (never above 0.05)
    align_speed: float = 0.25        # rad/s while searching / aligning
    align_ok_deg: float = 6.0        # marker bearing to start creeping
    blind_m: float = 0.20            # closer than this (marker plane) the camera may lose it: go straight
    contact_timeout_s: float = 10.0  # from the start of the blind part
    search_timeout_s: float = 14.0   # no marker for this long while aligning -> back off and retry
    lost_s: float = 1.0              # marker gone this long while creeping (not blind) -> back off
    backoff_m: float = 0.25
    tries: int = 3
    volt_rise: float = 0.15          # V above the creep-start voltage counts as contact (no current sensor)
    contact_hold_s: float = 1.0      # contact must last this long
    cam_x: float = 0.05              # camera position in base_link (m) and height (m)
    cam_y: float = 0.0
    cam_z: float = 0.20
    lookahead_m: float = 0.20


def marker_in_robot(obs, p):
    """Camera-frame observation -> (mx, my, nx, ny): marker centre and its normal (towards the robot) in the
    robot frame (x forward, y left)."""
    mx = p.cam_x + obs['z_m']
    my = p.cam_y - obs['x_m']
    a = math.radians(obs.get('yaw_deg', 0.0))
    # normal in the camera frame: (sin a, -cos a) in (x right, z ahead); robot frame: (z, -x)
    nx, ny = -math.cos(a), -math.sin(a)
    return mx, my, nx, ny


def dock_errors(obs, p):
    """along: base_link distance in front of the docked pose (m); lat: sideways offset from the dock line
    (m, + = the robot is to the RIGHT of the line, seen facing the dock); head: heading error (rad, + = turn left to face the dock squarely)."""
    mx, my, nx, ny = marker_in_robot(obs, p)
    rx, ry = -mx, -my                      # robot (origin) seen from the marker
    along = rx * nx + ry * ny - p.contact_dist_m
    lat = nx * ry - ny * rx
    head = math.atan2(-ny, -nx)            # direction of -normal (towards the dock) in the robot frame
    return along, lat, head


def fuse_yaw(obs, robot_yaw, dock_yaw, trust_below_m=0.45, max_disagree_deg=20.0):
    """The marker's own yaw is noisy until it is big in the picture: far away (or when it disagrees a lot
    with the map) use the dock heading from the map (robot yaw from SLAM vs the saved dock yaw) instead."""
    na = dock_yaw + math.pi - robot_yaw            # marker normal in the robot frame
    map_yaw = math.degrees(math.atan2(-math.sin(na), -math.cos(na)))
    out = dict(obs)
    d = (obs.get('yaw_deg', map_yaw) - map_yaw + 180.0) % 360.0 - 180.0
    if obs.get('z_m', 9) > trust_below_m or abs(d) > max_disagree_deg:
        out['yaw_deg'] = map_yaw
        out['yaw_src'] = 'map'
    else:
        out['yaw_src'] = 'marker'
    return out


def synth_observation(robot, marker, p, noise=None):
    """Simulators/tests: what the camera would report. robot = (x, y, yaw); marker = (x, y, yaw of its normal).
    Returns None when the marker is out of the camera's view (behind, too far, > 50 deg off axis)."""
    rx, ry, ryaw = robot
    c, s = math.cos(ryaw), math.sin(ryaw)
    cx, cy = rx + c * p.cam_x - s * p.cam_y, ry + s * p.cam_x + c * p.cam_y
    dx, dy = marker[0] - cx, marker[1] - cy
    fwd, left = c * dx + s * dy, -s * dx + c * dy
    if fwd < 0.06 or fwd > 3.0 or abs(math.atan2(left, fwd)) > math.radians(50):
        return None
    na = marker[2] - ryaw                  # marker normal in the robot frame
    nfx, nly = math.cos(na), math.sin(na)
    if nfx > -0.2:                         # seeing the back / edge of the marker
        return None
    obs = {'found': True, 'x_m': -left, 'y_m': 0.0, 'z_m': fwd,
           'yaw_deg': math.degrees(math.atan2(-nly, -nfx))}
    if noise:
        obs = noise(obs)
    return obs


class FrontDocker:
    def __init__(self, params=None):
        self.p = params or DockParams()
        self.phase = 'off'
        self.try_n = 0
        self.message = ''
        self.marker = None
        self.result = None           # None | 'docked' | ('fail', code, text)
        self.ever_seen = False

    # ------------------------------------------------------------ control
    def start(self, now):
        self.try_n = 1
        self.result = None
        self.ever_seen = False
        self._enter('align', now)

    def stop(self):
        self.phase = 'off'

    def _enter(self, phase, now):
        self.phase = phase
        self._t_phase = now
        if phase == 'align':
            self._search_dir = 1.0
            self._search_t = now
            self._search_len = 2.0          # first half-sweep 2 s, then full sweeps of 4 s (+-25 deg)
        if phase == 'backoff':
            self._back_from = None

    def mode_on(self):
        """ESP32 docking mode (relaxed front ultrasonic stop) only while creeping in / touching."""
        return self.phase in ('creep', 'contact')

    def step(self, now, obs, odom, battery, rear_clear=True):
        """obs: marker observation (camera frame) or None; odom: (x, y, yaw) any fixed frame (for distances);
        battery: {'charging': bool, 'v': volts}. Returns (v, w)."""
        p = self.p
        seen = bool(obs and obs.get('found'))
        if seen:
            self.ever_seen = True
            self.marker = {k: obs.get(k) for k in ('x_m', 'y_m', 'z_m', 'yaw_deg')}
            self.marker['found'] = True
            self._last_seen = now
            self._last_err = dock_errors(obs, p)
            self._last_odom = odom
        elif self.marker:
            self.marker['found'] = False
        ph = self.phase

        if ph == 'align':
            if seen:
                along, lat, head = self._last_err
                bearing = math.atan2(-obs['x_m'], obs['z_m'] + p.cam_x)
                if abs(bearing) < math.radians(p.align_ok_deg):
                    self._v_start = battery.get('v')
                    self._enter('creep', now)
                    self.message = f'marker {obs["z_m"]:.2f} m ahead: creeping in'
                    return 0.0, 0.0
                self.message = 'turning to the marker'
                return 0.0, max(-p.align_speed, min(p.align_speed, 1.5 * bearing))
            if now - self._t_phase > p.search_timeout_s:
                return self._retry(now, 'DOCK_NO_MARKER' if not self.ever_seen else 'DOCK_NO_CONTACT',
                                   'marker not found in front of the dock')
            # sweep +-25 deg around the start heading
            if now - self._search_t > self._search_len:
                self._search_dir = -self._search_dir
                self._search_t = now
                self._search_len = 4.0
            self.message = 'looking for the dock marker'
            return 0.0, 0.5 * p.align_speed * self._search_dir

        if ph == 'creep':
            if seen:
                along, lat, head = self._last_err
                if along <= p.blind_m:
                    self._contact_from = odom
                    # the dock line's heading in the odometry frame: held while the camera is too close to see
                    self._line_yaw = odom[2] + head if odom else None
                    self._line_lat = lat
                    self._enter('contact', now)
                    return 0.0, 0.0
                v = max(0.02, min(p.creep_speed, 0.05, 0.25 * along))
                # pure pursuit to a point on the dock line, lookahead ahead of the robot
                la = max(0.05, min(p.lookahead_m, along))
                # the dock line point beside the robot is lat * (left of travel); then la along the line
                tx = la * math.cos(head) - lat * math.sin(head)
                ty = la * math.sin(head) + lat * math.cos(head)
                d2 = tx * tx + ty * ty
                w = max(-0.4, min(0.4, 2.0 * v * ty / max(d2, 1e-3)))
                self.message = f'creeping in: {along * 100:.0f} cm to go, {lat * 100:+.1f} cm off the line'
                return v, w
            if now - getattr(self, '_last_seen', now) > p.lost_s:
                return self._retry(now, 'DOCK_NO_CONTACT', 'lost the marker while creeping in')
            return 0.0, 0.0                                  # brief dropout: wait for the next frame

        if ph == 'contact':
            touching = battery.get('charging') or (
                self._v_start is not None and battery.get('v') is not None and battery['v'] - self._v_start >= p.volt_rise)
            if touching:
                if getattr(self, '_touch_t', None) is None:
                    self._touch_t = now
                if now - self._touch_t >= p.contact_hold_s:
                    self._enter('charging', now)
                    self.result = 'docked'
                    self.message = 'docked: charging'
                return 0.0, 0.0
            self._touch_t = None
            moved = _dist(odom, self._contact_from)
            if now - self._t_phase > p.contact_timeout_s or moved > p.blind_m + 0.06:
                return self._retry(now, 'DOCK_NO_CONTACT', 'reached the dock but no charging current')
            self.message = 'touching the contacts…'
            w = 0.0
            if getattr(self, '_line_yaw', None) is not None and odom:
                err = math.atan2(math.sin(self._line_yaw - odom[2]), math.cos(self._line_yaw - odom[2]))
                w = max(-0.3, min(0.3, 2.0 * err))
            return min(0.03, p.creep_speed), w

        if ph == 'backoff':
            if self._back_from is None:
                self._back_from = odom
            if _dist(odom, self._back_from) >= p.backoff_m:
                self._enter('align', now)
                return 0.0, 0.0
            if not rear_clear:
                self.message = 'backing off: something behind me, waiting'
                return 0.0, 0.0
            self.message = f'backing off to try again ({self.try_n} of {p.tries})'
            return -0.05, 0.0
        return 0.0, 0.0

    def _retry(self, now, code, text):
        if self.try_n >= self.p.tries:
            self.phase = 'failed'
            self.result = ('fail', code, f'{text} after {self.try_n} tries')
            self.message = self.result[2]
            return 0.0, 0.0
        self.try_n += 1
        self._last_step = self.phase
        self._enter('backoff', now)
        self.message = f'{text}: backing off to try again'
        return 0.0, 0.0

    def status(self):
        if self.phase == 'off':
            return None
        last = {'align': 1, 'creep': 2, 'contact': 3}.get(getattr(self, '_last_step', ''), None)
        return {'phase': self.phase, 'try': self.try_n, 'tries': self.p.tries, 'message': self.message,
                'marker': self.marker, 'last_step': last}


def _dist(a, b):
    if a is None or b is None:
        return 0.0
    return math.hypot(a[0] - b[0], a[1] - b[1])
