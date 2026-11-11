"""Nav2 behind Pathik's follower interface (pure Python, unit-tested; the ROS side is in navigator.py).

    nav_backend: nav2      (config/robot_params.yaml, or launch nav:=nav2)

What changes: the route and the wheel speeds come from Nav2, the standard ROS 2 navigation stack
(planner server + controller server + behaviour tree), instead of Pathik's own A* and pure pursuit.
What stays: missions, the queue, waits, docking, error recovery, "why did you stop?", the app. The
core still asks its follower for (v, w) twenty times a second; this follower answers with Nav2's
command after Pathik's own safety checks:

    core --set_path(goal)--> Nav2Follower --NavigateToPose--> Nav2 (plans on its costmap)
    Nav2 --cmd_vel_nav2--> Nav2Follower.step(): front clearance (LiDAR + ultrasonic), body check,
                           speed limits  --(v, w)--> core --> /cmd_vel --> ESP32

So an obstacle only the ultrasonics see still stops the robot, and Nav2 can never command more than
the limits set in the app. When Nav2 gives up (no route, stuck), the core is told BLOCKED and its
usual replan / recovery ladder runs.

mode 'navigate': Nav2 plans and drives (the standard stack).
mode 'follow':   Pathik's route (which knows the ultrasonic-only obstacles) is handed to Nav2's
                 controller with FollowPath; Nav2 only drives.
"""
import math

from lidar_robot.follower import PathFollower, _clamp

MODES = ('navigate', 'follow')


class Nav2Follower(PathFollower):
    def __init__(self, params, send_goal, cancel, clock, mode='navigate', cmd_timeout=0.6, start_timeout=10.0):
        super().__init__(params)
        self._send_goal = send_goal          # (seq, mode, path_points, (x, y, yaw)) -> None
        self._cancel = cancel                # (seq) -> None
        self.clock = clock
        self.mode = mode if mode in MODES else 'navigate'
        self.cmd_timeout = cmd_timeout       # s: an older Nav2 command is not driven
        self.start_timeout = start_timeout   # s without any answer from Nav2 = it is not running
        self.seq = 0                         # one number per goal: late answers to old goals are ignored
        self._active = False
        self._result = None                  # None | ('ok', '') | ('failed', reason)
        self._cmd = (0.0, 0.0, -1e9)
        self._sent_at = 0.0
        self._heard = False
        self._goal = None
        self.nav2_note = ''                  # last thing Nav2 said (for the log and "why did you stop?")
        self.stats = {'goals': 0, 'arrived': 0, 'failed': 0, 'gated': 0}

    # ---------------------------------------------------------------- from the core
    def set_path(self, pts, final_yaw=None, tight=False):
        super().set_path(pts, final_yaw, tight)
        if self._active:
            self._cancel(self.seq)
        if not self.path:
            self._active = False
            return
        gx, gy = self.path[-1]
        if final_yaw is None:                # arrive facing the way the route ends
            ax, ay = self.path[-2] if len(self.path) > 1 else (gx - 1.0, gy)
            final_yaw_goal = math.atan2(gy - ay, gx - ax)
        else:
            final_yaw_goal = final_yaw
        self.seq += 1
        self._active, self._result, self._heard = True, None, False
        self._cmd = (0.0, 0.0, -1e9)
        self._sent_at = self.clock()
        self._goal = (gx, gy, final_yaw_goal)
        self.stats['goals'] += 1
        self._send_goal(self.seq, self.mode, list(self.path), self._goal)

    def clear(self):
        if self._active:
            self._cancel(self.seq)
        self._active = False
        self._result = None
        super().clear()

    # ---------------------------------------------------------------- from Nav2 (the node calls these)
    def on_cmd(self, v, w):
        if math.isfinite(v) and math.isfinite(w):
            self._cmd = (v, w, self.clock())
            self._heard = True

    def on_accepted(self, seq, accepted, why=''):
        if seq != self.seq or not self._active:
            return
        self._heard = True
        if not accepted:
            self._result = ('failed', why or 'Nav2 refused the goal')

    def on_result(self, seq, ok, why=''):
        if seq != self.seq or not self._active:
            return                           # an answer to a goal we already replaced or cancelled
        self._heard = True
        self._result = ('ok', '') if ok else ('failed', why or 'Nav2 could not reach the goal')

    def on_plan(self, pts):
        """Nav2's own route (navigate mode), so the app draws what the robot really follows."""
        if self._active and self.mode == 'navigate' and len(pts) >= 2 and self._goal:
            self.path = [(float(x), float(y)) for x, y in pts]
            self.idx = 0

    # ---------------------------------------------------------------- 20 times a second
    def step(self, pose, front_clearance, dt, guard=None):
        p = self.p
        if not self.path or not self._active:
            return 0.0, 0.0, None
        now = self.clock()
        self.blocked = False
        if self._result is not None:
            kind, why = self._result
            self._active = False
            self._v = self._w = 0.0
            if kind == 'ok':
                self.stats['arrived'] += 1
                return 0.0, 0.0, 'ARRIVED'
            self.stats['failed'] += 1
            self.nav2_note, self.why_blocked = why, 'nav2'
            return 0.0, 0.0, 'BLOCKED'       # the core replans, then its recovery ladder takes over
        if not self._heard and now - self._sent_at > self.start_timeout:
            self._cancel(self.seq)
            self._active = False
            self.stats['failed'] += 1
            self.nav2_note, self.why_blocked = 'Nav2 did not answer: is it running? (launch nav:=nav2)', 'nav2'
            return 0.0, 0.0, 'BLOCKED'

        self._advance(pose[0], pose[1])      # keeps "distance left" and the drawn route up to date
        v, w, t = self._cmd
        if now - t > self.cmd_timeout:
            v = w = 0.0                      # Nav2 is thinking (planning, recovery): stand still
        v = _clamp(v, -p.max_linear, p.max_linear)
        w = _clamp(w, -p.max_angular, p.max_angular)

        # Pathik's own checks on top of Nav2's costmap: the ultrasonics and the body outline
        if v > 0.0:
            if front_clearance < p.stop_distance:
                v = w = 0.0
                self.blocked, self.why_blocked = True, 'front'
            elif front_clearance < p.slow_distance:
                frac = (front_clearance - p.stop_distance) / (p.slow_distance - p.stop_distance)
                v = max(p.min_linear, v * frac)
        if guard is not None and not self.blocked and (v or w) and not guard.motion_clear(v, w):
            if guard.motion_clear(v, w * 0.4):
                w *= 0.4
            elif v > 0.0 and guard.motion_clear(p.min_linear, 0.0):
                v, w = p.min_linear, 0.0
            else:
                v = w = 0.0
                self.blocked, self.why_blocked = True, 'body'
        if self.blocked:
            self.stats['gated'] += 1
            self._blocked_for += dt
            if self._blocked_for > p.blocked_after:
                self._blocked_for = 0.0
                self._cancel(self.seq)       # Nav2 does not see what stopped us: let the core route around it
                self._active = False
                return 0.0, 0.0, 'BLOCKED'
        else:
            self._blocked_for = 0.0
        self._v, self._w = v, w
        return v, w, None
