"""NavigatorCore: places + A* planning + path following + mission queue + recovery (no ROS).

Used by the ROS node (navigator.py) and by the offline simulator (tools/dashboard_sim.py), so
everything the robot does can be tested on a laptop.

JSON request API (handle):
  {"type": "goto_place", "name": "HOD office", "priority": 1}
  {"type": "goto_pose", "x": 1, "y": 2, "yaw": 0, "label": "clicked point", "priority": 1}
  {"type": "command", "text": "urgent: take this to the HOD office then lab 3"}
  {"type": "preview", "name": "Lab 3"}   or   {"type": "preview", "x": 1, "y": 2}
  {"type": "cancel"}  /  {"type": "cancel", "id": 7}
  {"type": "resume"}                     after an e-stop: missions stay on HOLD until this
  {"type": "confirm_localization"}       localization mode: the scan matches the map, allow driving
  {"type": "go_home"}
  {"type": "add_place", "name": "Lab 3", "x": .., "y": .., "yaw": .., "aliases": [..]}  ("here": true = robot pose)
  {"type": "delete_place", "name": "Lab 3"}
  {"type": "set_home", "here": true}  or with x/y/yaw
  {"type": "search", "q": "hod"}
  {"type": "confirm"}                    answer a "wait for OK" stop (state CONFIRM)
  {"type": "explain", "speak": true}     "why did you stop?" in plain words
  {"type": "find", "q": "3D printer"}    which room has it (room-level semantics)
  {"type": "add_room", "name": "Lab 3", "polygon": [[x, y], ...], "contents": [..], "aliases": [..]}
  {"type": "update_room", "name": "Lab 3", "contents": [..]}  /  {"type": "delete_room", "name": ..}
  {"type": "goto_room", "name": "Lab 3"}  or  {"type": "goto_room", "thing": "3D printer"}
  {"type": "set_dock", "here": true}     robot is sitting ON the dock, contacts touching (front docking: facing it)
  {"type": "go_dock"}                    drive in front of the dock, then creep nose-first onto the contacts guided
                                         by the dock marker (docking.py); dock_front=False = old reverse docking
  {"type": "schedule_add", "name": "Patrol", "at": "10:00", "days": [0,1,2,3,4], "action": {...}}
  {"type": "schedule_delete", "id": 1} / {"type": "schedule_enable", "id": 1, "on": false}
  {"type": "mission_log", "n": 50}       evaluation records + summary stats
  {"type": "goto_poses", "waypoints": [{"x":..,"y":..,"label":..}], "repeat": -1..99}   route / patrol (-1)
  {"type": "check_path", "points": [[x, y], ...]}  /  {"type": "follow_path", "points": [...], "repeat": 0}
  {"type": "explore_start", "speed": .18, "turn_speed": .45, "scan_spin": true}  /  {"type": "explore_stop"}
  {"type": "get_tune"}  /  {"type": "tune", "follower": {...}, "save": true}     path-follower tuning
  {"type": "find", ...}, {"type": "schedule_list"}, {"type": "search", "q": ...}
  {"type": "recovery_answer", "id": 3, "choice": "retry|wait|skip|home|dock|cancel"}   reply to "ask a person"

Error recovery (recovery.py): when a mission step fails with a recoverable code, the robot stops (state
RECOVERING) and the LLM (or the rules) picks one action from a fixed menu; see status()['recovery'].

Failure reason codes (for logs, the dashboard and LLM error recovery):
  NO_MAP, NO_PATH, GOAL_IN_OBSTACLE, OUTSIDE_MAP, START_BLOCKED, BLOCKED, POSE_LOST, ESTOP, UNKNOWN_PLACE,
  DOCK_NO_CONTACT, DOCK_NO_MARKER
"""
import math
import os
import time

from lidar_robot import drawn_path
from lidar_robot.docking import DockParams, FrontDocker, fuse_yaw, synth_observation
from lidar_robot.battery import BatteryMonitor
from lidar_robot.explain import explain
from lidar_robot.footprint import Footprint, FootprintGuard, check_path
from lidar_robot.missionlog import MissionLog
from lidar_robot.schedules import ScheduleStore
from lidar_robot.commands import CommandParser
from lidar_robot.explorer import ExploreParams, find_frontiers, choose_goal, mapped_area
from lidar_robot.follower import FollowerParams, PathFollower
from lidar_robot.missions import Mission, MissionQueue, Stop
from lidar_robot.planner import Planner, PlannerParams, path_length
from lidar_robot.recovery import RECOVERABLE, Recovery, describe, rule_decide, validate
from lidar_robot.tuning import (TuningStore, apply_follower, follower_values, ranges_for_app,
                                FOLLOWER_RANGES)

IDLE, PLANNING, DRIVING, DWELL, WAITING = 'IDLE', 'PLANNING', 'DRIVING', 'DWELL', 'WAITING'
SCANNING = 'SCANNING'          # exploration: turning slowly on the spot so the LiDAR sees every wall
CONFIRM = 'CONFIRM'            # at a stop, waiting for someone to press OK
DOCKING, UNDOCKING, CHARGING = 'DOCKING', 'UNDOCKING', 'CHARGING'
RECOVERING = 'RECOVERING'      # a step failed: stopped while the recovery planner decides / acts
BATTERY_SOURCES = ('battery', 'auto-home', 'explore-return', 'recovery')
ANSWERS = {'retry': ('wait_retry', {'wait_s': 3}, 'Try again'), 'wait': ('wait_retry', {'wait_s': 60}, 'Wait 1 min, then try'),
           'skip': ('skip_stop', {}, 'Skip this stop'), 'home': ('go_home', {}, 'Go home'),
           'dock': ('go_dock', {}, 'Go and charge'), 'cancel': (None, {}, 'Cancel the mission')}


def _reason_code(planner_error):
    e = planner_error or ''
    if 'no map' in e:
        return 'NO_MAP'
    if 'outside' in e:
        return 'OUTSIDE_MAP'
    if 'goal is inside' in e:
        return 'GOAL_IN_OBSTACLE'
    if 'too close to an obstacle to plan' in e:
        return 'START_BLOCKED'
    return 'NO_PATH'


class NavigatorCore:
    def __init__(self, places, planner_params=None, follower_params=None, parser=None,
                 dwell_s=3.0, auto_return_s=0.0, max_replans=3, plan_retry_s=2.0, max_plan_attempts=3,
                 clock=time.monotonic, tuning_path=None, footprint=None, data_dir=None, battery=None,
                 localtime=time.localtime, confirm_timeout_s=300.0, dock_offset=0.55, recovery=None,
                 dock_front=True, dock_params=None, dock_without_marker=False):
        self.places = places
        self.planner = Planner(planner_params or PlannerParams())
        self.follower = PathFollower(follower_params or FollowerParams())
        self.tuning = TuningStore(tuning_path)           # saved Tuning Lab values override the YAML
        apply_follower(self.follower.p, self.tuning.data['follower'])
        self.parser = parser or CommandParser(places)
        self.queue = MissionQueue()
        self.dwell_s = dwell_s
        self.auto_return_s = auto_return_s           # 0 = never go home automatically
        self.max_replans = max_replans
        self.plan_retry_s = plan_retry_s
        self.max_plan_attempts = max_plan_attempts
        self.clock = clock
        self.state = IDLE
        self.message = 'ready'
        self.scan_world = []
        self.preview = None
        self.events = []
        self.estopped = False
        self.hold = False               # set by e-stop; only an explicit "resume" clears it
        self.localized = True           # False in localization mode until a human confirms the pose
        self.last_failure = None        # {'code', 'text', 'mission'}: input for LLM error recovery
        self._replans = 0
        self._dwell_until = 0.0
        self._retry_at = 0.0
        self._idle_since = clock()
        self._pose = None
        self._pose_lost_for = 0.0
        self.pose_lost_fail_s = 30.0
        self.say_queue = []                   # (key, text): the ROS node speaks these (speech.Speaker)
        self._blocked_since = None          # wait this long for the position to come back before giving up
        self._scan = None
        self.grid = None                # latest GridMap (for exploration)
        self.explore_params = ExploreParams()
        self.explore = {'active': False, 'state': 'off'}
        # body shape, battery, schedules, evaluation log (v2.4)
        self.footprint = footprint or Footprint()
        self.body_checks = True                  # False = old behaviour (circle robot), for comparisons
        self.scan_base = []                      # live scan + ultrasonic points in base_link
        self.tight_spots = []                    # where the current route barely fits the body
        self.battery = battery or BatteryMonitor(clock=clock)
        self._bat_mode = None                    # None | 'low' | 'critical' (already acted on)
        data_dir = data_dir or (os.path.dirname(tuning_path) if tuning_path else None)
        self.schedules = ScheduleStore(os.path.join(data_dir, 'schedules.json') if data_dir else None,
                                       localtime=localtime)
        self.mlog = MissionLog(os.path.join(data_dir, 'missions.jsonl') if data_dir else None)
        self._next_sched = 0.0
        self.confirm_timeout_s = confirm_timeout_s
        self._confirm_until = 0.0
        self.dock_offset = dock_offset           # m in front of the dock where the reverse starts
        self.docked = False
        self._dock = None
        self.dock_front = dock_front                 # front docking with the marker (False: reverse, v2.4)
        self.docker = FrontDocker(dock_params or DockParams())
        self.dock_without_marker = dock_without_marker   # no camera: use the map pose of the dock instead
        self._marker = (None, -1e9)
        self.localtime = localtime
        self.rec = recovery or Recovery(mode='off')      # error recovery (off unless the node/sim turns it on)
        self.rec.clock = clock

    # ---------------------------------------------------------------- inputs
    def set_map(self, grid):
        self.planner.set_map(grid)
        self.grid = grid

    def set_grid(self, grid):
        """Latest raw map for the explorer (the planner is swapped in separately)."""
        self.grid = grid

    def swap_planner(self, planner):
        """Install a planner whose map was prepared in another thread (atomic reference swap)."""
        self.planner = planner

    def update_scan_world(self, pts):
        self.scan_world = pts

    def update_scan_base(self, pts):
        """Scan / ultrasonic points in base_link (robot frame) for the body-shape checks."""
        self.scan_base = pts

    def set_marker(self, obs):
        """Latest dock marker observation from the camera (marker_pose.detect format), any time."""
        self._marker = (obs, self.clock())

    def dock_mode(self):
        """True while the ESP32 should relax its front ultrasonic stop (creeping onto the dock)."""
        return self.dock_front and self.state == DOCKING and self.docker.mode_on()

    def set_battery(self, volts, amps=None, src='ina226'):
        self.battery.update(volts, amps, src)

    def note_bump(self):
        """A bumper switch (or the simulator) reports contact: count it and find another route."""
        self.mlog.bump(self.queue.active, 'bumps')
        self._log('bump detected: stopping and replanning', 'warn')
        if self.queue.active and self.state == DRIVING:
            self.follower.clear()
            self._replans += 1
            self._set(PLANNING, 'bumped into something, replanning')

    def set_estop(self, on):
        if on and not self.estopped:
            self.mlog.bump(self.queue.active, 'estops')
            self.estopped = True
            self.hold = True
            if self.rec.busy():
                self.rec.cancel('e-stop')
            if self.state == RECOVERING:
                self.state = WAITING
            if self.queue.active:
                self.queue.preempt()
                self.queue.queue[0].message = 'paused by e-stop' if self.queue.queue else ''
            self.follower.clear()
            self._set(WAITING, 'E-STOP engaged: missions paused')
            self._log('E-stop engaged, missions paused', 'warn')
        elif not on and self.estopped:
            self.estopped = False
            self._set(WAITING, 'E-stop released: press Resume to continue missions')
            self._log('E-stop released (missions on hold until Resume)')

    # ---------------------------------------------------------------- request API
    def handle(self, req, pose=None):
        if not isinstance(req, dict):
            return {'ok': False, 'error': 'request must be a JSON object'}
        t = req.get('type')
        self._req_by = str(req.get('by') or '')[:32]     # app login of whoever sent it (mission log)
        pose = pose or self._pose
        try:
            if t == 'goto_place':
                p = self._resolve(req.get('name', ''))
                return self._enqueue([self._stop_from_place(p)], req.get('priority', 1), 'dashboard', p['name'])
            if t == 'goto_pose':
                for k in ('x', 'y'):
                    if not math.isfinite(float(req[k])):
                        raise ValueError(f'{k} must be a finite number')
                st = Stop(req.get('label') or 'map point', float(req['x']), float(req['y']),
                          None if req.get('yaw') is None else float(req['yaw']))
                if req.get('replace'):
                    self._cancel_all('new goal')
                return self._enqueue([st], req.get('priority', 1), req.get('source', 'dashboard'), st.label)
            if t == 'check_path':
                path = drawn_path.prepare(req.get('points') or [])
                bad = drawn_path.check(self.planner, path, self.planner.p.robot_radius) if self.planner.has_map() else []
                length = path_length(path)
                return {'ok': True, 'path': [[round(x, 3), round(y, 3)] for x, y in path[::2]] + [list(path[-1])],
                        'length': round(length, 2), 'blocked': bad,
                        'eta_s': round(length / max(0.05, self.follower.p.max_linear * 0.7))}
            if t == 'follow_path':
                path = drawn_path.prepare(req.get('points') or [])
                if not self.planner.has_map():
                    raise ValueError('no map yet')
                bad = drawn_path.check(self.planner, path, self.planner.p.robot_radius)
                if bad and not req.get('allow_close'):
                    raise ValueError(f'the path passes too close to a wall at {bad[0][0]:.1f} m '
                                     '(shown in red): redraw that part')
                repeat = int(req.get('repeat', 0))
                if repeat < -1 or repeat > 99:
                    raise ValueError('repeat must be -1 (loop) or 0..99')
                if req.get('replace'):
                    self._cancel_all('new drawn path')
                end = path[-1]
                yaw = req.get('yaw')
                st = Stop(req.get('label') or 'drawn path', end[0], end[1], None if yaw is None else float(yaw), path=path)
                r = self._enqueue([st], req.get('priority', 1), req.get('source', 'dashboard'),
                                  f'drawn path {path_length(path):.1f} m')
                m = self._mission_by_id(r['mission']['id'])
                m.repeat = repeat
                r['mission'] = m.summary()
                return r
            if t == 'goto_poses':
                pts = req.get('waypoints') or []
                if not isinstance(pts, list) or not 1 <= len(pts) <= 30:
                    raise ValueError('give 1 to 30 waypoints')
                stops = [Stop(str(w.get('label') or f'point {i + 1}'), float(w['x']), float(w['y']),
                              None if w.get('yaw') is None else float(w['yaw'])) for i, w in enumerate(pts)]
                repeat = int(req.get('repeat', 0))
                if repeat < -1 or repeat > 99:
                    raise ValueError('repeat must be -1 (patrol) or 0..99')
                if req.get('replace'):
                    self._cancel_all('new route')
                text = req.get('label') or (f'patrol of {len(stops)} points' if repeat == -1 else f'route of {len(stops)} points')
                r = self._enqueue(stops, req.get('priority', 1), req.get('source', 'dashboard'), text)
                m = self._mission_by_id(r['mission']['id'])
                m.repeat = repeat
                m.skip_unreachable = len(stops) > 1
                r['mission'] = m.summary()
                return r
            if t == 'command':
                return self._command(req.get('text', ''), dry_run=bool(req.get('dry_run')))
            if t == 'preview':
                return self._preview(req, pose)
            if t == 'cancel':
                if req.get('id') is None and self.explore.get('active'):
                    self._explore_stop('canceled')
                n = self.queue.cancel(req.get('id'))
                if not self.queue.active:
                    if self.rec.busy():
                        self.rec.cancel('canceled')
                    self.follower.clear()
                    self._set(IDLE, 'canceled')
                self._log(f'canceled {n} mission(s)')
                return {'ok': True, 'canceled': n}
            if t == 'resume':
                if self.estopped:
                    raise ValueError('release the e-stop first')
                self.hold = False
                self._set(IDLE, 'resumed')
                self._log('missions resumed')
                return {'ok': True}
            if t == 'confirm_localization':
                self.localized = True
                self._set(IDLE, 'position confirmed')
                self._log('operator confirmed the robot position on the map')
                return {'ok': True}
            if t == 'go_home':
                if not self.places.home:
                    raise ValueError('home is not set (long-press the map → Set home here)')
                if req.get('replace'):
                    self._cancel_all('going home')
                return self._go_home(req.get('priority', 1), 'dashboard')
            if t == 'add_place':
                if req.get('here'):
                    if not pose:
                        raise ValueError('robot pose unknown')
                    x, y, yaw = pose
                else:
                    x, y, yaw = float(req['x']), float(req['y']), float(req.get('yaw', 0.0))
                rec = self.places.add(req['name'], x, y, yaw, req.get('aliases', []), req.get('note', ''))
                self._log(f'place saved: {rec["name"]}')
                return {'ok': True, 'place': rec}
            if t == 'delete_place':
                self.places.remove(req['name'])
                self._log(f'place deleted: {req["name"]}')
                return {'ok': True}
            if t == 'set_home':
                if req.get('here'):
                    if not pose:
                        raise ValueError('robot pose unknown')
                    x, y, yaw = pose
                else:
                    x, y, yaw = float(req['x']), float(req['y']), float(req.get('yaw', 0.0))
                self.places.set_home(x, y, yaw)
                self._log('home position set')
                return {'ok': True, 'home': self.places.home}
            if t == 'explore_start':
                return self._explore_start(req, pose)
            if t == 'explore_stop':
                return self._explore_stop('stopped by user')
            if t == 'get_tune':
                return {'ok': True, 'follower': follower_values(self.follower.p),
                        'ranges': ranges_for_app(FOLLOWER_RANGES)}
            if t == 'tune':
                applied = apply_follower(self.follower.p, req.get('follower', {}))
                if req.get('save'):
                    self.tuning.save_follower(follower_values(self.follower.p))
                self._log(f'follower tuning {"saved" if req.get("save") else "changed"}: '
                          + ', '.join(f'{k}={v:g}' for k, v in applied.items()))
                return {'ok': True, 'follower': follower_values(self.follower.p), 'saved': bool(req.get('save'))}
            if t == 'confirm':
                if self.state != CONFIRM:
                    return {'ok': False, 'error': 'nothing is waiting for OK'}
                self._log(f'OK received at {self.queue.active.current.label}')
                self.say_queue.append(('confirm', 'Thank you.'))
                self._set(DWELL, 'confirmed')
                self._dwell_until = self.clock()
                return {'ok': True}
            if t == 'explain':
                r = explain(self)
                if req.get('speak'):
                    self.say_queue.append(('explain', r['answer']))
                return {'ok': True, **r}
            if t == 'find':
                return self._find(req.get('q', ''))
            if t == 'add_room':
                rec = self.places.add_room(req['name'], req['polygon'], req.get('contents', []), req.get('aliases', []))
                self._log(f'room saved: {rec["name"]} ({len(rec["contents"])} things)')
                return {'ok': True, 'room': rec}
            if t == 'update_room':
                rec = self.places.update_room(req['name'], req.get('contents'), req.get('aliases'), req.get('new_name'))
                return {'ok': True, 'room': rec}
            if t == 'delete_room':
                self.places.remove_room(req['name'])
                return {'ok': True}
            if t == 'goto_room':
                if req.get('thing'):
                    hits = self.places.find_thing(req['thing'])
                    if not hits:
                        raise ValueError(f'no room has "{req["thing"]}" in its contents list')
                    room = hits[0][1]
                else:
                    room = self.places.get_room(req.get('name', ''))
                    if not room:
                        raise ValueError(f'no room called "{req.get("name")}"')
                tgt = self.places.room_target(room)
                return self._enqueue([Stop(tgt['name'], tgt['x'], tgt['y'], tgt.get('yaw'))],
                                     req.get('priority', 1), req.get('source', 'dashboard'), tgt['name'])
            if t == 'set_dock':
                if req.get('here'):
                    if not pose:
                        raise ValueError('robot pose unknown')
                    x, y, yaw = pose
                else:
                    x, y, yaw = float(req['x']), float(req['y']), float(req['yaw'])
                self.places.set_dock(x, y, yaw)
                self.docked = bool(req.get('here'))
                self._log('charging dock position saved')
                return {'ok': True, 'dock': self.places.dock}
            if t == 'go_dock':
                if req.get('replace'):
                    self._cancel_all('going to charge')
                return self._go_dock(req.get('priority', 1), req.get('source', 'dashboard'))
            if t == 'schedule_add':
                rec = self.schedules.add(req.get('name'), req.get('at'), req.get('action'), req.get('days'),
                                         req.get('enabled', True), req.get('id'))
                self._log(f'schedule saved: {rec["name"]} at {rec["at"]}')
                return {'ok': True, 'schedule': rec, 'schedules': self.schedules.summary()}
            if t == 'schedule_delete':
                self.schedules.remove(req['id'])
                return {'ok': True, 'schedules': self.schedules.summary()}
            if t == 'schedule_enable':
                self.schedules.set_enabled(req['id'], req.get('on', True))
                return {'ok': True, 'schedules': self.schedules.summary()}
            if t == 'schedule_list':
                return {'ok': True, 'schedules': self.schedules.summary()}
            if t == 'mission_log':
                n = max(1, min(300, int(req.get('n', 50))))
                inc = bool(req.get('include_explore'))
                return {'ok': True, 'missions': self.mlog.recent(n, inc), 'stats': self.mlog.stats(inc)}
            if t == 'recovery_answer':
                return self._recovery_answer(req)
            if t == 'search':
                return {'ok': True, 'results': [{'score': s, **p} for s, p in self.places.search(req.get('q', ''), limit=8, min_score=0.35)]}
            return {'ok': False, 'error': f'unknown request type {t!r}'}
        except (KeyError, ValueError, TypeError) as e:
            return {'ok': False, 'error': str(e)}

    def _resolve(self, name):
        p = self.places.get(name)
        if p:
            return p
        hits = self.places.search(name, limit=1, min_score=0.6)
        if not hits:
            raise ValueError(f'no place matches "{name}"')
        return hits[0][1]

    @staticmethod
    def _stop_from_place(p):
        return Stop(p['name'], p['x'], p['y'], p.get('yaw'))

    def _mission_by_id(self, mid):
        if self.queue.active and self.queue.active.id == mid:
            return self.queue.active
        return next(q for q in self.queue.queue if q.id == mid)

    def _cancel_all(self, why):
        if self.explore.get('active'):
            self._explore_stop(why)
        if self.rec.busy():
            self.rec.cancel(why)
        n = self.queue.cancel(None)
        self.follower.clear()
        self._set(IDLE, why)
        if n:
            self._log(f'replaced {n} mission(s): {why}')

    def _enqueue(self, stops, priority, source, text):
        if source not in BATTERY_SOURCES and source != 'explore' and self.battery.current_level() == 'critical':
            raise ValueError(f'battery critical ({self.battery.pct()} %): charge me first')
        for st in stops:
            if not (math.isfinite(st.x) and math.isfinite(st.y)) or (st.yaw is not None and not math.isfinite(st.yaw)):
                raise ValueError('goal coordinates must be finite numbers')
        m = Mission(stops, max(0, min(3, int(priority))), source, text)
        m.by = getattr(self, '_req_by', '') if source not in ('schedule', 'explore', 'battery') else ''
        preempt = self.queue.add(m)
        if preempt:
            self.queue.preempt()
            self.follower.clear()
            self._set(IDLE, 'urgent task: current mission paused')
            self._log(f'urgent mission #{m.id} pre-empted the current one', 'warn')
        self._log(f'mission #{m.id} queued: {" -> ".join(s.label for s in stops)} (priority {m.priority})')
        return {'ok': True, 'mission': m.summary()}

    def _go_dock(self, priority, source):
        d = self.places.dock
        if not d:
            raise ValueError('the dock is not set (put the robot on the dock → Set dock here)')
        if self.dock_front:                       # in front of the dock, facing it
            ax = d['x'] - self.docker.p.predock_m * math.cos(d['yaw'])
            ay = d['y'] - self.docker.p.predock_m * math.sin(d['yaw'])
        else:                                     # v2.4: the approach point is ahead, then reverse in
            ax = d['x'] + self.dock_offset * math.cos(d['yaw'])
            ay = d['y'] + self.dock_offset * math.sin(d['yaw'])
        st = Stop('dock', ax, ay, d['yaw'], dock=True)
        return self._enqueue([st], priority, source, 'charge on the dock')

    def _find(self, q):
        hits = self.places.find_thing(q)
        here = self.places.room_at(*self._pose[:2]) if self._pose else None
        if not hits:
            p = self.places.search(q, limit=1, min_score=0.6)
            if p:
                return {'ok': True, 'answer': f'{p[0][1]["name"]} is a saved place on the map.', 'rooms': []}
            return {'ok': True, 'answer': f'I don\'t know where "{q}" is. Add it to a room\'s contents list.',
                    'rooms': []}
        names = [h[1]['name'] for h in hits if h[0] >= hits[0][0] - 0.02]
        ans = f'"{hits[0][2]}" is in {" or ".join(names)}.'
        if here and here['name'] in names:
            ans += ' That is the room I am in now.'
        return {'ok': True, 'answer': ans, 'rooms': [{'name': h[1]['name'], 'item': h[2], 'score': h[0]} for h in hits]}

    def _go_home(self, priority, source):
        if self.places.dock and not self.places.home:
            return self._go_dock(priority, source)
        h = self.places.home
        if not h:
            raise ValueError('home is not set (use "Set home here")')
        return self._enqueue([Stop('home', h['x'], h['y'], h.get('yaw'))], priority, source, 'home')

    def _command(self, text, dry_run=False):
        c = self.parser.parse(text)
        res = {'ok': True, 'interpretation': c}
        if dry_run:                              # the app shows the plan as a timeline before sending it
            res['plan'] = self._plan_summary(c)
            return res
        if c['intent'] == 'stop':
            if self.explore.get('active'):            # "stop" also ends Map by itself
                self._explore_stop('stopped by command')
            self.queue.cancel()
            if self.rec.busy():
                self.rec.cancel('stopped by command')
            self.follower.clear()
            self._set(IDLE, 'stopped by command')
        elif c['intent'] == 'return_home':
            res.update(self._go_home(c['priority'], 'command'))
        elif c['intent'] == 'navigate':
            stops = []
            for st in c.get('steps') or [{'place': n, 'wait_s': 0.0} for n in c['targets']]:
                stop = self._stop_from_place(self.places.target(st['place']))
                stop.wait_s = float(st.get('wait_s') or 0.0)
                stop.confirm = bool(st.get('confirm'))
                stops.append(stop)
            h = self.places.home
            if c.get('then_home') and h:
                stops.append(Stop('home', h['x'], h['y'], h.get('yaw')))
            res.update(self._enqueue(stops, c['priority'], 'command', text))
        elif c['intent'] == 'status':
            res['status'] = self.status()['message']
            b = self.battery.info()
            if b['pct'] is not None:
                res['status'] += f' · battery {b["pct"]} %' + (' (charging)' if b['charging'] else '')
        elif c['intent'] == 'dock':
            res.update(self._go_dock(c['priority'], 'command'))
        elif c['intent'] == 'explain':
            r = explain(self)
            res['answer'] = r['answer']
            res['interpretation']['reply'] = r['answer']
        elif c['intent'] == 'find':
            r = self._find(c.get('query', ''))
            res['answer'] = r['answer']
            res['interpretation']['reply'] = r['answer']
        elif c['intent'] == 'confirm':
            if self.state == CONFIRM:
                self.handle({'type': 'confirm'})
            else:
                res['interpretation']['reply'] = 'Nothing is waiting for an OK.' 
        self._log(f'command "{text}" -> {c["intent"]} {c["targets"]} via {c["source"]}')
        return res

    def _plan_summary(self, c):
        steps = []
        if c['intent'] == 'navigate':
            steps = [{'kind': 'go', 'label': st['place'], 'wait_s': st.get('wait_s', 0.0),
                      'confirm': bool(st.get('confirm'))}
                     for st in (c.get('steps') or [{'place': n, 'wait_s': 0.0} for n in c['targets']])]
            if c.get('then_home'):
                steps.append({'kind': 'home', 'label': 'home', 'wait_s': 0.0,
                              'missing': not self.places.home})
        elif c['intent'] == 'return_home':
            steps = [{'kind': 'home', 'label': 'home', 'wait_s': 0.0, 'missing': not self.places.home}]
        elif c['intent'] == 'dock':
            steps = [{'kind': 'dock', 'label': 'charging dock', 'wait_s': 0.0, 'missing': not self.places.dock}]
        return {'intent': c['intent'], 'priority': c['priority'], 'steps': steps}

    def _preview(self, req, pose):
        if not pose:
            raise ValueError('robot pose unknown')
        if 'name' in req:
            p = self._resolve(req['name'])
            target, label = (p['x'], p['y']), p['name']
        else:
            target, label = (float(req['x']), float(req['y'])), req.get('label', 'map point')
        path = self.planner.plan(pose[:2], target, self._dynamic_obstacles())
        if path is None:
            self.preview = {'label': label, 'path': [], 'error': self.planner.last_error,
                            'reason': _reason_code(self.planner.last_error)}
            return {'ok': False, 'error': self.planner.last_error, 'reason': self.preview['reason']}
        length = path_length(path)
        self.preview = {'label': label, 'path': _decimate(path), 'length': round(length, 2),
                        'eta_s': round(length / max(0.05, self.follower.p.max_linear * 0.7)),
                        'note': self.planner.last_note}
        return {'ok': True, **self.preview}

    def _dynamic_obstacles(self):
        return self.scan_world

    # ---------------------------------------------------------------- control loop
    def step(self, pose, front_clearance, dt):
        """pose = (x, y, yaw) in the map frame or None if unknown. Returns (v, w)."""
        v, w = self._step(pose, front_clearance, dt)
        self.mlog.sync(self.queue)
        return v, w

    def _step(self, pose, front_clearance, dt):
        now = self.clock()
        self._pose = pose
        if now >= self._next_sched:
            self._next_sched = now + 5.0
            self._run_schedules()
        if self.estopped or self.hold:
            if self.hold and not self.estopped and (self.queue.active or self.queue.queue):
                self.message = 'on hold after e-stop: press Resume'
            return 0.0, 0.0
        if not self.localized:
            if self.queue.active or self.queue.queue:
                self._set(WAITING, 'check the scan matches the map, then press "Position OK"')
            return 0.0, 0.0
        if self.state == RECOVERING:
            if pose is None:
                self._pose_lost_for += dt
            return self._recovery_tick(pose, now)
        if pose is None:
            self._pose_lost_for += dt
            if self.queue.active:
                if self._pose_lost_for > self.pose_lost_fail_s:
                    self._fail('POSE_LOST', f'no robot position for {self.pose_lost_fail_s:.0f} s '
                                            '(is SLAM running? is the ESP32 sending odometry?)')
                elif self._pose_lost_for > 3.0 and self.state != WAITING:
                    self._log('robot position lost: waiting for it to come back', 'warn')
                    self.follower.clear()
                    self._set(WAITING, 'position lost for a moment, waiting (check the battery and the ESP32 link)')
            return 0.0, 0.0
        if self._pose_lost_for > 3.0 and self.queue.active:
            self._log(f'position back after {self._pose_lost_for:.1f} s: continuing')
            self._set(PLANNING, 'position back, replanning')
        # a short gap (< 3 s, e.g. the Pi was busy) just pauses: keep the current route, no re-plan
        self._pose_lost_for = 0.0

        d = self.places.dock
        if not self.docked and d and self.battery.charging and self.state != DOCKING \
                and math.hypot(pose[0] - d['x'], pose[1] - d['y']) < 0.15:
            self.docked = True                     # started up on the dock / pushed onto it by hand
        self._battery_policy(pose)
        if self.explore.get('active'):
            self._explore_tick(pose, now)
        m = self.queue.active or self.queue.next()
        if m is None:
            self.mlog.idle()
            if self.docked:
                b = self.battery.info()
                self._set(CHARGING, 'on the dock' + (f', charging ({b["pct"]} %)' if b['charging'] else
                                                     (f' ({b["pct"]} %)' if b['pct'] is not None else '')))
                return 0.0, 0.0
            self._maybe_auto_home(pose, now)
            if self.state not in (IDLE,):
                self._set(IDLE, self.message if self.state == WAITING else 'idle')
            return 0.0, 0.0

        guard = FootprintGuard(self.footprint, self.scan_base) if (self.scan_base and self.body_checks) else None
        self.mlog.on_pose(m, pose, dt, blocked=self.follower.blocked,
                          gap=guard.min_gap if guard else None)

        if self.docked and self.state != DOCKING:     # leave the dock straight before turning anywhere
            return self._undock(pose, front_clearance)
        if self.state == DOCKING:
            return self._docking(m, pose, now)
        if self.state == CONFIRM:
            if now > self._confirm_until:
                self._log(f'nobody pressed OK at {m.current.label} in {self.confirm_timeout_s:.0f} s: continuing', 'warn')
                self._set(DWELL, 'no OK received, continuing')
                self._dwell_until = now
            else:
                self.message = f'at {m.current.label}: waiting for OK'
                return 0.0, 0.0

        if self.state == SCANNING and (self._scan or {}).get('mid') != m.id:
            self._set(PLANNING, f'next stop: {m.current.label}')      # scan belonged to a canceled goal
        if self.state == SCANNING:
            d = math.atan2(math.sin(pose[2] - self._scan['last']), math.cos(pose[2] - self._scan['last']))
            self._scan['acc'] += abs(d)
            self._scan['last'] = pose[2]
            self._scan['t'] += dt
            if self._scan['acc'] < 2 * math.pi * 0.97 and self._scan['t'] < self._scan['limit']:
                return 0.0, self._scan['w']
            self._set(DWELL, f'at {m.current.label}')
            self._dwell_until = now
            return 0.0, 0.0

        if self.state == DWELL:
            if now < self._dwell_until:
                return 0.0, 0.0
            m.next_stop += 1
            if m.current is None and m.repeat != 0:          # route loop / patrol: start the next round
                m.repeat -= 1 if m.repeat > 0 else 0
                m.round += 1
                m.next_stop = 0
                self._log(f'mission #{m.id}: round {m.round}')
            if m.current is None:
                self.queue.finish('DONE', 'all stops reached')
                self._mission_ended(m, True)
                self._log(f'mission #{m.id} done')
                self._set(IDLE, f'mission #{m.id} done')
                self._idle_since = now
                return 0.0, 0.0
            self._set(PLANNING, f'next stop: {m.current.label}')

        if self.state in (IDLE, PLANNING, WAITING) or not self.follower.path:
            if now < self._retry_at:
                return 0.0, 0.0
            return self._plan_current(m, pose, now)

        v, w, event = self.follower.step(pose, front_clearance, dt, guard=guard)
        if event == 'ARRIVED':
            m.skips = 0
            if m.current.via:                         # recovery detour point: straight on to the real stop
                self._log(f'passed {m.current.label}')
                self.follower.clear()
                m.stops.pop(m.next_stop)
                self._set(PLANNING, f'next stop: {m.current.label}')
                return 0.0, 0.0
            if self.rec.busy() and self.rec.current.get('mission') == m.id:
                self.rec.resolve('recovered')
                self._log(f'recovered: reached {m.current.label}')
            if m.current.dock:
                self.follower.clear()
                self._dock = {'t0': now, 'contact_t': None}
                if self.dock_front:
                    self.docker.start(now)
                    self._set(DOCKING, 'in front of the dock: looking for the marker')
                    self._log('in front of the dock: aligning with the marker')
                    self.say_queue.append(('dock', 'Docking.'))
                else:
                    self._set(DOCKING, 'reversing onto the dock')
                    self._log('at the dock: reversing onto the contacts')
                return 0.0, 0.0
            if m.source not in ('explore',) and m.current.label not in ('map goal', 'drawn path') \
                    and not m.current.label.startswith('point '):
                self.say_queue.append(('arrived', f'Arrived at {m.current.label}.'))
            self._log(f'arrived at {m.current.label}')
            self.follower.clear()
            self._replans = 0
            last = m.next_stop == len(m.stops) - 1
            self._dwell_until = now + (0.0 if last else self.dwell_s) + m.current.wait_s
            self._set(DWELL, f'at {m.current.label}')
            if m.current.confirm:
                self._confirm_until = now + self.confirm_timeout_s
                self._set(CONFIRM, f'at {m.current.label}: waiting for OK')
                self.say_queue.append(('confirm', f'I am at {m.current.label}. Please press OK on the app when you are done.'))
                self._log(f'waiting for OK at {m.current.label}')
            if m.source == 'explore' and self.explore_params.scan_spin:
                w = self.explore_params.scan_speed
                self._scan = {'mid': m.id, 'acc': 0.0, 'last': pose[2], 'w': w, 't': 0.0,
                              'limit': 2 * math.pi / max(0.1, w) * 2.0 + 5.0}
                self._set(SCANNING, 'scanning the area')
            return 0.0, 0.0
        if event in ('BLOCKED', 'OFF_PATH'):
            self.mlog.bump(m, 'replans')
            self._replans += 1
            if self._replans > self.max_replans:
                self._fail('BLOCKED', f'path to {m.current.label} stays blocked after {self.max_replans} new routes')
                return 0.0, 0.0
            why = 'obstacle in the way' if event == 'BLOCKED' else 'drifted off the path'
            self._log(f'{why}: finding another route ({self._replans}/{self.max_replans})', 'warn')
            self.follower.clear()
            self._set(PLANNING, f'{why}, replanning')
            return 0.0, 0.0
        if self.follower.blocked:
            if self._blocked_since is None:
                self._blocked_since = now
            elif now - self._blocked_since > 1.5:
                self.say_queue.append(('make_way', 'Excuse me, please make way.'))
        else:
            self._blocked_since = None
        self.message = (f'blocked, waiting… ({m.current.label})' if self.follower.blocked
                        else f'driving to {m.current.label}')
        return v, w

    def _plan_current(self, m, pose, now):
        stop = m.current
        extra = self._dynamic_obstacles() if self._replans else ()
        bad = []
        if stop.path:
            path = drawn_path.route_from_pose(self.planner, pose, stop.path,
                                              from_nearest=bool(self._replans), extra_obstacles=extra)
        else:
            path = None
            if stop.margin:                          # recovery: a wider berth than usual, if one exists
                path = self.planner.plan(pose[:2], (stop.x, stop.y), extra,
                                         radius=self.planner.p.robot_radius + stop.margin)
            if path is None:
                path = self.planner.plan(pose[:2], (stop.x, stop.y), extra)
            if path is not None and self.body_checks and not stop.margin:
                path, bad = self._fit_body(path, pose, stop, extra)
        if path is None:
            m.attempts += 1
            code = _reason_code(self.planner.last_error)
            if m.attempts >= self.max_plan_attempts or code in ('OUTSIDE_MAP', 'GOAL_IN_OBSTACLE'):
                self._fail(code, f'cannot reach {stop.label}: {self.planner.last_error}')
            else:
                self._retry_at = now + self.plan_retry_s
                self._set(WAITING, f'no route to {stop.label} yet ({self.planner.last_error}), retrying')
            return 0.0, 0.0
        m.attempts = 0
        key = (m.id, m.next_stop, m.round)
        if key != getattr(self, '_announced', None):           # "Going to Lab 3." once per stop, not per replan
            self._announced = key
            if not (stop.via or m.source in ('explore',) or stop.label in ('map goal', 'drawn path', 'map point')
                    or stop.label.startswith('point ') or stop.dock):
                self.say_queue.append(('going', f'Going to {stop.label}.'))
        self.follower.set_path(path, stop.yaw, tight=bool(bad))
        self.tight_spots = bad[:20]
        if bad:
            self._log(f'tight spot near ({bad[0][0]:.1f}, {bad[0][1]:.1f}): my body barely fits, driving slowly', 'warn')
        self.mlog.on_plan(m, m.next_stop, path_length(path), pose, (stop.x, stop.y))
        self.preview = None
        self._set(DRIVING, f'driving to {stop.label}')
        self._log(f'route to {stop.label}: {path_length(path):.1f} m' +
                  (f' ({self.planner.last_note})' if self.planner.last_note else ''))
        return 0.0, 0.0

    def _fit_body(self, path, pose, stop, extra):
        """Check the route with the real body shape; if it clips, try a wider berth (one more A*)."""
        note = self.planner.last_note
        bad = check_path(self.planner, path, self.footprint, start_yaw=pose[2], end_yaw=stop.yaw)
        if bad:
            wide_r = self.footprint.circumradius + self.footprint.margin
            wide = self.planner.plan(pose[:2], (stop.x, stop.y), extra, radius=wide_r)
            if wide is not None and not self.planner.last_note \
                    and path_length(wide) <= 1.5 * path_length(path) + 1.0:
                bad2 = check_path(self.planner, wide, self.footprint, start_yaw=pose[2], end_yaw=stop.yaw)
                if len(bad2) < len(bad):
                    path, bad = wide, bad2
                    note = 'wide route: keeps the whole body clear'
        self.planner.last_note = note
        return path, bad

    def _undock(self, pose, front_clearance):
        d = self.places.dock
        if not d or math.hypot(pose[0] - d['x'], pose[1] - d['y']) > 0.30:
            self.docked = False
            self._log('off the dock')
            self._set(PLANNING, 'off the dock')
            return 0.0, 0.0
        if self.dock_front:                       # nose on the dock: back straight off it first
            rear = -self.footprint.rear
            hw = self.footprint.half_width + 0.03
            if any(rear - 0.30 < x < rear and abs(y) < hw for x, y in self.scan_base):
                self._set(UNDOCKING, 'leaving the dock: something is behind me')
                return 0.0, 0.0
            self._set(UNDOCKING, 'backing off the dock')
            return -0.06, 0.0
        if front_clearance < self.follower.p.stop_distance:
            self._set(UNDOCKING, 'leaving the dock: something is in front of me')
            return 0.0, 0.0
        self._set(UNDOCKING, 'leaving the dock')
        return 0.08, 0.0

    def _front_docking(self, m, pose, now):
        d = self.places.dock
        obs, t = self._marker
        p = self.docker.p
        fresh = obs is not None and obs.get('found') and now - t < 0.6
        marker_world = (d['x'] + p.contact_dist_m * math.cos(d['yaw']), d['y'] + p.contact_dist_m * math.sin(d['yaw']),
                        d['yaw'] + math.pi)
        if fresh:
            obs = fuse_yaw(obs, pose[2], d['yaw'])
        elif self.dock_without_marker:
            obs = synth_observation(pose, marker_world, p)          # no camera: the map's idea of the marker
        else:
            obs = None
        rear = -self.footprint.rear
        hw = self.footprint.half_width + 0.03
        rear_clear = not any(rear - 0.30 < x < rear and abs(y) < hw for x, y in self.scan_base)
        v, w = self.docker.step(now, obs, pose, self.battery.info(), rear_clear)
        self.message = self.docker.message
        res = self.docker.result
        if res == 'docked':
            self.docker.stop()
            return self._docked(m, 'charging started')
        if res:
            self.docker.stop()
            self._log(f'docking failed: {res[2]}', 'error')
            self._fail(res[1], res[2])
            return 0.0, 0.0
        if self.docker.phase == 'backoff' and getattr(self, '_dock_said', None) != self.docker.try_n:
            self._dock_said = self.docker.try_n
            self._log(f'docking: {self.docker.message}', 'warn')
        return v, w

    def _docking(self, m, pose, now):
        """Reverse along the dock line until the charger current starts (or the dock pose is reached)."""
        if self.dock_front:
            return self._front_docking(m, pose, now)
        d = self.places.dock
        yd = d['yaw']
        c, s = math.cos(yd), math.sin(yd)
        ex, ey = pose[0] - d['x'], pose[1] - d['y']
        along = ex * c + ey * s                    # m in front of the dock (0 = sitting on it)
        lat = -ex * s + ey * c                     # m to the left of the dock line
        th = math.atan2(math.sin(pose[2] - yd), math.cos(pose[2] - yd))
        b = self.battery
        sensor = b.current_level() != 'unknown' and b.a is not None
        if sensor and b.charging:
            if self._dock['contact_t'] is None:
                self._dock['contact_t'] = now
            if now - self._dock['contact_t'] > 1.0:        # stable contact for a second
                return self._docked(m, 'charging started')
            return 0.0, 0.0
        if now - self._dock['t0'] > 25.0 or along < -0.05:
            if not sensor:
                return self._docked(m, 'docked (no current sensor to confirm the contacts)')
            self._fail('DOCK_NO_CONTACT', 'reversed onto the dock but no charging current')
            return 0.0, 0.0
        if not sensor and along < 0.005:
            return self._docked(m, 'docked (no current sensor to confirm the contacts)')
        if abs(th) > 0.35 and along > 0.15:               # line up first (back towards the dock)
            self.message = 'lining up with the dock'
            return 0.0, -math.copysign(0.35, th)
        th_des = max(-0.3, min(0.3, 2.5 * lat))
        w = max(-0.4, min(0.4, 2.0 * (th_des - th)))
        v = -0.05 if along > 0.08 else -0.03            # slow for the last few cm
        self.message = f'reversing onto the dock ({max(0.0, along) * 100:.0f} cm to go)'
        return v, w

    def _docked(self, m, why):
        self.docked = True
        self.queue.finish('DONE', why)
        self._log(why)
        self.say_queue.append(('dock', 'Docked. Charging.' if 'charging' in why else 'Docked.'))
        self._set(CHARGING, why)
        self._bat_mode = None
        return 0.0, 0.0

    def _battery_policy(self, pose):
        lvl = self.battery.current_level()
        if lvl in ('ok', 'charging', 'unknown'):
            if lvl != 'unknown':
                self._bat_mode = None
            return
        if self.docked or self._bat_mode == lvl or (self._bat_mode == 'critical' and lvl == 'low'):
            return
        active = self.queue.active
        if active and active.source == 'battery':
            return
        self._bat_mode = lvl
        pct = self.battery.pct()
        if self.explore.get('active'):
            self._explore_stop(f'battery {lvl}')
        try:
            if self.places.dock:
                r = self._go_dock(3 if lvl == 'critical' else 2, 'battery')
            elif self.places.home:
                r = self._go_home(3 if lvl == 'critical' else 2, 'battery')
            else:
                self._log(f'battery {lvl} ({pct} %) but no dock or home is set', 'error')
                return
        except ValueError as e:
            self._log(f'battery {lvl}: {e}', 'error')
            return
        self._log(f'battery {lvl} ({pct} %): going to charge' + (' now' if lvl == 'critical' else ' after this task'),
                  'warn')
        self.say_queue.append(('battery', 'My battery is low. I am going to charge.'))
        return r

    def _run_schedules(self):
        for sch in self.schedules.due():
            if self.estopped or self.hold:
                self._log(f'schedule "{sch["name"]}" skipped: e-stop', 'warn')
                continue
            res = self.handle(dict(sch['action'], source='schedule'))
            if res.get('ok'):
                self._log(f'schedule "{sch["name"]}" started')
            else:
                self._log(f'schedule "{sch["name"]}" could not start: {res.get("error")}', 'warn')

    def _fail(self, code, text):
        m = self.queue.active
        if (m and m.current and self.rec.enabled and code in RECOVERABLE and m.source != 'explore'
                and not m.skip_unreachable and not self.estopped and self._start_recovery(m, code, text)):
            return
        self._fail_now(code, text)

    def _fail_now(self, code, text):
        m = self.queue.active
        if self.rec.busy():
            self.rec.resolve('failed', 'failed')
        self.follower.clear()
        self._replans = 0
        if m and m.skip_unreachable and m.skips + 1 < len(m.stops) and code != 'POSE_LOST':
            m.skips += 1
            m.attempts = 0
            self.mlog.bump(m, 'skipped')
            self._log(f'skipping {m.current.label}: {text}', 'warn')
            m.next_stop += 1
            if m.current is None and m.repeat != 0:
                m.repeat -= 1 if m.repeat > 0 else 0
                m.round += 1
                m.next_stop = 0
            if m.current is not None:
                self._set(PLANNING, f'skipped a point, next: {m.current.label}')
                return
            self.queue.finish('DONE', 'route finished (some points skipped)')
            self._set(IDLE, 'route finished (some points skipped)')
            self._idle_since = self.clock()
            return
        if m:
            m.message = f'{code}: {text}'
            self.queue.finish('FAILED', m.message)
            self._mission_ended(m, False)
            if m.source == 'explore':          # unreachable frontier: not an error, just skip it
                self._log(f'skipping an unreachable area ({code})', 'warn')
                self.follower.clear()
                self._replans = 0
                self._set(IDLE, 'exploring: trying another area')
                return
            self._log(f'mission #{m.id} failed: {text}', 'error')
        self._set(IDLE, f'failed: {text}')
        self.last_failure = {'code': code, 'text': text, 'mission': m.summary() if m else None, 't': time.time()}
        self._idle_since = self.clock()

    # ---------------------------------------------------------------- error recovery
    def _start_recovery(self, m, code, text):
        if self.rec.busy():                            # the last recovery did not work
            self.rec.resolve('failed_again', 'failed')
        if m.current.via:                              # the detour itself failed: drop it, count the real stop
            m.stops.pop(m.next_stop)
            if m.current is None:
                return False
        used, here = self.rec.tries(m.id, m.current.label)
        if used >= self.rec.max_per_mission:
            self._log(f'no recovery left for mission #{m.id} ({used} used)', 'warn')
            return False
        ctx = self._recovery_context(m, code, text, here, used)
        self.follower.clear()
        self._replans = 0
        m.attempts = 0
        if code == 'POSE_LOST':
            self._pose_lost_for = 0.0
        self.rec.start(ctx)
        self.last_failure = {'code': code, 'text': text, 'mission': m.summary(), 't': time.time(), 'recovering': True}
        self._set(RECOVERING, f'{code} at {m.current.label}: deciding what to do')
        self._log(f'{m.current.label}: {text} -> recovering ({self.rec.mode})', 'warn')
        return True

    def _recovery_context(self, m, code, text, tries_here, tries_mission):
        pose = self._pose
        st = m.current
        b = self.battery.info()
        lt = self.localtime()
        room = self.places.room_at(*pose[:2]) if pose else None
        return {
            'failure': {'code': code, 'text': text, 'place': st.label, 'stop_index': m.next_stop},
            'mission': {'id': m.id, 'text': m.text, 'stops': [s.label for s in m.stops], 'next_stop': m.next_stop,
                        'priority': m.priority, 'source': m.source, 'round': m.round, 'repeat': m.repeat},
            'queue': [{'id': q.id, 'text': q.text, 'priority': q.priority} for q in self.queue.queue[:5]],
            'battery': {'pct': b.get('pct'), 'level': b.get('level'), 'charging': b.get('charging')},
            'time': time.strftime('%H:%M %A', lt),
            'robot': {'x': round(pose[0], 2), 'y': round(pose[1], 2), 'room': room['name'] if room else None}
            if pose else None,
            'history': {'this_place': self.rec.place_history(st.label),
                        'recent': [{'place': h['place'], 'code': h['code'], 'action': h['action'], 'outcome': h['outcome']}
                                   for h in self.rec.history[:5]]},
            'tries_here': tries_here, 'tries_mission': tries_mission, 'max_tries': self.rec.max_per_mission,
            'alternatives': self._alternatives(st, pose) if code in ('BLOCKED', 'NO_PATH') else [],
            'home_set': bool(self.places.home), 'dock_set': bool(self.places.dock),
            'going_home': st.label == 'home',
        }

    def _alternatives(self, stop, pose):
        """Detour waypoints: saved places from which the stop is reachable, by total route length."""
        if not pose or not self.planner.has_map():
            return []
        extra = self._dynamic_obstacles()
        cands = list(self.places.places) + ([dict(self.places.home, name='home')] if self.places.home else [])
        def far(c):
            return (math.hypot(c['x'] - stop.x, c['y'] - stop.y) > 1.0 and
                    math.hypot(c['x'] - pose[0], c['y'] - pose[1]) > 1.0)
        cands = sorted([c for c in cands if far(c)],
                       key=lambda c: math.hypot(c['x'] - pose[0], c['y'] - pose[1]) + math.hypot(c['x'] - stop.x, c['y'] - stop.y))[:6]
        out = []
        for c in cands:
            a = self.planner.plan(pose[:2], (c['x'], c['y']), extra)
            b = self.planner.plan((c['x'], c['y']), (stop.x, stop.y), extra) if a else None
            if a and b:
                out.append({'label': c['name'], 'x': round(c['x'], 2), 'y': round(c['y'], 2),
                            'length_m': round(path_length(a) + path_length(b), 1)})
        out.sort(key=lambda o: o['length_m'])
        for i, o in enumerate(out[:3]):
            o['id'] = f'via{i + 1}'
        return out[:3]

    def _recovery_tick(self, pose, now):
        m = self.queue.active
        r = self.rec.current
        if m is None or r is None:
            if self.rec.busy():
                self.rec.cancel('mission gone')
            self._set(IDLE, 'idle')
            return 0.0, 0.0
        if r['status'] == 'deciding':
            d = self.rec.poll()
            if d is None:
                self.message = 'deciding what to do'
                return 0.0, 0.0
            return self._apply_recovery(m, d, now)
        if r['status'] == 'asking':
            if r['_until'] is not None and now > r['_until']:
                self._log('nobody answered in the app: the rules decide', 'warn')
                d = rule_decide(r['_ctx'], exclude=('ask_person',))
                r['why'] = 'nobody answered in the app; ' + (d['reason'] if d else 'nothing left to try')
                r['source'] = 'rules'
                return self._apply_recovery(m, d or {'action': None, 'params': {}, 'say': ''}, now)
            self.message = f'at {m.current.label if m.current else "?"}: waiting for an answer in the app'
            return 0.0, 0.0
        if r['status'] == 'running' and r['_until'] is not None and now < r['_until']:
            self.message = r['doing']
            return 0.0, 0.0
        if pose is None:                               # still lost: keep waiting, the pose timer decides
            self._set(WAITING, 'position lost, waiting for it to come back')
            return 0.0, 0.0
        self._set(PLANNING, f'trying {m.current.label} again' if m.current else 'recovering')
        return 0.0, 0.0

    def _apply_recovery(self, m, d, now):
        r = self.rec.current
        ctx = r['_ctx']
        code, text = ctx['failure']['code'], ctx['failure']['text']
        a, p = d.get('action'), d.get('params') or {}
        if a is None:
            self._fail_now(code, text)
            return 0.0, 0.0
        self.mlog.rec(m)['recoveries'].append({'code': code, 'place': ctx['failure']['place'], 'action': a,
                                               'params': p, 'source': r['source'], 'reason': d.get('reason', ''),
                                               't': round(time.time(), 1)})
        alt = next((o for o in ctx.get('alternatives', []) if o['id'] == p.get('via')), None)
        doing = describe(a, p, ctx, alt and alt['label'])
        self._log(f'recovery ({r["source"]}): {doing} {("- " + d["reason"]) if d.get("reason") else ""}'.strip(), 'warn')
        if d.get('say'):
            self.say_queue.append(('recovery', d['say']))
        if a == 'wait_retry':
            self.rec.running(doing, until=now + p['wait_s'])
            self.message = doing
            return 0.0, 0.0
        if a == 'replan_wide':
            m.current.margin = p['margin_m']
            self.rec.running(doing)
        elif a == 'alt_route':
            m.stops.insert(m.next_stop, Stop(f'via {alt["label"]}', alt['x'], alt['y'], None, via=True))
            self.rec.running(doing)
        elif a == 'ask_person':
            opts = [o for o in ANSWERS.items() if o[1][0] is None or o[1][0] in ctx['allowed'] or o[0] in ('retry', 'wait')]
            self.rec.asking(p.get('question') or doing, [(k, v[2], k == 'retry') for k, v in opts])
            self.rec.current['doing'] = 'Waiting for someone to choose in the app.'
            self.say_queue.append(('recovery_ask', p.get('question') or 'I need help. Please check the app.'))
            self.message = 'waiting for an answer in the app'
            return 0.0, 0.0
        elif a == 'skip_stop':
            self.rec.running(doing)
            self.rec.resolve('skipped')
            self.mlog.bump(m, 'skipped')
            m.next_stop += 1
            if m.current is None:
                self.queue.finish('DONE', f'finished, {ctx["failure"]["place"]} skipped after {code}')
                self._mission_ended(m, True)
                self._set(IDLE, f'mission #{m.id} finished ({ctx["failure"]["place"]} skipped)')
                self._idle_since = self.clock()
                return 0.0, 0.0
        elif a == 'reorder':
            self.rec.running(doing)
            self.rec.resolve('postponed')
            if self.queue.queue:
                m.state, m.message, m.created = 'QUEUED', f'postponed after {code}', time.time()
                self.queue.queue.append(m)
                self.queue.queue.sort(key=lambda q: (-q.priority, q.created, q.id))
                self.queue.active = None
                self._set(IDLE, 'postponed this task, the next one goes first')
                return 0.0, 0.0
            m.stops.append(m.stops.pop(m.next_stop))
        elif a in ('go_home', 'go_dock'):
            self.rec.running(doing)
            self.rec.resolve('gave_up_' + a[3:])
            m.message = f'{code}: {text} (recovery: {a})'
            self.queue.finish('FAILED', m.message)
            self._mission_ended(m, False)
            self.last_failure = {'code': code, 'text': text, 'mission': m.summary(), 't': time.time()}
            try:
                (self._go_dock if a == 'go_dock' else self._go_home)(2, 'recovery')
            except ValueError as e:
                self._log(f'recovery {a} not possible: {e}', 'error')
            self._set(IDLE, doing)
            return 0.0, 0.0
        self._set(PLANNING, doing)
        return 0.0, 0.0

    def _recovery_answer(self, req):
        r = self.rec.current
        if not r or r['status'] != 'asking':
            raise ValueError('nothing is waiting for an answer')
        if req.get('id') is not None and int(req['id']) != r['id']:
            raise ValueError('that question was already answered')
        choice = str(req.get('choice') or '')
        if choice not in ANSWERS:
            raise ValueError(f'choice must be one of {", ".join(ANSWERS)}')
        action, params, label = ANSWERS[choice]
        who = self._req_by or 'someone'
        self._log(f'{who} answered in the app: {label}')
        r['why'] = f'{who} chose "{label}" in the app'
        r['source'] = 'person'
        m = self.queue.active
        if action is None:
            self.rec.resolve('canceled_by_person')
            if m:
                self.queue.cancel(m.id)
            self.follower.clear()
            self._set(IDLE, 'mission canceled from the app')
            return {'ok': True, 'canceled': True}
        d = validate({'action': action, 'params': params, 'reason': r['why']},
                     dict(r['_ctx'], allowed=list(set(r['_ctx']['allowed']) | {'wait_retry'})))
        r['status'] = 'deciding'
        if m:
            self._apply_recovery(m, d, self.clock())
        return {'ok': True, 'action': action}

    # ---------------------------------------------------------------- exploration
    def _explore_start(self, req, pose):
        if self.grid is None or not self.planner.has_map():
            raise ValueError('no map yet: wait for the first scans')
        if not pose:
            raise ValueError('robot pose unknown')
        if self.estopped or self.hold:
            raise ValueError('release the e-stop and resume first')
        p = self.explore_params
        p.speed = min(0.4, max(0.05, float(req.get('speed', p.speed))))
        p.turn_speed = min(1.2, max(0.2, float(req.get('turn_speed', p.turn_speed))))
        if 'scan_spin' in req:
            p.scan_spin = bool(req['scan_spin'])
        if not self.explore.get('active'):
            self.explore = {'active': True, 'state': 'running', 'started': self.clock(), 'goals_done': 0,
                            'skipped': 0, 'blacklist': [], 'frontiers': [], 'target': None,
                            'start_pose': tuple(pose), 'none_count': 0, 'next_pick': 0.0,
                            'saved_speed': (self.follower.p.max_linear, self.follower.p.max_angular),
                            'area': round(mapped_area(self.grid), 1), 'message': 'looking for unexplored areas'}
            self._log(f'exploration started at {p.speed:.2f} m/s')
        self.follower.p.max_linear, self.follower.p.max_angular = p.speed, p.turn_speed
        return {'ok': True, 'explore': self._explore_status()}

    def _explore_stop(self, why, done=False):
        e = self.explore
        if not e.get('active'):
            return {'ok': True, 'explore': self._explore_status()}
        e['active'] = False
        e['state'] = 'done' if done else 'stopped'
        e['message'] = why
        self.follower.p.max_linear, self.follower.p.max_angular = e['saved_speed']
        # drop exploration goals, keep anything a person asked for
        if self.queue.active and self.queue.active.source == 'explore':
            self.queue.cancel(self.queue.active.id)
            self.follower.clear()
            self._set(IDLE, why)
        for m in list(self.queue.queue):
            if m.source == 'explore':
                self.queue.cancel(m.id)
        self._log(f'exploration {"finished" if done else "stopped"}: {why}')
        return {'ok': True, 'explore': self._explore_status()}

    def _explore_tick(self, pose, now):
        e, p = self.explore, self.explore_params
        m = self.queue.active
        if m and m.source == 'explore':
            if now - e.get('goal_started', now) > p.max_goal_time:
                if e.get('target'):
                    e['blacklist'].append(e['target'])
                e['skipped'] += 1
                e['target'] = None
                self.queue.cancel(m.id)
                self.follower.clear()
                self._set(IDLE, 'exploring: trying another area')
                self._log('exploration goal took too long: skipping that area', 'warn')
            return
        if m or self.queue.queue or now < e['next_pick'] or self.grid is None or self.hold:
            return                                  # a person's mission runs first
        e['next_pick'] = now + 2.0
        fr = find_frontiers(self.grid, p.min_frontier_cells)
        e['frontiers'] = [[round(f.cx, 2), round(f.cy, 2), f.size] for f in fr[:30]]
        e['area'] = round(mapped_area(self.grid), 1)
        pick = choose_goal(fr, self.planner, pose, e['blacklist'], p, self.planner.p.robot_radius,
                           self._dynamic_obstacles())
        if pick is None:
            e['none_count'] += 1
            e['target'] = None
            e['message'] = 'no reachable unexplored area left' if fr else 'everything reachable is mapped'
            if e['none_count'] >= 3:
                self._explore_stop(e['message'], done=True)
                h = self.places.home
                home = (h['x'], h['y'], h.get('yaw')) if h else e['start_pose']
                try:
                    self._enqueue([Stop('home' if h else 'exploration start', home[0], home[1], home[2])],
                                  0, 'explore-return', 'return after exploring')
                except ValueError:
                    pass
            return
        gx, gy, f, _ = pick
        e['none_count'] = 0
        e['target'] = (round(gx, 2), round(gy, 2))
        e['goal_started'] = now
        e['message'] = f'heading to an unmapped spot ({f.size * self.grid.resolution:.1f} m of open edge)'
        self._enqueue([Stop('unexplored area', gx, gy, None)], 0, 'explore', 'explore')

    def _mission_ended(self, m, ok):
        e = self.explore
        if m.source != 'explore' or not e.get('active') or not e.get('target'):
            return
        e['blacklist'].append(e['target'])          # don't pick the same spot again
        if ok:
            e['goals_done'] += 1
        else:
            e['skipped'] += 1
        e['target'] = None
        e['next_pick'] = 0.0

    def _explore_status(self):
        e = self.explore
        out = {k: e.get(k) for k in ('active', 'state', 'goals_done', 'skipped', 'frontiers', 'target',
                                       'area', 'message')}
        if e.get('started') is not None:
            out['elapsed_s'] = round(self.clock() - e['started'])
        return out

    def _maybe_auto_home(self, pose, now):
        h = self.places.home
        if not (self.auto_return_s and h) or self.estopped:
            return
        if now - self._idle_since > self.auto_return_s and math.hypot(pose[0] - h['x'], pose[1] - h['y']) > 0.5:
            self._idle_since = now
            try:
                self._go_home(0, 'auto-home')
            except ValueError as e:                 # e.g. battery critical: never crash the control loop
                self._log(f'auto-return home skipped: {e}', 'warn')

    # ---------------------------------------------------------------- status
    def _set(self, state, message):
        self.state, self.message = state, message

    def _log(self, text, level='info'):
        self.events.insert(0, {'t': round(time.time(), 1), 'text': text, 'level': level})
        del self.events[40:]

    def status(self):
        m = self.queue.active
        pose = self._pose
        remaining = self.follower.remaining_path() if self.follower.path else []
        dist = self.follower.remaining_length(pose) if (pose and self.follower.path) else None
        return {
            'state': self.state, 'message': self.message, 'estop': self.estopped, 'hold': self.hold,
            'localized': self.localized, 'last_failure': self.last_failure,
            'goal': (None if not (m and m.current) else
                     {'label': m.current.label, 'x': m.current.x, 'y': m.current.y, 'yaw': m.current.yaw}),
            'path': _decimate(remaining), 'distance': None if dist is None else round(dist, 2),
            'eta_s': None if dist is None else round(dist / max(0.05, self.follower.p.max_linear * 0.7)),
            'blocked': self.follower.blocked, 'preview': self.preview,
            'missions': self.queue.summary(), 'events': self.events[:15],
            'has_map': self.planner.has_map(),
            'explore': self._explore_status(),
            'footprint': self.footprint.as_list(), 'tight': self.tight_spots,
            'battery': self.battery.info(), 'docked': self.docked,
            'room': (lambda r: r['name'] if r else None)(self.places.room_at(*pose[:2]) if pose else None),
            'confirm': ({'label': m.current.label, 'left_s': round(max(0.0, self._confirm_until - self.clock()))}
                        if (self.state == CONFIRM and m and m.current) else None),
            'schedules': self.schedules.summary(),
            'log_stats': self.mlog.stats(),
            'recovery': self.rec.public(), 'recovery_mode': self.rec.mode,
            'docking': self._docking_status(m), 'dock_direction': 'front' if self.dock_front else 'reverse',
        }

    def _docking_status(self, m):
        """For the app's docking card: phase predock / align / creep / contact / backoff / failed / charging."""
        if not self.dock_front:
            return None
        if self.state == DOCKING:
            return self.docker.status()
        if m and m.current and m.current.dock:
            return {'phase': 'predock', 'try': None, 'tries': self.docker.p.tries,
                    'message': 'driving to the front of the dock', 'marker': None}
        if self.docked and self.state == CHARGING:
            pct = self.battery.info().get('pct')
            return {'phase': 'charging', 'try': None, 'tries': self.docker.p.tries, 'marker': None,
                    'message': 'on the dock' + (f', {pct} %' if pct is not None else '')}
        return None


def _decimate(path, step=0.2):
    if not path:
        return []
    out = [path[0]]
    for p in path[1:]:
        if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) >= step:
            out.append(p)
    if out[-1] != path[-1]:
        out.append(path[-1])
    return [[round(x, 3), round(y, 3)] for x, y in out]
