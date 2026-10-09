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

Failure reason codes (for logs, the dashboard and future LLM error recovery):
  NO_MAP, NO_PATH, GOAL_IN_OBSTACLE, OUTSIDE_MAP, START_BLOCKED, BLOCKED, POSE_LOST, ESTOP, UNKNOWN_PLACE
"""
import math
import time

from lidar_robot.commands import CommandParser
from lidar_robot.explorer import ExploreParams, find_frontiers, choose_goal, mapped_area
from lidar_robot.follower import FollowerParams, PathFollower
from lidar_robot.missions import Mission, MissionQueue, Stop
from lidar_robot.planner import Planner, PlannerParams, path_length
from lidar_robot.tuning import (TuningStore, apply_follower, follower_values, ranges_for_app,
                                FOLLOWER_RANGES)

IDLE, PLANNING, DRIVING, DWELL, WAITING = 'IDLE', 'PLANNING', 'DRIVING', 'DWELL', 'WAITING'
SCANNING = 'SCANNING'          # exploration: turning slowly on the spot so the LiDAR sees every wall


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
                 clock=time.monotonic, tuning_path=None):
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
        self._scan = None
        self.grid = None                # latest GridMap (for exploration)
        self.explore_params = ExploreParams()
        self.explore = {'active': False, 'state': 'off'}

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

    def set_estop(self, on):
        if on and not self.estopped:
            self.estopped = True
            self.hold = True
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
                return self._enqueue([st], req.get('priority', 1), req.get('source', 'dashboard'), st.label)
            if t == 'command':
                return self._command(req.get('text', ''))
            if t == 'preview':
                return self._preview(req, pose)
            if t == 'cancel':
                if req.get('id') is None and self.explore.get('active'):
                    self._explore_stop('canceled')
                n = self.queue.cancel(req.get('id'))
                if not self.queue.active:
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

    def _enqueue(self, stops, priority, source, text):
        for st in stops:
            if not (math.isfinite(st.x) and math.isfinite(st.y)) or (st.yaw is not None and not math.isfinite(st.yaw)):
                raise ValueError('goal coordinates must be finite numbers')
        m = Mission(stops, max(0, min(3, int(priority))), source, text)
        preempt = self.queue.add(m)
        if preempt:
            self.queue.preempt()
            self.follower.clear()
            self._set(IDLE, 'urgent task: current mission paused')
            self._log(f'urgent mission #{m.id} pre-empted the current one', 'warn')
        self._log(f'mission #{m.id} queued: {" -> ".join(s.label for s in stops)} (priority {m.priority})')
        return {'ok': True, 'mission': m.summary()}

    def _go_home(self, priority, source):
        h = self.places.home
        if not h:
            raise ValueError('home is not set (use "Set home here")')
        return self._enqueue([Stop('home', h['x'], h['y'], h.get('yaw'))], priority, source, 'home')

    def _command(self, text):
        c = self.parser.parse(text)
        res = {'ok': True, 'interpretation': c}
        if c['intent'] == 'stop':
            self.queue.cancel()
            self.follower.clear()
            self._set(IDLE, 'stopped by command')
        elif c['intent'] == 'return_home':
            res.update(self._go_home(c['priority'], 'command'))
        elif c['intent'] == 'navigate':
            stops = [self._stop_from_place(self.places.get(n)) for n in c['targets']]
            res.update(self._enqueue(stops, c['priority'], 'command', text))
        elif c['intent'] == 'status':
            res['status'] = self.status()['message']
        self._log(f'command "{text}" -> {c["intent"]} {c["targets"]} via {c["source"]}')
        return res

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
        now = self.clock()
        self._pose = pose
        if self.estopped or self.hold:
            if self.hold and not self.estopped and (self.queue.active or self.queue.queue):
                self.message = 'on hold after e-stop: press Resume'
            return 0.0, 0.0
        if not self.localized:
            if self.queue.active or self.queue.queue:
                self._set(WAITING, 'check the scan matches the map, then press "Position OK"')
            return 0.0, 0.0
        if pose is None:
            self._pose_lost_for += dt
            if self.queue.active and self._pose_lost_for > 3.0:
                self._fail('POSE_LOST', 'lost the robot position (is SLAM running?)')
            return 0.0, 0.0
        self._pose_lost_for = 0.0

        if self.explore.get('active'):
            self._explore_tick(pose, now)
        m = self.queue.active or self.queue.next()
        if m is None:
            self._maybe_auto_home(pose, now)
            if self.state not in (IDLE,):
                self._set(IDLE, self.message if self.state == WAITING else 'idle')
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

        v, w, event = self.follower.step(pose, front_clearance, dt)
        if event == 'ARRIVED':
            self._log(f'arrived at {m.current.label}')
            self.follower.clear()
            self._replans = 0
            last = m.next_stop == len(m.stops) - 1
            self._dwell_until = now + (0.0 if last else self.dwell_s)
            self._set(DWELL, f'at {m.current.label}')
            if m.source == 'explore' and self.explore_params.scan_spin:
                w = self.explore_params.scan_speed
                self._scan = {'mid': m.id, 'acc': 0.0, 'last': pose[2], 'w': w, 't': 0.0,
                              'limit': 2 * math.pi / max(0.1, w) * 2.0 + 5.0}
                self._set(SCANNING, 'scanning the area')
            return 0.0, 0.0
        if event in ('BLOCKED', 'OFF_PATH'):
            self._replans += 1
            if self._replans > self.max_replans:
                self._fail('BLOCKED', f'path to {m.current.label} stays blocked after {self.max_replans} new routes')
                return 0.0, 0.0
            why = 'obstacle in the way' if event == 'BLOCKED' else 'drifted off the path'
            self._log(f'{why}: finding another route ({self._replans}/{self.max_replans})', 'warn')
            self.follower.clear()
            self._set(PLANNING, f'{why}, replanning')
            return 0.0, 0.0
        self.message = (f'blocked, waiting… ({m.current.label})' if self.follower.blocked
                        else f'driving to {m.current.label}')
        return v, w

    def _plan_current(self, m, pose, now):
        stop = m.current
        path = self.planner.plan(pose[:2], (stop.x, stop.y),
                                 self._dynamic_obstacles() if self._replans else ())
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
        self.follower.set_path(path, stop.yaw)
        self.preview = None
        self._set(DRIVING, f'driving to {stop.label}')
        self._log(f'route to {stop.label}: {path_length(path):.1f} m' +
                  (f' ({self.planner.last_note})' if self.planner.last_note else ''))
        return 0.0, 0.0

    def _fail(self, code, text):
        m = self.queue.active
        self.follower.clear()
        self._replans = 0
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
        self.last_failure = {'code': code, 'text': text, 'mission': m.summary() if m else None}
        self._idle_since = self.clock()

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
            self._go_home(0, 'auto-home')
            self._idle_since = now

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
        }


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
