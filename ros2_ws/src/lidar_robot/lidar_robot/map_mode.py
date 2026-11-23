"""Map mode chosen in the app: keep mapping, or freeze the map and only localize on it (pure Python, tested).

The choice lives in ~/maps/map_mode.json and the launch reads it at every start, so the robot boots into
whatever was chosen last (no editing ~/robot.env). Switching = save the choice, then restart the robot
software: the dashboard exits with RESTART_CODE, the launch shuts down on that, systemd starts it again
(robot.service has Restart=always).

    {"mode": "localization", "map": "/home/arpit/maps/cse_floor_2026-11-02.pbstream", "since": 1793...}

Modes:
  mapping       Cartographer builds the map live. With "map" set: continues that saved map (loaded
                unfrozen, so new areas are added and old places keep their coordinates).
  localization  The saved map is frozen (load_frozen_state); the robot only finds itself on it.
                Nothing is added: people and chairs never become walls.

Actions from the app (plan_switch):
  freeze    mapping -> save the live map as <name>.pbstream (+ .pgm/.yaml) -> localization on it
  continue  -> mapping on top of the current saved map (or the one given)
  new       -> mapping from an empty map
  use       -> localization on one of the saved maps
"""
import json
import os
import re
import time

RESTART_CODE = 42
MODES = ('mapping', 'localization')
ACTIONS = ('freeze', 'continue', 'new', 'use')


def default_file(map_dir='~/maps'):
    return os.path.join(os.path.expanduser(map_dir), 'map_mode.json')


def read_mode(path):
    """The saved choice, or None (no file / unreadable / nonsense)."""
    try:
        with open(path) as f:
            d = json.load(f)
    except (OSError, ValueError):
        return None
    if not isinstance(d, dict) or d.get('mode') not in MODES:
        return None
    m = str(d.get('map') or '')
    pose = d.get('pose')
    try:
        pose = {k: float(pose[k]) for k in ('x', 'y', 'yaw')} if isinstance(pose, dict) else None
    except (KeyError, TypeError, ValueError):
        pose = None
    return {'mode': d['mode'], 'map': os.path.expanduser(m) if m else '', 'since': d.get('since'), 'pose': pose}


def write_mode(path, mode, map_file='', clock=time.time, pose=None):
    """pose: where the robot stood on the map when it was frozen ({'x','y','yaw'}): after the restart the
    dashboard starts localization there, instead of at the spot where mapping began (18 Nov: the robot showed
    up at the map's start and needed Set position by hand)."""
    if mode not in MODES:
        raise ValueError(f'mode must be one of {MODES}')
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    d = {'mode': mode, 'map': map_file or '', 'since': int(clock())}
    if pose:
        d['pose'] = {k: round(float(pose[k]), 4) for k in ('x', 'y', 'yaw')}
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(d, f)
    os.replace(tmp, path)


def startup_pose(saved, slam_mode, map_file, now, max_age_s=900.0):
    """The frozen-at pose to start localization from, or None: only right after a Freeze of this very map."""
    if not saved or slam_mode != 'localization' or not saved.get('pose'):
        return None
    if os.path.expanduser(saved.get('map') or '') != os.path.expanduser(map_file or ''):
        return None
    since = saved.get('since')
    if not isinstance(since, (int, float)) or not 0 <= now - since <= max_age_s:
        return None
    return saved['pose']


# ---------------------------------------------------------------- fixed start position (23 Nov)
# An operator marks where the robot is parked when it is switched on (e.g. Home by the lab door). After every
# restart on that saved map the dashboard starts localization there, so nobody has to use Set position by hand.
START_FILE = 'start_poses.json'


def _map_key(map_file):
    b = os.path.basename(map_file or '')
    return b[:-9] if b.endswith('.pbstream') else b


def read_start_poses(map_dir):
    try:
        with open(os.path.join(os.path.expanduser(map_dir), START_FILE)) as f:
            d = json.load(f)
        return d if isinstance(d, dict) else {}
    except (OSError, ValueError):
        return {}


def get_start_pose(map_dir, map_file):
    """The fixed start position of this saved map ({'x','y','yaw', 'by', 't'}) or None."""
    key = _map_key(map_file)
    p = read_start_poses(map_dir).get(key) if key else None
    if not isinstance(p, dict):
        return None
    try:
        out = {k: float(p[k]) for k in ('x', 'y', 'yaw')}
    except (KeyError, TypeError, ValueError):
        return None
    if not all(v == v and abs(v) < 1e4 for v in out.values()):
        return None
    out.update(by=str(p.get('by', '')), t=p.get('t'))
    return out


def set_start_pose(map_dir, map_file, pose, by='', clock=time.time):
    """pose {'x','y','yaw'} to fix the start position of this map, None to remove it. Atomic write."""
    key = _map_key(map_file)
    if not key:
        raise ValueError('The start position belongs to a saved map: use a saved map first.')
    d = read_start_poses(map_dir)
    if pose is None:
        d.pop(key, None)
    else:
        v = {k: round(float(pose[k]), 4) for k in ('x', 'y', 'yaw')}
        if not all(x == x and abs(x) < 1e4 for x in v.values()):
            raise ValueError('start position must be finite numbers')
        d[key] = dict(v, by=str(by)[:40], t=int(clock()))
    path = os.path.join(os.path.expanduser(map_dir), START_FILE)
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(d, f)
    os.replace(tmp, path)
    return get_start_pose(map_dir, map_file)


def resolve(cli_mode, cli_map, path):
    """What the launch should run: the app's saved choice wins over ROBOT_ARGS.

    -> (mode, map_file, note). A localization choice whose map file is gone falls back to mapping (the
    robot must still boot), with a note for the log."""
    d = read_mode(path) if path else None
    if d is None:
        return cli_mode, cli_map, ''
    if d['mode'] == 'localization' and not os.path.isfile(d['map']):
        return 'mapping', '', f'map mode file asks for {d["map"]} but it is missing: mapping instead'
    if d['mode'] == 'mapping' and d['map'] and not os.path.isfile(d['map']):
        return 'mapping', '', f'map {d["map"]} to continue is missing: new map instead'
    return d['mode'], d['map'], f'map mode from {path}'


def list_maps(map_dir):
    """Saved maps usable by Cartographer (.pbstream), newest first."""
    d = os.path.expanduser(map_dir)
    out = []
    try:
        names = os.listdir(d)
    except OSError:
        return out
    for n in names:
        if not n.endswith('.pbstream'):
            continue
        p = os.path.join(d, n)
        try:
            st = os.stat(p)
        except OSError:
            continue
        out.append({'name': n[:-len('.pbstream')], 'file': p, 'size_kb': round(st.st_size / 1024),
                    'mtime': int(st.st_mtime), 'picture': os.path.isfile(p[:-len('.pbstream')] + '.pgm')})
    return sorted(out, key=lambda m: -m['mtime'])


def default_name(clock=time.localtime):
    return time.strftime('map_%Y-%m-%d_%H%M', clock())


def safe_map_name(name):
    name = re.sub(r'[^A-Za-z0-9_\-]', '_', str(name or '').strip())[:60]
    return name


def plan_switch(action, current_mode, current_map, map_dir, name=None, map_name=None):
    """Check an app request and say what to do. -> dict:
        {'save_as': <name> | None,  'mode': ..., 'map': <file or ''>, 'say': text for the app}
    'save_as' set = save the live map under that name first (its .pbstream becomes 'map').
    Raises ValueError with a message for the app when the request makes no sense right now."""
    if action not in ACTIONS:
        raise ValueError(f'action must be one of {ACTIONS}')
    maps = {m['name']: m['file'] for m in list_maps(map_dir)}
    if action == 'freeze':
        if current_mode != 'mapping':
            raise ValueError('The map is already frozen. Use "Continue mapping" to change it.')
        n = safe_map_name(name) or default_name()
        return {'save_as': n, 'mode': 'localization', 'map': os.path.join(os.path.expanduser(map_dir), n + '.pbstream'),
                'say': f'Saving the map as {n} and freezing it. The robot restarts in about 20 s; '
                       'then check the position and tap Position OK.'}
    if action == 'new':
        return {'save_as': None, 'mode': 'mapping', 'map': '',
                'say': 'Starting a new, empty map. The robot restarts in about 20 s. Places saved on the old '
                       'map will not match the new one.'}
    if action == 'continue':
        f = maps.get(safe_map_name(map_name)) if map_name else (current_map if current_map else None)
        if not f or not os.path.isfile(f):
            raise ValueError('No saved map to continue. Freeze or save a map first, or start a new one.')
        return {'save_as': None, 'mode': 'mapping', 'map': f,
                'say': f'Mapping again on top of {os.path.basename(f)[:-9]}. The robot restarts in about 20 s. '
                       'Drive slowly into the new areas, then freeze it again.'}
    # use
    f = maps.get(safe_map_name(map_name)) if map_name else None
    if not f:
        raise ValueError('Choose one of the saved maps.')
    return {'save_as': None, 'mode': 'localization', 'map': f,
            'say': f'Driving on {os.path.basename(f)[:-9]} (frozen). The robot restarts in about 20 s; '
                   'then check the position and tap Position OK.'}


class PeriodicMapSaver:
    """Saves the live map while mapping, so a crash, a hot Pi or a flat battery never loses a whole drive.

        saver = PeriodicMapSaver(save_fn, period_s=180)      # save_fn(name) -> text, may raise ValueError
        saver.tick(now, mapping=True, map_version=v, hot=False)   # call every few seconds; starts a save when due

    * Every period_s while mapping, but only when the map changed since the last save (standing still = no save).
    * At once when the Pi gets hot (hot=True), at most once a minute: heat is what killed the 16 Nov drive.
    * One run name per robot start (autosave_<date>_<time>), and every save is its own snapshot
      (<run>_s<HHMM>), so neither a restart nor a map that goes bad late in a drive overwrites the good map
      before it (20 Nov: the department map distorted at the end and the only autosave was overwritten).
      Kept: the newest `snapshots` of the running drive, the newest snapshot of the newest `keep` drives,
      and the biggest file. The app lists them with the other maps; "Continue mapping" picks up from there.
    * The save runs in a thread (Cartographer takes a few seconds to write); never two at once.
    """

    RUN_LEN = len('autosave_2026-11-20_0903')

    def __init__(self, save_fn, period_s=180.0, map_dir='~/maps', keep=3, clock=time.localtime, threaded=True,
                 snapshots=5):
        self.save_fn, self.period_s, self.keep = save_fn, float(period_s), int(keep)
        self.snapshots = max(1, int(snapshots))
        self.clock = clock
        self.map_dir = os.path.expanduser(map_dir)
        self.name = time.strftime('autosave_%Y-%m-%d_%H%M', clock())     # the run; each save adds _s<HHMM>
        self.threaded = threaded
        self._last_t = None            # monotonic time of the last save start
        self._last_hot_t = -1e9
        self._last_version = None
        self._busy = False
        self._first_ref = None         # first tick: the first save comes half a period after it
        self.info = {'state': 'idle', 'name': self.name, 'saved_at': None, 'error': None}

    def due(self, now, mapping, map_version, hot=False):
        if not mapping or self._busy or map_version is None or map_version == self._last_version:
            return False
        if hot and now - self._last_hot_t >= 60.0:
            return True
        if self._last_t is None:
            return now - self._first_ref >= self.period_s * 0.5
        return now - self._last_t >= self.period_s

    def tick(self, now, mapping, map_version, hot=False):
        if self._first_ref is None:
            self._first_ref = now
        if self.period_s <= 0 or not self.due(now, mapping, map_version, hot):
            return False
        if hot:
            self._last_hot_t = now
        self._last_t, self._last_version, self._busy = now, map_version, True
        self.info['state'] = 'saving'
        if self.threaded:
            import threading
            threading.Thread(target=self._save, daemon=True).start()
        else:
            self._save()
        return True

    def snapshot_name(self):
        base = self.name + time.strftime('_s%H%M', self.clock())
        n, i = base, 2
        while os.path.exists(os.path.join(self.map_dir, n + '.pbstream')):   # two saves in one minute
            n, i = '%s_%d' % (base, i), i + 1
        return n

    def _save(self):
        try:
            name = self.snapshot_name()
            text = self.save_fn(name)
            ok = 'WARNING' not in str(text)
            self.info.update(name=name, state='saved' if ok else 'partial', saved_at=int(time.time()),
                             error=None if ok else 'picture saved, Cartographer state not (is SLAM running?)')
            self.prune()
        except Exception as e:                          # never take the dashboard down for an autosave
            self.info.update(state='error', error=str(e)[:200])
        finally:
            self._busy = False

    def prune(self):
        """Delete autosaves older than the newest `keep` (all their files: .pbstream .pgm .yaml).

        The biggest autosave is always kept: a few quick restarts each start a new, nearly empty autosave,
        and they must never push out the one long drive (18 Nov: three restarts within 20 minutes)."""
        try:
            names = sorted({n.split('.')[0] for n in os.listdir(self.map_dir) if n.startswith('autosave_')},
                           reverse=True)
        except OSError:
            return

        def size(n):
            try:
                return os.path.getsize(os.path.join(self.map_dir, n + '.pbstream'))
            except OSError:
                return -1
        biggest = max(names, key=size) if names else None
        keep, runs = {biggest}, []
        for n in names:                                   # newest first
            run = n[:self.RUN_LEN]
            if run == self.name:
                if sum(1 for k in keep if k and k[:self.RUN_LEN] == run) < self.snapshots:
                    keep.add(n)
                if run not in runs:
                    runs.append(run)
            elif run not in runs:
                runs.append(run)
                if len(runs) <= self.keep:
                    keep.add(n)                           # newest snapshot of a recent drive
        for old in names:
            if old in keep:
                continue
            for ext in ('.pbstream', '.pgm', '.yaml'):
                try:
                    os.remove(os.path.join(self.map_dir, old + ext))
                except OSError:
                    pass
