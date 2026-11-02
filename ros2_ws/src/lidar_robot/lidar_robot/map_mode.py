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
    return {'mode': d['mode'], 'map': os.path.expanduser(m) if m else '', 'since': d.get('since')}


def write_mode(path, mode, map_file='', clock=time.time):
    if mode not in MODES:
        raise ValueError(f'mode must be one of {MODES}')
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump({'mode': mode, 'map': map_file or '', 'since': int(clock())}, f)
    os.replace(tmp, path)


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
