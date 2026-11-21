"""Places, rooms, home and dock belong to ONE map (21 Nov 2026).

Their x/y are map-frame coordinates, so they are only right on the map they were made on. Before this, a new map
(e.g. the exhibition hall) started with the department's places and rooms floating over it. Now every map keeps
its own set:

  ~/maps/places.json              the set in use (everything else keeps reading this one file)
  ~/maps/annotations/<map>.json   the sets of the other maps (<map> = the .pbstream name without .pbstream)

The launch calls switch_places() before any node starts, with the map it is about to run:
  - same map as last time -> nothing happens
  - another saved map     -> the current set is put away under its map, the new map's set comes back
                             (or an empty set, the first time on that map)
  - New map (mapping)     -> starts empty. Places marked while mapping go with the map when it is frozen
  - places.json without an owner (made before this change) -> it belongs to the map in use now
Nothing is ever deleted: a set that is replaced is kept in annotations/.
"""
import json
import os
import tempfile
import time

NEW = '_new_map'          # the map being built right now (no name until it is frozen)


def map_key(map_file, slam_mode):
    """Which set this run needs: the saved map's name, NEW for a fresh mapping run, None = leave it alone."""
    if map_file:
        n = os.path.basename(str(map_file))
        return n[:-9] if n.endswith('.pbstream') else (os.path.splitext(n)[0] or None)
    return NEW if slam_mode == 'mapping' else None


def _read(path):
    try:
        with open(path) as f:
            d = json.load(f)
        return d if isinstance(d, dict) else None
    except (OSError, ValueError):
        return None


def _write(path, d):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(os.path.abspath(path)), suffix='.tmp')
    with os.fdopen(fd, 'w') as f:
        json.dump(d, f, indent=2)
    os.replace(tmp, path)


def empty(key):
    return {'version': 1, 'frame': 'map', 'map': key, 'home': None, 'places': [], 'rooms': []}


def count(d):
    return len((d or {}).get('places') or []) + len((d or {}).get('rooms') or [])


def store_dir(places_path):
    return os.path.join(os.path.dirname(os.path.abspath(places_path)), 'annotations')


def switch_places(places_path, key, clock=time.time):
    """Make places_path hold the set for map `key`. Returns a one-line note for the log ('' = no change)."""
    if not key or not places_path:
        return ''
    cur = _read(places_path) if os.path.isfile(places_path) else None
    owner = str((cur or {}).get('map') or '')
    if cur is not None and owner == key:
        return ''
    if cur is not None and not owner:                       # made before 21 Nov: it belongs to the map in use now
        cur['map'] = key
        _write(places_path, cur)
        return f'places: {count(cur)} places/rooms now belong to map {key}'
    sdir = store_dir(places_path)
    saved_path = os.path.join(sdir, key + '.json')
    if cur is not None and count(cur) or (cur or {}).get('home') or (cur or {}).get('dock'):
        # put the current set away under its own map (a NEW set gets a time stamp: never restored by itself)
        name = owner if owner != NEW else f'{NEW}_{time.strftime("%Y%m%d_%H%M", time.localtime(clock()))}'
        if owner == NEW and key != NEW and not os.path.isfile(saved_path):
            cur['map'] = key                                 # the map just built was frozen as `key`: keep its places
            _write(places_path, cur)
            return f'places: the {count(cur)} places/rooms marked while mapping now belong to map {key}'
        _write(os.path.join(sdir, name + '.json'), cur)
    nxt = _read(saved_path) if key != NEW else None
    if nxt is None:
        nxt = empty(key)
        note = f'places: map {key} has no places yet (the previous set is kept in {sdir})'
    else:
        nxt['map'] = key
        note = f'places: {count(nxt)} places/rooms loaded for map {key}'
    _write(places_path, nxt)
    return note


def copy_set(places_path, src_key):
    """Bring another map's set into the set in use (same frame only: a snapshot / continued version of that map).
    Places and rooms with a name already in use are skipped. Returns how many were added."""
    src = _read(os.path.join(store_dir(places_path), src_key + '.json'))
    cur = _read(places_path) or empty('')
    if not src:
        return 0
    have = {p.get('name', '').lower() for p in cur.get('places') or []}
    rooms = {r.get('name', '').lower() for r in cur.get('rooms') or []}
    n = 0
    for p in src.get('places') or []:
        if p.get('name', '').lower() not in have:
            cur.setdefault('places', []).append(p)
            n += 1
    for r in src.get('rooms') or []:
        if r.get('name', '').lower() not in rooms:
            cur.setdefault('rooms', []).append(r)
            n += 1
    for k in ('home', 'dock'):
        if not cur.get(k) and src.get(k):
            cur[k] = src[k]
    _write(places_path, cur)
    return n


def list_sets(places_path):
    sdir = store_dir(places_path)
    out = []
    if os.path.isdir(sdir):
        for f in sorted(os.listdir(sdir)):
            if f.endswith('.json'):
                out.append((f[:-5], count(_read(os.path.join(sdir, f)))))
    return out


if __name__ == '__main__':          # python3 -m lidar_robot.annotations [list | copy <map>]
    import sys
    pp = os.path.expanduser('~/maps/places.json')
    if len(sys.argv) >= 3 and sys.argv[1] == 'copy':
        print(f'added {copy_set(pp, sys.argv[2])} places/rooms from {sys.argv[2]}; restart the robot to use them')
    else:
        cur = _read(pp)
        print(f'in use: map {(cur or {}).get("map") or "?"}, {count(cur)} places/rooms')
        for k, n in list_sets(pp):
            print(f'  kept: {k}  ({n} places/rooms)')
