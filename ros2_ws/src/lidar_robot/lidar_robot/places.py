"""Named places on the saved map ("HOD office", "Lab 3", ...) + fuzzy search (pure Python, tested).

Stored as JSON, e.g. ~/maps/places.json:
{
  "version": 1, "frame": "map", "map": "cse_floor",
  "home": {"x": 0.0, "y": 0.0, "yaw": 0.0},
  "places": [{"name": "HOD office", "aliases": ["hod", "head of department"],
              "x": 12.3, "y": 4.5, "yaw": 1.57, "note": "door on the left"}]
}
Rooms (room-level semantics: "this room contains that", no per-object positions):
  "rooms": [{"name": "Lab 3", "polygon": [[x, y], ...], "contents": ["3D printer", "oscilloscope"],
             "aliases": ["computer lab"]}]
  "go to the room with the 3D printer" -> drives to the room's centre (or its door place if one
  has the same name). Contents are typed in the app now; the camera can suggest them later.

Coordinates are in the map frame, so they only stay valid while you localise on the SAME saved
map (bringup slam_mode:=localization). A fresh mapping run starts a new map frame.
"""
import difflib
import json
import os
import re
import tempfile
import time

STOPWORDS = {'the', 'a', 'an', 'to', 'of', 'room', 'please', 'go', 'take', 'me', 'us', 'at',
             'in', 'near', 'by', 'for', 'and', 'then', 'robot', 'my', 'our', 'this', 'that'}


def normalize(text):
    text = text.lower().replace('&', ' and ')
    text = re.sub(r'[^a-z0-9 ]+', ' ', text)
    return ' '.join(text.split())


def tokens(text):
    return [t for t in normalize(text).split() if t not in STOPWORDS]


class PlaceStore:
    def __init__(self, path=None):
        self.path = path
        self.data = {'version': 1, 'frame': 'map', 'map': '', 'home': None, 'places': [], 'rooms': []}
        self.version = 0
        if path and os.path.isfile(path):
            self.load()

    # ---------------------------------------------------------------- persistence
    def load(self):
        with open(self.path) as f:
            d = json.load(f)
        d.setdefault('places', [])
        d.setdefault('home', None)
        d.setdefault('rooms', [])
        self.data = d
        self.version += 1

    def save(self):
        if not self.path:
            return
        os.makedirs(os.path.dirname(os.path.abspath(self.path)), exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=os.path.dirname(os.path.abspath(self.path)), suffix='.tmp')
        with os.fdopen(fd, 'w') as f:
            json.dump(self.data, f, indent=2)
        os.replace(tmp, self.path)          # atomic: never leaves a half-written file

    # ---------------------------------------------------------------- edits
    @property
    def places(self):
        return self.data['places']

    @property
    def home(self):
        return self.data.get('home')

    def get(self, name):
        n = normalize(name)
        for p in self.places:
            if normalize(p['name']) == n:
                return p
        return None

    def add(self, name, x, y, yaw=0.0, aliases=(), note=''):
        name = name.strip()
        if not name:
            raise ValueError('place name is empty')
        if len(name) > 60:
            raise ValueError('place name too long')
        rec = {'name': name, 'aliases': [a.strip() for a in aliases if a.strip()],
               'x': round(float(x), 3), 'y': round(float(y), 3), 'yaw': round(float(yaw), 4),
               'note': note, 'created': int(time.time())}
        old = self.get(name)
        if old:
            self.places.remove(old)
        self.places.append(rec)
        self.version += 1
        self.save()
        return rec

    def remove(self, name):
        p = self.get(name)
        if not p:
            raise ValueError(f'no place called "{name}"')
        self.places.remove(p)
        self.version += 1
        self.save()

    def set_home(self, x, y, yaw=0.0):
        self.data['home'] = {'x': round(float(x), 3), 'y': round(float(y), 3), 'yaw': round(float(yaw), 4)}
        self.version += 1
        self.save()

    # ---------------------------------------------------------------- search
    def search(self, query, limit=5, min_score=0.45):
        """Ranked [(score, place)] for free text like 'hod', 'lab three', 'principal office'."""
        q = normalize(query)
        qt = set(tokens(query))
        if not q:
            return []
        out = []
        for p in self.places:
            best = 0.0
            for label in [p['name']] + list(p.get('aliases', [])):
                n = normalize(label)
                nt = set(tokens(label))
                if q == n:
                    s = 1.0
                elif n and (f' {n} ' in f' {q} ' or f' {q} ' in f' {n} '):
                    s = 0.92                       # whole-word containment either way
                else:
                    jacc = len(qt & nt) / len(qt | nt) if (qt | nt) else 0.0
                    ratio = difflib.SequenceMatcher(None, q, n).ratio()
                    tok_ratio = max((difflib.SequenceMatcher(None, a, b).ratio()
                                     for a in qt for b in nt), default=0.0)
                    s = max(0.85 * jacc + 0.1, ratio * 0.9, tok_ratio * 0.8 if len(qt) == 1 else 0)
                best = max(best, s)
            if best >= min_score:
                out.append((round(best, 3), p))
        out.sort(key=lambda t: (-t[0], t[1]['name']))
        return out[:limit]

    def find_in_text(self, text):
        """Places mentioned inside a longer sentence, in the order they appear."""
        nt = normalize(text)
        found = []
        for p in self.places:
            for label in [p['name']] + list(p.get('aliases', [])):
                n = normalize(label)
                if not n:
                    continue
                m = re.search(r'\b' + re.escape(n) + r'\b', nt)
                if m:
                    found.append((m.start(), -len(n), p))
                    break
        for r in self.rooms:                         # rooms by name ("go to the workshop")
            if self.get(r['name']):
                continue                             # a place with the same name already matched
            for label in [r['name']] + list(r.get('aliases', [])):
                n = normalize(label)
                m = re.search(r'\b' + re.escape(n) + r'\b', nt) if n else None
                if m:
                    found.append((m.start(), -len(n), self.room_target(r)))
                    break
        found.sort(key=lambda t: (t[0], t[1]))
        out, seen = [], set()
        for _, _, p in found:
            if p['name'] not in seen:
                out.append(p)
                seen.add(p['name'])
        return out

    # ---------------------------------------------------------------- rooms
    @property
    def rooms(self):
        return self.data.setdefault('rooms', [])

    def get_room(self, name):
        n = normalize(name)
        for r in self.rooms:
            if normalize(r['name']) == n or n in [normalize(a) for a in r.get('aliases', [])]:
                return r
        return None

    def add_room(self, name, polygon, contents=(), aliases=()):
        name = (name or '').strip()
        if not name or len(name) > 60:
            raise ValueError('room name must be 1-60 characters')
        poly = [[round(float(x), 3), round(float(y), 3)] for x, y in polygon]
        if len(poly) < 3 or abs(polygon_area(poly)) < 0.5:
            raise ValueError('draw the room as an area of at least 0.5 m²')
        rec = {'name': name, 'polygon': poly,
               'contents': _clean_list(contents), 'aliases': _clean_list(aliases)}
        old = self.get_room(name)
        if old:
            self.rooms.remove(old)
        self.rooms.append(rec)
        self.version += 1
        self.save()
        return rec

    def update_room(self, name, contents=None, aliases=None, new_name=None):
        r = self.get_room(name)
        if not r:
            raise ValueError(f'no room called "{name}"')
        if contents is not None:
            r['contents'] = _clean_list(contents)
        if aliases is not None:
            r['aliases'] = _clean_list(aliases)
        if new_name:
            r['name'] = new_name.strip()[:60]
        self.version += 1
        self.save()
        return r

    def remove_room(self, name):
        r = self.get_room(name)
        if not r:
            raise ValueError(f'no room called "{name}"')
        self.rooms.remove(r)
        self.version += 1
        self.save()

    def room_at(self, x, y):
        best = None
        for r in self.rooms:
            if point_in_polygon(x, y, r['polygon']):
                if best is None or abs(polygon_area(r['polygon'])) < abs(polygon_area(best['polygon'])):
                    best = r                 # smallest room wins (a lab inside a "first floor" area)
        return best

    def find_thing(self, query, limit=5, min_score=0.6):
        """Rooms whose contents match free text: [(score, room, item)]."""
        q = normalize(query)
        qt = set(tokens(query))
        if not q:
            return []
        out = []
        for r in self.rooms:
            best, item = 0.0, None
            for thing in r.get('contents', []):
                n = normalize(thing)
                nt = set(tokens(thing))
                if not n:
                    continue
                if q == n:
                    s = 1.0
                elif f' {n} ' in f' {q} ':
                    s = 0.95
                elif f' {q} ' in f' {n} ':
                    s = 0.9
                else:
                    jacc = len(qt & nt) / len(qt | nt) if (qt | nt) else 0.0
                    s = max(0.85 * jacc + 0.1 if jacc else 0.0,
                            difflib.SequenceMatcher(None, q, n).ratio() * 0.9)
                if s > best:
                    best, item = s, thing
            if best >= min_score:
                out.append((round(best, 3), r, item))
        out.sort(key=lambda t: -t[0])
        return out[:limit]

    def room_target(self, room):
        """Where to drive for a room: a place with the same name, else the room's centre."""
        p = self.get(room['name'])
        if p:
            return {'name': p['name'], 'x': p['x'], 'y': p['y'], 'yaw': p.get('yaw')}
        cx, cy = polygon_centroid(room['polygon'])
        return {'name': room['name'], 'x': round(cx, 3), 'y': round(cy, 3), 'yaw': None, 'room': True}

    def target(self, name):
        """A place, or a room (as a drivable target), by exact name."""
        p = self.get(name)
        if p:
            return p
        r = self.get_room(name)
        return self.room_target(r) if r else None

    def summary(self):
        return {'map': self.data.get('map', ''), 'home': self.home,
                'places': [{k: p[k] for k in ('name', 'aliases', 'x', 'y', 'yaw', 'note') if k in p}
                           for p in self.places],
                'rooms': self.rooms,
                'dock': self.data.get('dock'),
                'version': self.version}

    # ---------------------------------------------------------------- dock
    @property
    def dock(self):
        return self.data.get('dock')

    def set_dock(self, x, y, yaw):
        self.data['dock'] = {'x': round(float(x), 3), 'y': round(float(y), 3), 'yaw': round(float(yaw), 4)}
        self.version += 1
        self.save()


def _clean_list(items):
    if isinstance(items, str):
        items = items.split(',')
    out = []
    for it in items or ():
        it = str(it).strip()[:60]
        if it and it.lower() not in [o.lower() for o in out]:
            out.append(it)
    return out[:50]


def polygon_area(poly):
    return 0.5 * sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]))


def polygon_centroid(poly):
    a = polygon_area(poly)
    if abs(a) < 1e-9:
        return (sum(p[0] for p in poly) / len(poly), sum(p[1] for p in poly) / len(poly))
    cx = cy = 0.0
    for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
        f = x0 * y1 - x1 * y0
        cx += (x0 + x1) * f
        cy += (y0 + y1) * f
    return cx / (6 * a), cy / (6 * a)


def point_in_polygon(x, y, poly):
    inside = False
    for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
        if (y0 > y) != (y1 > y) and x < (x1 - x0) * (y - y0) / (y1 - y0) + x0:
            inside = not inside
    return inside
