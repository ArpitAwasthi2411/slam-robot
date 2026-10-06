"""Named places on the saved map ("HOD office", "Lab 3", ...) + fuzzy search (pure Python, tested).

Stored as JSON, e.g. ~/maps/places.json:
{
  "version": 1, "frame": "map", "map": "cse_floor",
  "home": {"x": 0.0, "y": 0.0, "yaw": 0.0},
  "places": [{"name": "HOD office", "aliases": ["hod", "head of department"],
              "x": 12.3, "y": 4.5, "yaw": 1.57, "note": "door on the left"}]
}
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
        self.data = {'version': 1, 'frame': 'map', 'map': '', 'home': None, 'places': []}
        self.version = 0
        if path and os.path.isfile(path):
            self.load()

    # ---------------------------------------------------------------- persistence
    def load(self):
        with open(self.path) as f:
            d = json.load(f)
        d.setdefault('places', [])
        d.setdefault('home', None)
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
        found.sort(key=lambda t: (t[0], t[1]))
        out, seen = [], set()
        for _, _, p in found:
            if p['name'] not in seen:
                out.append(p)
                seen.add(p['name'])
        return out

    def summary(self):
        return {'map': self.data.get('map', ''), 'home': self.home,
                'places': [{k: p[k] for k in ('name', 'aliases', 'x', 'y', 'yaw', 'note') if k in p}
                           for p in self.places],
                'version': self.version}
