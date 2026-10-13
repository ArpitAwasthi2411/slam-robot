"""Scheduled missions: "patrol at 10:00 on weekdays" (pure Python, tested).

Stored next to places.json as schedules.json:
  [{"id": 3, "name": "Morning patrol", "at": "10:00", "days": [0, 1, 2, 3, 4],
    "action": {"type": "goto_place", "name": "Lab 3"}, "enabled": true, "last": "2026-10-13 10:00"}]
days: 0 = Monday ... 6 = Sunday (empty = every day). The robot uses its own clock (the Pi keeps
time over NTP when it has internet; set it once after a power cut without internet).
A schedule only fires within `grace_min` minutes of its time: if the robot was off at 10:00 it
does not suddenly run the 10:00 patrol at 15:00.
"""
import json
import os
import re
import tempfile
import time

ALLOWED = ('goto_place', 'goto_poses', 'go_home', 'command', 'explore_start', 'follow_path')
DAY_NAMES = ['Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat', 'Sun']


def _parse_at(at):
    m = re.fullmatch(r'\s*(\d{1,2}):(\d{2})\s*', str(at or ''))
    if not m or int(m.group(1)) > 23 or int(m.group(2)) > 59:
        raise ValueError('time must look like 09:30 (24-hour)')
    return int(m.group(1)), int(m.group(2))


class ScheduleStore:
    def __init__(self, path=None, grace_min=2, localtime=time.localtime):
        self.path = path
        self.grace = grace_min
        self.localtime = localtime
        self.items = []
        self.version = 0
        if path and os.path.isfile(path):
            try:
                with open(path) as f:
                    self.items = json.load(f)
            except (OSError, ValueError):
                self.items = []

    def save(self):
        self.version += 1
        if not self.path:
            return
        d = os.path.dirname(os.path.abspath(self.path))
        os.makedirs(d, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=d, suffix='.tmp')
        with os.fdopen(fd, 'w') as f:
            json.dump(self.items, f, indent=2)
        os.replace(tmp, self.path)

    def add(self, name, at, action, days=None, enabled=True, sid=None):
        _parse_at(at)
        if not isinstance(action, dict) or action.get('type') not in ALLOWED:
            raise ValueError('action must be one of: ' + ', '.join(ALLOWED))
        days = sorted({int(d) for d in (days or [])})
        if any(not 0 <= d <= 6 for d in days):
            raise ValueError('days are 0 (Mon) .. 6 (Sun)')
        name = (name or '').strip()[:60] or action.get('name') or action.get('text') or action['type']
        if sid is not None:
            self.items = [s for s in self.items if s['id'] != sid]
        new_id = sid if sid is not None else max([s['id'] for s in self.items], default=0) + 1
        rec = {'id': new_id, 'name': name, 'at': '%02d:%02d' % _parse_at(at), 'days': days,
               'action': action, 'enabled': bool(enabled), 'last': None}
        self.items.append(rec)
        self.items.sort(key=lambda s: s['at'])
        self.save()
        return rec

    def get(self, sid):
        for s in self.items:
            if s['id'] == int(sid):
                return s
        raise ValueError(f'no schedule #{sid}')

    def remove(self, sid):
        self.get(sid)
        self.items = [s for s in self.items if s['id'] != int(sid)]
        self.save()

    def set_enabled(self, sid, on):
        s = self.get(sid)
        s['enabled'] = bool(on)
        self.save()
        return s

    def due(self):
        """Schedules to run now (marks them as run)."""
        t = self.localtime()
        now_min = t.tm_hour * 60 + t.tm_min
        today = time.strftime('%Y-%m-%d', t)
        out = []
        for s in self.items:
            if not s.get('enabled'):
                continue
            if s['days'] and t.tm_wday not in s['days']:
                continue
            h, m = _parse_at(s['at'])
            start = h * 60 + m
            if not 0 <= now_min - start <= self.grace:
                continue
            stamp = f'{today} {s["at"]}'
            if s.get('last') == stamp:
                continue
            s['last'] = stamp
            out.append(s)
        if out:
            self.save()
        return out

    def summary(self):
        return [{**s, 'days_text': 'every day' if not s['days'] else
                 ('weekdays' if s['days'] == [0, 1, 2, 3, 4] else ', '.join(DAY_NAMES[d] for d in s['days']))}
                for s in self.items]
