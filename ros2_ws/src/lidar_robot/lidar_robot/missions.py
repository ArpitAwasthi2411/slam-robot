"""Mission queue: priority scheduling of multi-stop trips (pure Python, tested).

A Mission is an ordered list of stops. Higher priority runs first; equal priority is FIFO.
An urgent (priority 3) mission pre-empts a running lower-priority one; the interrupted mission
goes back to the queue and resumes from the stop it was heading to.
"""
import itertools
import time
from dataclasses import dataclass, field
from typing import List, Optional

_ids = itertools.count(1)


@dataclass
class Stop:
    label: str
    x: float
    y: float
    yaw: Optional[float] = None


@dataclass
class Mission:
    stops: List[Stop]
    priority: int = 1
    source: str = 'user'            # dashboard, command, rviz, auto-home
    text: str = ''                  # original command, for the log
    id: int = field(default_factory=lambda: next(_ids))
    created: float = field(default_factory=time.time)
    next_stop: int = 0
    state: str = 'QUEUED'           # QUEUED, ACTIVE, DONE, FAILED, CANCELED
    attempts: int = 0
    message: str = ''
    repeat: int = 0                 # extra rounds after this one; -1 = patrol until canceled
    round: int = 1
    skip_unreachable: bool = False  # routes: skip a point that can't be reached instead of failing
    skips: int = 0                  # consecutive skipped points (all skipped in a row = fail)

    @property
    def current(self):
        return self.stops[self.next_stop] if self.next_stop < len(self.stops) else None

    def summary(self):
        return {'id': self.id, 'priority': self.priority, 'source': self.source, 'text': self.text,
                'state': self.state, 'message': self.message,
                'stops': [s.label for s in self.stops], 'next_stop': self.next_stop,
                'repeat': self.repeat, 'round': self.round}


class MissionQueue:
    def __init__(self, history=20):
        self.queue: List[Mission] = []
        self.active: Optional[Mission] = None
        self.history: List[Mission] = []
        self._hist_n = history

    def add(self, mission):
        """Returns True if this mission should pre-empt the active one."""
        self.queue.append(mission)
        self.queue.sort(key=lambda m: (-m.priority, m.created, m.id))
        return bool(self.active and mission.priority >= 3 and mission.priority > self.active.priority)

    def next(self):
        """Pop the highest-priority mission and make it active."""
        if self.active is None and self.queue:
            self.active = self.queue.pop(0)
            self.active.state = 'ACTIVE'
        return self.active

    def preempt(self):
        if self.active:
            self.active.state = 'QUEUED'
            self.active.message = 'paused for an urgent task'
            self.queue.append(self.active)
            self.queue.sort(key=lambda m: (-m.priority, m.created, m.id))
            self.active = None

    def finish(self, state, message=''):
        if self.active:
            self.active.state = state
            self.active.message = message
            self.history.insert(0, self.active)
            del self.history[self._hist_n:]
            self.active = None

    def cancel(self, mission_id=None):
        """Cancel one queued/active mission (by id) or everything (None). Returns count."""
        n = 0
        if mission_id is None or (self.active and self.active.id == mission_id):
            if self.active:
                self.finish('CANCELED', 'canceled by user')
                n += 1
        keep = []
        for m in self.queue:
            if mission_id is None or m.id == mission_id:
                m.state = 'CANCELED'
                self.history.insert(0, m)
                n += 1
            else:
                keep.append(m)
        self.queue = keep
        del self.history[self._hist_n:]
        return n

    def summary(self):
        return {'active': self.active.summary() if self.active else None,
                'queue': [m.summary() for m in self.queue],
                'history': [m.summary() for m in self.history[:8]]}
