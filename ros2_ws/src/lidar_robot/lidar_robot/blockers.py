"""What is in the way: a person or an object? (pure Python, unit-tested)

When the robot's front is blocked, what it should do depends on what blocks it:

  a person   -> stop, ask once "Excuse me, may I pass?", give them time to step aside (person_wait_s),
                say "Thank you" when the way clears; only if they stay, look for another route
  an object  -> a box or a chair will not move: look for another route at once (object_wait_s)
  unknown    -> no camera picture (camera off, phone not mounted, detector resting): behave as before
                (ask to make way, replan after the follower's own blocked_after)

"A person" = the camera's people detector (on the Pi or on the mounted phone) reports someone in the path
within person_range_m while the front is blocked. The decision is taken over about a second, not on one
frame, because the detector runs a few times a second: an object is only called an object after
object_wait_s without any person in view. Being wrong is safe either way: the robot never drives through
anything, it only chooses between waiting and going around.
"""

PERSON, OBJECT, UNKNOWN = 'person', 'object', 'unknown'


class BlockerWatch:
    def __init__(self, enabled=True, person_wait_s=8.0, object_wait_s=1.2, person_range_m=2.5, fresh_s=1.5,
                 ask_text='Excuse me, may I pass?', thanks_text='Thank you.'):
        self.enabled = enabled
        self.person_wait_s = person_wait_s      # how long a person gets to step aside before a new route
        self.object_wait_s = object_wait_s      # how long "nobody in view" must hold to call it an object
        self.person_range_m = person_range_m
        self.fresh_s = fresh_s                  # an older camera decision does not count
        self.ask_text, self.thanks_text = ask_text, thanks_text
        self._people = None                     # (t, person_in_path: bool)
        self._since = None                      # when this block began
        self.kind = None                        # PERSON | OBJECT | UNKNOWN while blocked, else None
        self._asked = False
        self.counts = {PERSON: 0, OBJECT: 0, UNKNOWN: 0}
        self.lost_s = {PERSON: 0.0, OBJECT: 0.0, UNKNOWN: 0.0}
        self.last = None                        # {'kind', 's'} of the block that just ended

    # ---------------------------------------------------------------- camera
    def see_people(self, decision, now):
        """decision: vision_rules.decide() output (or None when the camera has nothing to say)."""
        if not isinstance(decision, dict) or 'no camera' in str(decision.get('reason', '')):
            self._people = None                 # no picture is not the same as "nobody there"
            return
        c = decision.get('closest') or {}
        near = c.get('distance_m') is None or c['distance_m'] <= self.person_range_m
        in_path = bool(decision.get('people_in_path')) or bool(c.get('in_path'))
        self._people = (now, in_path and near)

    def _camera(self, now):
        """True = person in the path, False = camera sees no one there, None = no fresh camera decision."""
        if self._people is None or now - self._people[0] > self.fresh_s:
            return None
        return self._people[1]

    # ---------------------------------------------------------------- every control tick
    def update(self, blocked, now):
        """-> {'kind', 'limit', 'say': [(key, text)], 'ended': {...}|None}

        limit: seconds the follower may stay blocked before it reports BLOCKED (None = its own setting)."""
        out = {'kind': None, 'limit': None, 'say': [], 'ended': None}
        if not self.enabled:
            return out
        if not blocked:
            if self._since is not None:
                s = now - self._since
                kind = self.kind or UNKNOWN
                if s >= 0.5:                                    # a real block, not one flickering reading
                    self.counts[kind] += 1
                    self.lost_s[kind] += s
                    self.last = out['ended'] = {'kind': kind, 's': round(s, 1)}
                    if kind == PERSON and self._asked:
                        out['say'].append(('thanks', self.thanks_text))
            self._since, self.kind, self._asked = None, None, False
            return out
        if self._since is None:
            self._since = now
        held = now - self._since
        cam = self._camera(now)
        if cam is True:
            self.kind = PERSON                                   # once a person, a person for this block
        elif self.kind != PERSON:
            self.kind = UNKNOWN if cam is None else (OBJECT if held >= self.object_wait_s else None)
        if self.kind == PERSON:
            out['limit'] = self.person_wait_s
            if not self._asked and held >= 0.4:
                self._asked = True
                out['say'].append(('make_way', self.ask_text))
        elif self.kind == OBJECT:
            out['limit'] = self.object_wait_s                    # already held that long: replan now
        elif self.kind == UNKNOWN:
            if not self._asked and held >= 1.5:                  # as before: we cannot tell, so ask anyway
                self._asked = True
                out['say'].append(('make_way', self.ask_text))
        else:
            out['limit'] = self.object_wait_s + 0.6              # still deciding: do not replan under it
        out['kind'] = self.kind
        return out

    def replanned(self, now):
        """The follower gave up on this block and the core is finding another route."""
        if self._since is not None:
            kind = self.kind or UNKNOWN
            self.counts[kind] += 1
            self.lost_s[kind] += now - self._since
            self.last = {'kind': kind, 's': round(now - self._since, 1)}
        ended = self.last
        self._since, self.kind, self._asked = None, None, False
        return ended

    def summary(self):
        return {'now': self.kind, 'counts': dict(self.counts),
                'lost_s': {k: round(v, 1) for k, v in self.lost_s.items()}, 'last': self.last}
