"""Mission log for evaluation: one JSON line per finished mission (pure Python, tested).

~/maps/missions.jsonl, fields per mission:
  id, text, source, priority, stops, started, ended, duration_s, result (DONE/FAILED/CANCELED),
  reason, distance_m (driven), planned_m (first route to each stop), straight_m (straight lines),
  replans, blocked_s, estops, bumps, skipped, min_gap_m (closest the body came to anything)

stats() gives the numbers for the project report: success rate, average time, path efficiency
(planned / driven), replans and bumps per mission.
"""
import json
import math
import os
import time

KEEP = 300


class MissionLog:
    def __init__(self, path=None, clock=time.time):
        self.path = path
        self.clock = clock
        self.open = {}               # mission id -> record being filled in
        self.done = []               # newest last
        self._last_pose = None
        if path and os.path.isfile(path):
            try:
                with open(path) as f:
                    for line in f.readlines()[-KEEP:]:
                        line = line.strip()
                        if line:
                            self.done.append(json.loads(line))
            except (OSError, ValueError):
                self.done = []

    # ------------------------------------------------------------ recording
    def rec(self, m):
        r = self.open.get(m.id)
        if r is None:
            r = self.open[m.id] = {
                'id': m.id, 'text': m.text, 'source': m.source, 'priority': m.priority,
                'stops': [s.label for s in m.stops], 'started': round(self.clock(), 1),
                'distance_m': 0.0, 'planned_m': 0.0, 'straight_m': 0.0, 'replans': 0, 'blocked_s': 0.0,
                'estops': 0, 'bumps': 0, 'skipped': 0, 'min_gap_m': None, '_planned_stops': set(),
                '_from': None}
        return r

    def on_pose(self, m, pose, dt, blocked=False, gap=None):
        r = self.rec(m)
        if r['_from'] is None:
            r['_from'] = (pose[0], pose[1])
        if self._last_pose is not None:
            d = math.hypot(pose[0] - self._last_pose[0], pose[1] - self._last_pose[1])
            if d < 0.5:                                   # ignore relocalization jumps
                r['distance_m'] += d
        self._last_pose = (pose[0], pose[1])
        if blocked:
            r['blocked_s'] += dt
        if gap is not None and math.isfinite(gap):
            r['min_gap_m'] = gap if r['min_gap_m'] is None else min(r['min_gap_m'], gap)

    def on_plan(self, m, stop_index, length, start, goal):
        r = self.rec(m)
        key = (m.round, stop_index)
        if key not in r['_planned_stops']:
            r['_planned_stops'].add(key)
            r['planned_m'] += length
            r['straight_m'] += math.hypot(goal[0] - start[0], goal[1] - start[1])

    def bump(self, m, field, n=1):
        if m is not None:
            self.rec(m)[field] += n

    def idle(self):
        self._last_pose = None

    def sync(self, queue):
        """Close records of missions that are no longer active or queued."""
        live = {q.id for q in queue.queue}
        if queue.active:
            live.add(queue.active.id)
        for mid in [k for k in self.open if k not in live]:
            m = next((h for h in queue.history if h.id == mid), None)
            self._close(mid, m.state if m else 'CANCELED', m.message if m else '')

    def _close(self, mid, result, reason):
        r = self.open.pop(mid)
        for k in [k for k in r if k.startswith('_')]:
            r.pop(k)
        r['ended'] = round(self.clock(), 1)
        r['duration_s'] = round(r['ended'] - r['started'], 1)
        r['result'] = result
        r['reason'] = reason
        for k in ('distance_m', 'planned_m', 'straight_m', 'blocked_s'):
            r[k] = round(r[k], 2)
        if r['min_gap_m'] is not None:
            r['min_gap_m'] = round(r['min_gap_m'], 3)
        self.done.append(r)
        del self.done[:-KEEP]
        if self.path:
            try:
                os.makedirs(os.path.dirname(os.path.abspath(self.path)), exist_ok=True)
                with open(self.path, 'a') as f:
                    f.write(json.dumps(r) + '\n')
            except OSError:
                pass
        return r

    # ------------------------------------------------------------ reading
    def recent(self, n=50, include_explore=False):
        rows = [r for r in self.done if include_explore or r['source'] != 'explore']
        return rows[-n:][::-1]

    def stats(self, include_explore=False):
        rows = [r for r in self.done if include_explore or r['source'] != 'explore']
        n = len(rows)
        done = [r for r in rows if r['result'] == 'DONE']
        failed = [r for r in rows if r['result'] == 'FAILED']
        eff = [r['planned_m'] / r['distance_m'] for r in done if r['distance_m'] > 0.3 and r['planned_m'] > 0]
        reasons = {}
        for r in failed:
            code = (r.get('reason') or '').split(':')[0] or 'unknown'
            reasons[code] = reasons.get(code, 0) + 1

        def avg(xs):
            return round(sum(xs) / len(xs), 2) if xs else None
        return {
            'missions': n, 'done': len(done), 'failed': len(failed),
            'canceled': n - len(done) - len(failed),
            'success_rate': round(len(done) / (len(done) + len(failed)), 3) if (done or failed) else None,
            'avg_duration_s': avg([r['duration_s'] for r in done]),
            'avg_speed_mps': avg([r['distance_m'] / r['duration_s'] for r in done if r['duration_s'] > 1]),
            'path_efficiency': avg(eff),
            'avg_replans': avg([r['replans'] for r in rows]),
            'total_distance_m': round(sum(r['distance_m'] for r in rows), 1),
            'bumps': sum(r.get('bumps', 0) for r in rows),
            'estops': sum(r.get('estops', 0) for r in rows),
            'min_gap_m': min([r['min_gap_m'] for r in rows if r.get('min_gap_m') is not None], default=None),
            'failure_reasons': reasons,
        }
