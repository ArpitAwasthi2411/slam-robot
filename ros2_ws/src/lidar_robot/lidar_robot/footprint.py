"""The robot's real shape: a rectangle, with the drive axle near the FRONT (pure Python, unit-tested).

Why this exists: the planner used to treat the robot as a 0.27 m circle around the axle. Our body
runs 0.37 m BEHIND the axle, so when the robot turns, its back swings out and clips walls the
planner thought were clear.

    fp = Footprint(front=0.07, rear=0.37, half_width=0.21)   # metres from the axle centre (base_link)
    fp.signed_distance(px, py)     # point in base_link: < 0 inside the body, > 0 outside (metres)
    fp.circumradius                # farthest body corner from the axle (radius of a turn on the spot)

    g = FootprintGuard(fp, scan_points_base)     # live LiDAR / ultrasonic points in base_link
    g.min_gap                      # closest obstacle to the body outline, any side (m)
    g.rotation_clear(angle)        # can we turn on the spot by `angle` rad (sign = direction)?
    g.motion_clear(v, w)           # does driving (v, w) for a moment bring the body into contact?

    check_path(planner, path, fp)  # map check of a planned route with the body at every point
"""
import math
from dataclasses import dataclass


@dataclass
class Footprint:
    front: float = 0.07          # m from the axle to the front bumper
    rear: float = 0.37           # m from the axle to the back edge
    half_width: float = 0.21     # m from the centre line to the side
    margin: float = 0.04         # m extra keep-out used by the checks

    @property
    def circumradius(self):
        return math.hypot(max(self.front, self.rear), self.half_width)

    @property
    def inscribed(self):
        return min(self.half_width, self.front, self.rear) if self.front > 0 else self.half_width

    def signed_distance(self, px, py):
        cx = (self.front - self.rear) / 2.0
        hx = (self.front + self.rear) / 2.0
        qx = abs(px - cx) - hx
        qy = abs(py) - self.half_width
        outside = math.hypot(max(qx, 0.0), max(qy, 0.0))
        return outside + min(max(qx, qy), 0.0)

    def outline(self, step=0.06):
        """Points around the body outline in base_link (for map checks and drawing)."""
        pts = []
        corners = [(self.front, self.half_width), (-self.rear, self.half_width),
                   (-self.rear, -self.half_width), (self.front, -self.half_width)]
        for (x0, y0), (x1, y1) in zip(corners, corners[1:] + corners[:1]):
            n = max(1, int(math.ceil(math.hypot(x1 - x0, y1 - y0) / step)))
            for k in range(n):
                t = k / n
                pts.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
        return pts

    def as_list(self):
        return [round(self.front, 3), round(self.rear, 3), round(self.half_width, 3)]


class FootprintGuard:
    """Live collision checks against the latest scan, in the robot's own frame."""
    SELF_BAND = 0.03       # points this deep inside the outline are the robot's own parts: ignore

    def __init__(self, fp: Footprint, points_base):
        self.fp = fp
        reach = fp.circumradius + fp.margin + 0.15
        self.pts = []
        best = math.inf
        for (x, y) in points_base or ():
            if x * x + y * y > reach * reach:
                continue
            d = fp.signed_distance(x, y)
            if d < -self.SELF_BAND:
                continue
            self.pts.append((x, y, d))
            best = min(best, d)
        self.min_gap = best

    def rotation_clear(self, angle, margin=None):
        """True if turning on the spot by `angle` (rad, + = left) keeps the body `margin` off every point."""
        m = self.fp.margin if margin is None else margin
        if not self.pts or abs(angle) < 1e-3:
            return True
        n = max(1, int(math.ceil(abs(angle) / 0.08)))
        for (x, y, d0) in self.pts:
            for k in range(1, n + 1):
                phi = angle * k / n
                c, s = math.cos(phi), math.sin(phi)
                bx, by = c * x + s * y, -s * x + c * y       # the point seen from the turned body
                d = self.fp.signed_distance(bx, by)
                if d < m and d < d0 - 0.005:                    # closer than allowed AND getting closer
                    return False
        return True

    def motion_clear(self, v, w, horizon=0.6, contact=0.02):
        """True if driving (v, w) for `horizon` s doesn't push the body to within `contact` of a point."""
        if not self.pts or (abs(v) < 1e-4 and abs(w) < 1e-4):
            return True
        steps = 6
        dt = horizon / steps
        x = y = th = 0.0
        for _ in range(steps):
            x += v * math.cos(th) * dt
            y += v * math.sin(th) * dt
            th += w * dt
            c, s = math.cos(th), math.sin(th)
            for (px, py, d0) in self.pts:
                dx, dy = px - x, py - y
                bx, by = c * dx + s * dy, -s * dx + c * dy
                d = self.fp.signed_distance(bx, by)
                if d < contact and d < d0 - 0.005:
                    return False
        return True


    @staticmethod
    def _crowding(gaps, near=0.25):
        """How hemmed-in the body is: sum of (near - gap) over close points (0 = open floor)."""
        return sum(near - g for g in gaps if g < near)

    def room_after(self, v, w, horizon=1.5, contact=0.02):
        """(min gap, crowding) after driving (v, w) for `horizon` s; None if it would touch on the way."""
        if not self.motion_clear(v, w, horizon, contact):
            return None
        steps = 6
        dt = horizon / steps
        x = y = th = 0.0
        for _ in range(steps):
            x += v * math.cos(th) * dt
            y += v * math.sin(th) * dt
            th += w * dt
        c, s = math.cos(th), math.sin(th)
        gaps = []
        for (px, py, _) in self.pts:
            dx, dy = px - x, py - y
            gaps.append(self.fp.signed_distance(c * dx + s * dy, -s * dx + c * dy))
        return (min(gaps) if gaps else math.inf), self._crowding(gaps)

    def escape(self, prefer_w=0.0, speed=0.06, horizon=1.5):
        """Best small move (v, w) that gives the body more room, or None.

        Used when a turn on the spot is blocked: with the axle at the front, pulling ahead out of a
        corner or backing away from a wall (like a forklift) gives the back room to swing round.
        """
        now = self._crowding([d for (_, _, d) in self.pts])
        best, best_score = None, now - 0.005
        for v in (speed, -speed):
            for w in (0.0, 0.3, -0.3):
                r = self.room_after(v, w, horizon)
                if r is None or r[0] < min(self.min_gap, 0.04) - 0.002:
                    continue
                score = r[1] - (0.002 if v > 0 else 0.0) - (0.001 if w * prefer_w > 0 else 0.0)
                if score < best_score:
                    best, best_score = (v, w), score
        return best


def _heading(path, i):
    j = min(len(path) - 1, i + 2)
    k = max(0, j - 3)
    (x0, y0), (x1, y1) = path[k], path[j]
    return math.atan2(y1 - y0, x1 - x0)


def check_path(planner, path, fp: Footprint, start_yaw=None, end_yaw=None, every=0.15, turn_above=1.2):
    """Body-on-the-map check of a route. Returns [(x, y), ...] where the body would touch a wall.

    Checks the outline at points along the route (heading = route direction) and the sweep of
    every turn on the spot (start turn and sharp corners), like the follower will drive it.
    """
    if not path or len(path) < 2 or not planner.has_map():
        return []
    outline = fp.outline()
    bad = []

    def body_hits(x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        for (bx, by) in outline:
            if planner.clearance(x + c * bx - s * by, y + s * bx + c * by) < fp.margin:
                return True
        return False

    def sweep_hits(x, y, a0, a1):
        d = math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))
        n = max(1, int(math.ceil(abs(d) / 0.15)))
        return any(body_hits(x, y, a0 + d * k / n) for k in range(n + 1))

    if start_yaw is not None:
        h0 = _heading(path, 0)
        if abs(math.atan2(math.sin(h0 - start_yaw), math.cos(h0 - start_yaw))) > turn_above \
                and sweep_hits(path[0][0], path[0][1], start_yaw, h0):
            bad.append((round(path[0][0], 2), round(path[0][1], 2)))
    acc = every
    prev_h = _heading(path, 0)
    for i in range(1, len(path)):
        acc += math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
        h = _heading(path, i)
        sharp = abs(math.atan2(math.sin(h - prev_h), math.cos(h - prev_h))) > turn_above
        if sharp and sweep_hits(path[i][0], path[i][1], prev_h, h):
            bad.append((round(path[i][0], 2), round(path[i][1], 2)))
        elif acc >= every:
            acc = 0.0
            if i < len(path) - 1 and body_hits(path[i][0], path[i][1], h):
                bad.append((round(path[i][0], 2), round(path[i][1], 2)))
        prev_h = h
    if end_yaw is not None and sweep_hits(path[-1][0], path[-1][1], prev_h, end_yaw):
        bad.append((round(path[-1][0], 2), round(path[-1][1], 2)))
    return bad


# ---------------------------------------------------------------------- tight-spot manoeuvres
def _sd_array(fp, bx, by):
    import numpy as np
    cx = (fp.front - fp.rear) / 2.0
    hx = (fp.front + fp.rear) / 2.0
    qx = np.abs(bx - cx) - hx
    qy = np.abs(by) - fp.half_width
    outside = np.hypot(np.maximum(qx, 0.0), np.maximum(qy, 0.0))
    return outside + np.minimum(np.maximum(qx, qy), 0.0)


def plan_manoeuvre(guard, target, clear=0.02, step=0.10, turn=0.25, max_expand=2500, face_within=0.35):
    """Shortest sequence of small moves (forward / back / turn on the spot) that ends facing `target`.

    target: (x, y) in base_link. Used when the robot is boxed in (e.g. backed into a corner) and
    cannot turn on the spot: like a forklift it pulls ahead or backs out first (a 3-point turn).
    Returns [('move', metres) | ('turn', radians), ...] (relative to the start pose) or None.
    """
    import heapq
    import numpy as np
    pts = [(x, y) for (x, y, _) in guard.pts]
    if not pts:
        return []
    P = np.array(pts)
    fp = guard.fp
    clear = max(0.003, min(clear, guard.min_gap - 0.005))     # parked tight: never closer than now

    def ok(x, y, th):
        c, s = math.cos(th), math.sin(th)
        dx, dy = P[:, 0] - x, P[:, 1] - y
        return bool(np.all(_sd_array(fp, c * dx + s * dy, -s * dx + c * dy) >= clear))

    def facing(x, y, th):
        a = math.atan2(target[1] - y, target[0] - x) - th
        return abs(math.atan2(math.sin(a), math.cos(a))) < face_within

    start = (0.0, 0.0, 0.0)
    if facing(*start):
        return []
    key = lambda x, y, th: (round(x / 0.05), round(y / 0.05), round(th / turn))   # noqa: E731
    heap = [(0.0, 0, start, [])]
    seen = {key(*start)}
    n = 0
    while heap and n < max_expand:
        cost, _, (x, y, th), seq = heapq.heappop(heap)
        n += 1
        moves = (('move', step), ('move', -step), ('turn', turn), ('turn', -turn))
        for kind, amt in moves:
            if kind == 'move':
                subs = [(x + amt * f * math.cos(th), y + amt * f * math.sin(th), th) for f in (0.34, 0.67, 1.0)]
                c = abs(amt) * (1.0 if amt > 0 else 1.6)              # prefer driving forwards
            else:
                subs = [(x, y, th + amt * f) for f in (0.34, 0.67, 1.0)]
                c = abs(amt) * 0.15
            if seq and seq[-1][0] == kind and kind == 'move' and (seq[-1][1] > 0) != (amt > 0):
                c += 0.15                                               # direction change
            nxt = subs[-1]
            k = key(*nxt)
            if k in seen or not all(ok(*p) for p in subs):
                continue
            seen.add(k)
            nseq = seq + [(kind, amt)]
            if facing(*nxt):
                out = []
                for kd, a in nseq:                                      # merge repeated moves
                    if out and out[-1][0] == kd and (out[-1][1] > 0) == (a > 0):
                        out[-1] = (kd, out[-1][1] + a)
                    else:
                        out.append((kd, a))
                return out
            heapq.heappush(heap, (cost + c, n * 4 + len(nseq), nxt, nseq))
    return None
