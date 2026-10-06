"""Global path planner: A* on an inflated occupancy grid (pure Python + numpy, unit-tested).

    grid = GridMap.from_occupancy(width, height, resolution, origin_x, origin_y, data)
    planner = Planner(PlannerParams(robot_radius=0.27))
    planner.set_map(grid)
    path = planner.plan((x0, y0), (x1, y1), extra_obstacles=[(x, y), ...])   # list of (x, y) or None
    planner.last_error  -> why it failed ("goal is inside an obstacle", ...)

Plans on a coarser grid (default 0.10 m) so a 20 x 20 m floor stays fast on a Pi 4.
Unknown cells are treated as blocked unless allow_unknown=True.
"""
import heapq
import math
from dataclasses import dataclass

import numpy as np

SQRT2 = math.sqrt(2.0)


@dataclass
class PlannerParams:
    plan_resolution: float = 0.10     # m per planning cell
    robot_radius: float = 0.27        # m: hard keep-out (half of the 440 mm chassis + margin)
    soft_radius: float = 0.60         # m: prefer staying this far from walls
    soft_weight: float = 6.0          # how strongly to prefer the middle of corridors
    occupied_thresh: int = 65
    allow_unknown: bool = False
    snap_radius: float = 0.6          # m: move start/goal out of keep-out zones by up to this
    smooth: bool = True
    waypoint_spacing: float = 0.10    # m between output points


@dataclass
class GridMap:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    occ: np.ndarray        # bool [h, w]  True = occupied
    unknown: np.ndarray    # bool [h, w]

    @classmethod
    def from_occupancy(cls, width, height, resolution, origin_x, origin_y, data, occupied_thresh=65):
        arr = np.asarray(data, dtype=np.int16).reshape(height, width)
        return cls(width, height, resolution, origin_x, origin_y,
                   arr >= occupied_thresh, arr < 0)


class Planner:
    def __init__(self, params: PlannerParams = None):
        self.p = params or PlannerParams()
        self.last_error = ''
        self.last_note = ''         # e.g. goal moved out of a wall
        self._base = None          # (cost, lethal) on the planning grid from the static map
        self._meta = None          # (w, h, res, ox, oy)

    # ------------------------------------------------------------------ map
    def set_map(self, grid: GridMap):
        p = self.p
        f = max(1, int(round(p.plan_resolution / grid.resolution)))
        h, w = grid.height // f, grid.width // f
        if h == 0 or w == 0:
            self._base = None
            return
        occ = grid.occ[:h * f, :w * f].reshape(h, f, w, f).any(axis=(1, 3))
        unk = grid.unknown[:h * f, :w * f].reshape(h, f, w, f).all(axis=(1, 3))
        blocked = occ | (unk if not p.allow_unknown else np.zeros_like(unk))
        self._meta = (w, h, grid.resolution * f, grid.origin_x, grid.origin_y)
        self._occ_static = blocked
        self._base = self._costs(blocked)

    def has_map(self):
        return self._base is not None

    def _costs(self, blocked):
        """Distance-to-obstacle (in cells, approx Euclidean via chamfer) -> cost grid."""
        w, h, res, _, _ = self._meta
        p = self.p
        big = 1e9
        dist = np.where(blocked, 0.0, big)
        # two-pass chamfer distance transform (3-4 style, units = cells)
        dist = _chamfer(dist)
        dist_m = dist * res
        lethal = dist_m < p.robot_radius
        frac = np.clip((p.soft_radius - dist_m) / max(1e-6, p.soft_radius - p.robot_radius), 0.0, 1.0)
        cost = 1.0 + p.soft_weight * frac ** 2
        cost[lethal] = np.inf
        return cost, lethal, dist_m

    # ------------------------------------------------------------------ helpers
    def world_to_cell(self, x, y):
        w, h, res, ox, oy = self._meta
        return int(math.floor((x - ox) / res)), int(math.floor((y - oy) / res))

    def cell_to_world(self, cx, cy):
        w, h, res, ox, oy = self._meta
        return ox + (cx + 0.5) * res, oy + (cy + 0.5) * res

    def _inside(self, cx, cy):
        w, h = self._meta[0], self._meta[1]
        return 0 <= cx < w and 0 <= cy < h

    def is_free(self, x, y):
        if not self.has_map():
            return False
        cx, cy = self.world_to_cell(x, y)
        return self._inside(cx, cy) and not self._base[1][cy, cx]

    def clearance(self, x, y):
        """Distance (m) from (x, y) to the nearest mapped obstacle; 0 outside the map."""
        if not self.has_map():
            return 0.0
        cx, cy = self.world_to_cell(x, y)
        if not self._inside(cx, cy):
            return 0.0
        return float(self._base[2][cy, cx])

    def _stamp(self, cost, pts):
        """Copy of the cost grid with a robot-radius keep-out disk around each point (fast)."""
        cost = cost.copy()
        w, h, res = self._meta[0], self._meta[1], self._meta[2]
        r = int(math.ceil(self.p.robot_radius / res))
        yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
        disk = (xx * xx + yy * yy) <= r * r
        for (x, y) in pts:
            cx, cy = self.world_to_cell(x, y)
            x0, x1, y0, y1 = cx - r, cx + r + 1, cy - r, cy + r + 1
            if x1 <= 0 or y1 <= 0 or x0 >= w or y0 >= h:
                continue
            dx0, dy0 = max(0, -x0), max(0, -y0)
            dx1, dy1 = disk.shape[1] - max(0, x1 - w), disk.shape[0] - max(0, y1 - h)
            sub = cost[max(0, y0):min(h, y1), max(0, x0):min(w, x1)]
            sub[disk[dy0:dy1, dx0:dx1]] = np.inf
        return cost

    def _snap(self, cost, cx, cy):
        """Nearest non-lethal cell within snap_radius (BFS rings)."""
        if self._inside(cx, cy) and np.isfinite(cost[cy, cx]):
            return cx, cy
        r_cells = int(math.ceil(self.p.snap_radius / self._meta[2]))
        best, best_d = None, None
        for dy in range(-r_cells, r_cells + 1):
            for dx in range(-r_cells, r_cells + 1):
                nx, ny = cx + dx, cy + dy
                if self._inside(nx, ny) and np.isfinite(cost[ny, nx]):
                    d = dx * dx + dy * dy
                    if d <= r_cells * r_cells and (best_d is None or d < best_d):
                        best, best_d = (nx, ny), d
        return best

    # ------------------------------------------------------------------ planning
    def plan(self, start, goal, extra_obstacles=()):
        """start/goal (x, y) in map frame -> list of (x, y) waypoints, or None (see last_error)."""
        self.last_error = ''
        self.last_note = ''
        if not self.has_map():
            self.last_error = 'no map yet'
            return None
        cost, lethal, _ = self._base
        if extra_obstacles:
            cost = self._stamp(cost, extra_obstacles)

        s = self._snap(cost, *self.world_to_cell(*start))
        if s is None:
            self.last_error = 'robot is too close to an obstacle to plan (move it away a little)'
            return None
        g = self._snap(cost, *self.world_to_cell(*goal))
        if g is None:
            gx, gy = self.world_to_cell(*goal)
            self.last_error = ('goal is outside the map' if not self._inside(gx, gy)
                               else 'goal is inside or too close to an obstacle')
            return None

        cells = self._astar(cost, s, g)
        if cells is None:
            self.last_error = 'no free path to the goal (blocked or unexplored)'
            return None
        if self.p.smooth:
            cells = self._shortcut(cost, cells)
        pts = [self.cell_to_world(cx, cy) for cx, cy in cells]
        pts[0] = (float(start[0]), float(start[1]))
        if g == self.world_to_cell(*goal):
            pts[-1] = (float(goal[0]), float(goal[1]))
        else:
            moved = math.hypot(pts[-1][0] - goal[0], pts[-1][1] - goal[1])
            self.last_note = f'goal moved {moved:.2f} m away from the obstacle it was in'
        return densify(pts, self.p.waypoint_spacing)

    def _astar(self, cost, s, g):
        w, h = self._meta[0], self._meta[1]
        gx, gy = g
        start_i = s[1] * w + s[0]
        goal_i = gy * w + gx
        flat = cost.ravel()
        g_score = {start_i: 0.0}
        came = {}
        heap = [(0.0, start_i)]
        closed = set()
        nbrs = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2))
        while heap:
            _, cur = heapq.heappop(heap)
            if cur == goal_i:
                out = [cur]
                while cur in came:
                    cur = came[cur]
                    out.append(cur)
                out.reverse()
                return [(i % w, i // w) for i in out]
            if cur in closed:
                continue
            closed.add(cur)
            cx, cy = cur % w, cur // w
            gc = g_score[cur]
            for dx, dy, step in nbrs:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < w and 0 <= ny < h):
                    continue
                ni = ny * w + nx
                c = flat[ni]
                if not math.isfinite(c) or ni in closed:
                    continue
                if dx and dy and (not math.isfinite(flat[cy * w + nx]) or not math.isfinite(flat[ny * w + cx])):
                    continue                      # no corner cutting
                ng = gc + step * c
                if ng < g_score.get(ni, math.inf):
                    g_score[ni] = ng
                    came[ni] = cur
                    hx, hy = abs(nx - gx), abs(ny - gy)
                    hval = (hx + hy) + (SQRT2 - 2) * min(hx, hy)   # octile distance
                    heapq.heappush(heap, (ng + hval, ni))
        return None

    def _shortcut(self, cost, cells):
        """Line-of-sight smoothing that refuses to cut through high-cost (near-wall) cells."""
        if len(cells) < 3:
            return cells
        out = [cells[0]]
        i = 0
        while i < len(cells) - 1:
            j = len(cells) - 1
            while j > i + 1 and not self._line_ok(cost, cells[i], cells[j]):
                j -= 1
            out.append(cells[j])
            i = j
        return out

    def _line_ok(self, cost, a, b, max_cost=None):
        max_cost = max_cost or (1.0 + 0.5 * self.p.soft_weight)
        (x0, y0), (x1, y1) = a, b
        n = max(abs(x1 - x0), abs(y1 - y0))
        for k in range(n + 1):
            t = k / n if n else 0
            x = int(round(x0 + (x1 - x0) * t))
            y = int(round(y0 + (y1 - y0) * t))
            c = cost[y, x]
            if not math.isfinite(c) or c > max_cost:
                return False
        return True


def _chamfer(d):
    """Approximate Euclidean distance transform (cells) with a 3-4 chamfer, vectorised by rows."""
    h, w = d.shape
    d = d.copy()
    a, b = 1.0, SQRT2
    for y in range(h):                       # forward pass
        row = d[y]
        if y > 0:
            up = d[y - 1]
            row = np.minimum(row, up + a)
            row[1:] = np.minimum(row[1:], up[:-1] + b)
            row[:-1] = np.minimum(row[:-1], up[1:] + b)
        for x in range(1, w):                 # left -> right (sequential dependency)
            if row[x - 1] + a < row[x]:
                row[x] = row[x - 1] + a
        d[y] = row
    for y in range(h - 1, -1, -1):           # backward pass
        row = d[y]
        if y < h - 1:
            dn = d[y + 1]
            row = np.minimum(row, dn + a)
            row[1:] = np.minimum(row[1:], dn[:-1] + b)
            row[:-1] = np.minimum(row[:-1], dn[1:] + b)
        for x in range(w - 2, -1, -1):
            if row[x + 1] + a < row[x]:
                row[x] = row[x + 1] + a
        d[y] = row
    return d


def densify(pts, spacing):
    if len(pts) < 2:
        return list(pts)
    out = [pts[0]]
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(math.ceil(seg / spacing)))
        for k in range(1, n + 1):
            t = k / n
            out.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
    return out


def path_length(pts):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
