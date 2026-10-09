"""Frontier exploration: let the robot map a floor by itself (pure Python + numpy, unit-tested).

A *frontier* is the border between mapped free space and unknown space. The explorer repeatedly
picks the most useful reachable frontier, sends the robot there slowly (good scans = good map),
and stops when no frontier big enough is left.

    find_frontiers(grid, min_cells)       -> [Frontier]
    choose_goal(frontiers, planner, pose, blacklist, params) -> (x, y, frontier) | None
"""
import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np


@dataclass
class ExploreParams:
    min_frontier_cells: int = 15        # ignore frontiers shorter than ~0.75 m at 5 cm/cell
    standoff: float = 0.05              # extra clearance (m) beyond robot_radius for goal points
    search_radius: float = 1.2          # m: look this far from a frontier for a safe goal point
    blacklist_radius: float = 0.6       # m: failed or finished goals block this area
    gain_per_cell: float = 0.03         # m of path we'd travel extra per frontier cell gained
    max_candidates: int = 6             # path-plan only the best few by straight-line score
    max_goal_time: float = 120.0        # s before a goal is abandoned
    speed: float = 0.15                 # m/s while exploring (slow = sharp map)
    turn_speed: float = 0.45            # rad/s while exploring
    scan_spin: bool = True              # turn once on the spot at every goal (walls from all angles)
    scan_speed: float = 0.4             # rad/s for that turn (~16 s per turn)


@dataclass
class Frontier:
    cx: float                 # centroid (m, map frame)
    cy: float
    size: int                 # number of frontier cells
    pts: list = field(default_factory=list)   # a few member points (m) for goal search / drawing


def find_frontiers(grid, min_cells=10, max_pts=40):
    """Frontier clusters in a GridMap (8-connected free cells touching unknown cells)."""
    occ, unk = grid.occ, grid.unknown
    free = ~occ & ~unk
    nb = np.zeros_like(unk)
    nb[1:, :] |= unk[:-1, :]
    nb[:-1, :] |= unk[1:, :]
    nb[:, 1:] |= unk[:, :-1]
    nb[:, :-1] |= unk[:, 1:]
    front = free & nb
    # don't treat the outermost map border as a frontier (it's just the edge of the array)
    front[0, :] = front[-1, :] = False
    front[:, 0] = front[:, -1] = False
    ys, xs = np.nonzero(front)
    if len(xs) == 0:
        return []
    seen = set()
    cells = set(zip(xs.tolist(), ys.tolist()))
    out = []
    res, ox, oy = grid.resolution, grid.origin_x, grid.origin_y
    for start in zip(xs.tolist(), ys.tolist()):
        if start in seen:
            continue
        q = deque([start])
        seen.add(start)
        members = []
        while q:
            x, y = q.popleft()
            members.append((x, y))
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    n = (x + dx, y + dy)
                    if n in cells and n not in seen:
                        seen.add(n)
                        q.append(n)
        if len(members) < min_cells:
            continue
        mx = sum(m[0] for m in members) / len(members)
        my = sum(m[1] for m in members) / len(members)
        step = max(1, len(members) // max_pts)
        pts = [(ox + (x + 0.5) * res, oy + (y + 0.5) * res) for x, y in members[::step]]
        out.append(Frontier(ox + (mx + 0.5) * res, oy + (my + 0.5) * res, len(members), pts))
    out.sort(key=lambda f: -f.size)
    return out


def _safe_point_near(planner, pts, robot_radius, params):
    """Closest point (to the frontier) where the robot fits, within search_radius."""
    if not planner.has_map():
        return None
    w, h, res, ox, oy = planner._meta
    dist_m = planner._base[2]
    need = robot_radius + params.standoff
    r = int(math.ceil(params.search_radius / res))
    best, best_d = None, None
    for (px, py) in pts:
        cx, cy = planner.world_to_cell(px, py)
        x0, x1 = max(0, cx - r), min(w, cx + r + 1)
        y0, y1 = max(0, cy - r), min(h, cy + r + 1)
        if x0 >= x1 or y0 >= y1:
            continue
        win = dist_m[y0:y1, x0:x1]
        ys, xs = np.nonzero(win >= need)
        if len(xs) == 0:
            continue
        d2 = (xs + x0 - cx) ** 2 + (ys + y0 - cy) ** 2
        i = int(np.argmin(d2))
        if best_d is None or d2[i] < best_d:
            best_d = d2[i]
            best = planner.cell_to_world(int(xs[i] + x0), int(ys[i] + y0))
    return best


def choose_goal(frontiers, planner, pose, blacklist, params=None, robot_radius=0.27, extra_obstacles=()):
    """Best reachable frontier goal as (x, y, frontier, path) or None.

    Score = path length - gain_per_cell * frontier size (lower is better). Straight-line distance
    ranks the candidates first so only a few need a full A* plan.
    """
    params = params or ExploreParams()
    cands = []
    for f in frontiers:
        if any(math.hypot(f.cx - bx, f.cy - by) < params.blacklist_radius for bx, by in blacklist):
            continue
        g = _safe_point_near(planner, f.pts, robot_radius, params)
        if g is None:
            continue
        if any(math.hypot(g[0] - bx, g[1] - by) < params.blacklist_radius for bx, by in blacklist):
            continue
        straight = math.hypot(g[0] - pose[0], g[1] - pose[1])
        cands.append((straight - params.gain_per_cell * f.size, g, f))
    cands.sort(key=lambda c: c[0])
    best = None
    for _, g, f in cands[:params.max_candidates]:
        path = planner.plan(pose[:2], g, extra_obstacles)
        if path is None:
            continue
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))
        score = length - params.gain_per_cell * f.size
        if best is None or score < best[0]:
            best = (score, g, f, path)
    if best is None:
        return None
    return best[1][0], best[1][1], best[2], best[3]


def mapped_area(grid):
    """Known free area in m²."""
    return float(np.count_nonzero(~grid.occ & ~grid.unknown)) * grid.resolution ** 2


class MapAutoSaver:
    """Saves the map once when an exploration finishes (watch the navigator's explore status).

    update(explore_status) is cheap; call it whenever a new status arrives. save_fn(name) runs in a
    background thread (map saving can take seconds while Cartographer writes its state).
    """

    def __init__(self, save_fn, prefix='explore'):
        import threading
        self._threading = threading
        self.save_fn, self.prefix = save_fn, prefix
        self._was_active = False
        self.info = {'state': 'idle'}

    def update(self, explore):
        if not explore:
            return
        active = bool(explore.get('active'))
        if self._was_active and not active and explore.get('state') == 'done':
            import time
            name = time.strftime(f'{self.prefix}_%Y%m%d_%H%M%S')
            self.info = {'state': 'saving', 'name': name}
            self._threading.Thread(target=self._save, args=(name,), daemon=True).start()
        self._was_active = active

    def _save(self, name):
        try:
            res = self.save_fn(name)
            self.info = {'state': 'saved', 'name': name, 'result': str(res)}
        except Exception as e:  # noqa: BLE001 - report any failure to the app
            self.info = {'state': 'failed', 'name': name, 'result': str(e)}
