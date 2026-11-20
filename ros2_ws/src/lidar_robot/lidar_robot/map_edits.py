"""Hand edits on a saved map, for route planning only (pure Python + numpy, tested).

The robot finds itself with Cartographer's .pbstream, which is built only from what the LiDAR saw and is never
touched here. Route planning uses the occupancy grid made from it, and unexplored (grey) cells are blocked. On
20 Nov the department map showed the cabins only from their doors, so no trip could go in. This file keeps hand
edits for one saved map, in their own file next to the maps (~/maps/map_edits/<map>.json), and lays them over
the grid the planner gets:

    free   a polygon (3+ corners): unexplored cells inside become drivable. Scanned walls stay walls.
    wall   a line (2+ points, 10 cm thick): the planner never routes through it (glass doors, stairs, cables).

    edits = MapEdits('~/maps/map_edits/cse.json')
    edits.add('free', [[x, y], ...]); edits.remove(id); edits.clear()
    grid2 = edits.apply(grid)            # a new GridMap; the original is unchanged

The LiDAR and ultrasonic stops keep working inside painted areas: an edit can make the planner try a route,
never make the robot drive into something it sees.
"""
import json
import math
import os
import time

import numpy as np

from .planner import GridMap

KINDS = ('free', 'wall')
MAX_EDITS = 100
MAX_POINTS = 40
MAX_FREE_AREA_M2 = 150.0      # a cabin or a lab, not "the whole floor" by a slip of the finger
WALL_HALF_M = 0.05            # painted walls are 10 cm thick


def edits_path(map_file, map_dir):
    """The edits file of a saved map, or '' when there is no saved map (mapping: the map still changes)."""
    if not map_file:
        return ''
    stem = os.path.splitext(os.path.basename(os.path.expanduser(map_file)))[0]
    return os.path.join(os.path.expanduser(map_dir), 'map_edits', stem + '.json') if stem else ''


def polygon_area(pts):
    a = 0.0
    for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]):
        a += x0 * y1 - x1 * y0
    return abs(a) / 2.0


def _clean_points(pts, need):
    if not isinstance(pts, (list, tuple)) or not (need <= len(pts) <= MAX_POINTS):
        raise ValueError(f'needs {need} to {MAX_POINTS} points')
    out = []
    for p in pts:
        try:
            x, y = float(p[0]), float(p[1])
        except (TypeError, ValueError, IndexError):
            raise ValueError('points must be [x, y] numbers')
        if not (math.isfinite(x) and math.isfinite(y)) or abs(x) > 1000 or abs(y) > 1000:
            raise ValueError('point outside the map')
        out.append([round(x, 3), round(y, 3)])
    return out


class MapEdits:
    def __init__(self, path, clock=time.time):
        self.path = os.path.expanduser(path) if path else ''
        self.clock = clock
        self.edits = []
        self.version = 0
        self.error = None
        self._load()

    @property
    def enabled(self):
        return bool(self.path)

    def _load(self):
        if not self.path:
            return
        try:
            with open(self.path) as f:
                data = json.load(f)
            items = data.get('edits', []) if isinstance(data, dict) else []
            good = []
            for e in items[:MAX_EDITS]:
                try:
                    kind = e['kind']
                    if kind not in KINDS:
                        continue
                    good.append({'id': int(e['id']), 'kind': kind,
                                 'pts': _clean_points(e['pts'], 3 if kind == 'free' else 2),
                                 'at': e.get('at')})
                except (KeyError, TypeError, ValueError):
                    continue
            self.edits = good
        except FileNotFoundError:
            self.edits = []
        except (OSError, ValueError) as e:
            self.error = f'could not read {self.path}: {e}'
            self.edits = []

    def _save(self):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        tmp = self.path + '.part'
        with open(tmp, 'w') as f:
            json.dump({'map': os.path.basename(self.path)[:-5], 'edits': self.edits}, f, indent=1)
        os.replace(tmp, self.path)              # never a half-written file
        self.version += 1

    def add(self, kind, pts):
        if not self.enabled:
            raise ValueError('painting works on a saved map: Map → Use a saved map first')
        if kind not in KINDS:
            raise ValueError(f'kind must be one of {KINDS}')
        if len(self.edits) >= MAX_EDITS:
            raise ValueError(f'{MAX_EDITS} edits at most: delete some first')
        pts = _clean_points(pts, 3 if kind == 'free' else 2)
        if kind == 'free':
            area = polygon_area(pts)
            if area < 0.05:
                raise ValueError('that area is too small: tap its corners further apart')
            if area > MAX_FREE_AREA_M2:
                raise ValueError(f'that area is {area:.0f} m²; paint at most {MAX_FREE_AREA_M2:.0f} m² at a time')
        rec = {'id': max([e['id'] for e in self.edits], default=0) + 1, 'kind': kind, 'pts': pts,
               'at': int(self.clock())}
        self.edits.append(rec)
        self._save()
        return rec

    def remove(self, edit_id):
        n = len(self.edits)
        self.edits = [e for e in self.edits if e['id'] != int(edit_id)]
        if len(self.edits) == n:
            raise ValueError('no such edit')
        self._save()

    def clear(self):
        self.edits = []
        if self.enabled:
            self._save()

    def summary(self):
        return {'enabled': self.enabled, 'map': os.path.basename(self.path)[:-5] if self.path else '',
                'edits': self.edits, 'version': self.version, 'error': self.error}

    # ------------------------------------------------------------------ grid
    def apply(self, grid: GridMap) -> GridMap:
        """A copy of the grid with the edits laid over it (the original stays as Cartographer made it)."""
        if not self.edits:
            return grid
        occ, unk = grid.occ.copy(), grid.unknown.copy()
        for e in self.edits:
            if e['kind'] == 'free':
                m = _polygon_mask(grid, e['pts'])
                unk[m] = False                     # unexplored -> free; a scanned wall stays a wall
            else:
                m = _polyline_mask(grid, e['pts'], WALL_HALF_M)
                occ[m] = True
                unk[m] = False
        return GridMap(grid.width, grid.height, grid.resolution, grid.origin_x, grid.origin_y, occ, unk)


def _cells(grid, xs, ys):
    """Cell index ranges covering the world box [xs] x [ys], clipped to the grid."""
    r = grid.resolution
    c0 = max(0, int(math.floor((min(xs) - grid.origin_x) / r)))
    c1 = min(grid.width, int(math.ceil((max(xs) - grid.origin_x) / r)) + 1)
    r0 = max(0, int(math.floor((min(ys) - grid.origin_y) / r)))
    r1 = min(grid.height, int(math.ceil((max(ys) - grid.origin_y) / r)) + 1)
    return r0, r1, c0, c1


def _polygon_mask(grid, pts):
    mask = np.zeros((grid.height, grid.width), dtype=bool)
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    r0, r1, c0, c1 = _cells(grid, xs, ys)
    if r0 >= r1 or c0 >= c1:
        return mask
    cx = grid.origin_x + (np.arange(c0, c1) + 0.5) * grid.resolution
    cy = grid.origin_y + (np.arange(r0, r1) + 0.5) * grid.resolution
    X, Y = np.meshgrid(cx, cy)
    inside = np.zeros(X.shape, dtype=bool)
    n = len(pts)
    for i in range(n):                         # even-odd ray casting, all cells at once
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % n]
        if y0 == y1:
            continue
        crosses = (Y >= min(y0, y1)) & (Y < max(y0, y1))
        xint = x0 + (Y - y0) * (x1 - x0) / (y1 - y0)
        inside ^= crosses & (X < xint)
    mask[r0:r1, c0:c1] = inside
    return mask


def _polyline_mask(grid, pts, half):
    mask = np.zeros((grid.height, grid.width), dtype=bool)
    reach = half + grid.resolution * 0.71        # a cell counts when the line passes through it
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        r0, r1, c0, c1 = _cells(grid, [x0 - reach, x1 + reach, x0 + reach, x1 - reach],
                                [y0 - reach, y1 + reach, y0 + reach, y1 - reach])
        if r0 >= r1 or c0 >= c1:
            continue
        cx = grid.origin_x + (np.arange(c0, c1) + 0.5) * grid.resolution
        cy = grid.origin_y + (np.arange(r0, r1) + 0.5) * grid.resolution
        X, Y = np.meshgrid(cx, cy)
        dx, dy = x1 - x0, y1 - y0
        L2 = dx * dx + dy * dy
        t = np.clip(((X - x0) * dx + (Y - y0) * dy) / L2, 0.0, 1.0) if L2 > 0 else np.zeros_like(X)
        d = np.hypot(X - (x0 + t * dx), Y - (y0 + t * dy))
        mask[r0:r1, c0:c1] |= d <= reach
    return mask
