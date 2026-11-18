"""Relative places: "stand in the corridor near Ravi sir's cabin", "HOD office ke bahar" (pure Python, tested).

A saved place is usually inside a room (or at its door). For a step with side 'outside' the robot should stop in
the open space next to it instead: the nearest free spot that is

- outside the place's room (when the place has a room drawn on the map; corridors/halls/lobbies do not count as rooms),
  or at least `min_gap` from the place when there is no room,
- open enough: `clearance >= need` (robot radius + a little), so it is a corridor, not a gap behind a cupboard,
- reachable from the place over free floor within `max_r` metres of walking (out through the door).

It then faces the place (the cabin door). Falls back to None: the caller drives to the place itself and says so.
"""
import math

OPEN_AREA = ('corridor', 'hallway', 'hall', 'gallery', 'passage', 'lobby', 'foyer', 'floor', 'wing', 'galiyara')


def is_open_area(room):
    n = (room.get('name') or '').lower()
    return any(w in n.split() or n.startswith(w) for w in OPEN_AREA)


def outside_spot(target, places, is_free, clearance, need=0.40, max_r=4.0, step=0.10, min_gap=0.8):
    """target: {'name', 'x', 'y'}; places: PlaceStore (room_at, get_room); is_free(x, y) / clearance(x, y) from the
    planner. Walks out from the place over free floor (so it leaves through the door, never "through" a wall into
    the next lab) and returns the first open spot outside the room: {'x', 'y', 'yaw', 'room', 'walk_m'} or None."""
    import heapq
    tx, ty = float(target['x']), float(target['y'])
    room = places.get_room(target['name']) or places.room_at(tx, ty)
    if room is not None and is_open_area(room):
        room = None                                     # the place is already in the corridor
    start = None
    for r in (0, 1, 2, 3):                              # the place may sit on a chair/desk cell: nearest free cell
        cand = [(i, j) for i in range(-r, r + 1) for j in range(-r, r + 1) if max(abs(i), abs(j)) == r
                and is_free(tx + i * step, ty + j * step)]
        if cand:
            start = min(cand, key=lambda c: c[0] ** 2 + c[1] ** 2)
            break
    if start is None:
        return None
    lim = max_r / step
    dist = {start: 0.0}
    heap = [(0.0, start)]
    moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
    while heap:
        d, (i, j) = heapq.heappop(heap)
        if d > dist.get((i, j), 1e9):
            continue
        x, y = tx + i * step, ty + j * step
        straight = math.hypot(x - tx, y - ty)
        ok = clearance(x, y) >= need
        if ok and room is not None:
            r = places.room_at(x, y)
            ok = r is None or is_open_area(r)
        elif ok:
            ok = straight >= min_gap
        if ok:
            return {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(math.atan2(ty - y, tx - x), 3),
                    'room': room['name'] if room else None, 'walk_m': round(d * step, 2)}
        for di, dj, c in moves:
            nb = (i + di, j + dj)
            nd = d + c
            if nd > lim or nd >= dist.get(nb, 1e9):
                continue
            nx, ny = tx + nb[0] * step, ty + nb[1] * step
            if not is_free(nx, ny) or not is_free((x + nx) / 2, (y + ny) / 2):
                continue
            dist[nb] = nd
            heapq.heappush(heap, (nd, nb))
    return None
