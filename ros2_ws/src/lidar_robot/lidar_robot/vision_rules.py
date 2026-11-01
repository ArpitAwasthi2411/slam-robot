"""Camera people-avoidance logic (pure Python + numpy, unit-tested; no ROS, no camera).

Used by vision_node.py (camera + YOLO on the Pi) and navigator.py (VisionGate):

  decode_yolo(out, ...)        YOLO11 NCNN output -> person boxes in image pixels (+ NMS)
  PersonTracker                ids across frames, approaching / moving away (box height growth)
  bearing_deg(box, w, hfov)    angle of a person: <0 left, >0 right (camera looks forward)
  lidar_distance(...)          range from the LiDAR scan at that bearing (metres), None if no hit
  decide(people, frame_h)      STOP / SLOW_DOWN / CONTINUE + speed factor + reason
  VisionGate                   navigator side: applies a fresh decision to (v, w)

Same rules as vision/robot_decision.py (Rashi's laptop version), but with real distances from the
LiDAR when the person's bearing hits the scan; box size is only the fallback.
"""
import math
import time

ACTIONS = ('STOP', 'SLOW_DOWN', 'CONTINUE')
CAUTION = {'STOP': 2, 'SLOW_DOWN': 1, 'CONTINUE': 0}

DEFAULTS = {
    'path_half_angle_deg': 15.0,   # |bearing| below this = in front of the robot
    'stop_m': 0.8,                 # person in the path closer than this -> STOP
    'slow_m': 1.8,                 # ... closer than this -> SLOW_DOWN (STOP if approaching)
    'side_slow_m': 0.6,            # person beside the path closer than this -> SLOW_DOWN
    'very_close_ratio': 0.60,      # fallback without LiDAR: box height / frame height
    'near_ratio': 0.35,
}


# ------------------------------------------------------------------ YOLO decoding
def letterbox_params(w, h, size):
    """Scale + padding used to fit a w x h frame into a size x size square (ultralytics style)."""
    s = min(size / w, size / h)
    nw, nh = int(round(w * s)), int(round(h * s))
    return s, (size - nw) // 2, (size - nh) // 2, nw, nh


def _iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def nms(boxes, iou_thr=0.5):
    """boxes: [(x1, y1, x2, y2, conf)] -> kept boxes, best first."""
    keep = []
    for b in sorted(boxes, key=lambda b: -b[4]):
        if all(_iou(b, k) < iou_thr for k in keep):
            keep.append(b)
    return keep


def decode_yolo(out, frame_w, frame_h, size, conf=0.4, cls=0):
    """YOLO11 detect output (84 x N or N x 84: cx, cy, w, h, 80 class scores, in letterbox pixels)
    -> [(x1, y1, x2, y2, conf)] of class `cls` (0 = person) in frame pixels."""
    import numpy as np
    a = np.asarray(out, dtype=np.float32)
    a = a.reshape(a.shape[-2], a.shape[-1]) if a.ndim == 3 else a
    if a.shape[0] != 84 and a.shape[1] == 84:
        a = a.T
    scores = a[4 + cls]
    idx = np.nonzero(scores >= conf)[0]
    s, px, py, _, _ = letterbox_params(frame_w, frame_h, size)
    boxes = []
    for i in idx:
        cx, cy, bw, bh = (float(v) for v in a[:4, i])
        x1 = max(0.0, (cx - bw / 2 - px) / s)
        y1 = max(0.0, (cy - bh / 2 - py) / s)
        x2 = min(float(frame_w), (cx + bw / 2 - px) / s)
        y2 = min(float(frame_h), (cy + bh / 2 - py) / s)
        if x2 > x1 and y2 > y1:
            boxes.append((x1, y1, x2, y2, float(scores[i])))
    return nms(boxes)


# ------------------------------------------------------------------ tracking
class PersonTracker:
    """Gives each person an id across frames and says whether the box is growing (approaching) or
    shrinking (moving away) over the last ~0.5 s.

    Matching (kept identical in web/app/vision.js):
      - every track's box is PREDICTED forward with its own velocity (centre and size, smoothed), so a person
        walking across the picture still matches next frame;
      - a pair matches on IoU with the prediction >= iou_thr, or, failing that, on a centre within 0.5 x the
        track's height and a similar height (people crossing, a box that jumped);
      - all pairs are taken best first (not box by box), so two people crossing don't swap ids;
      - two stages: confident boxes (>= high_conf) first; weaker boxes may only CONTINUE a track that lost its
        match (someone half hidden), never start one;
      - a track not seen is kept, still predicted, for forget_s (1.0 s of prediction at most), so a person who
        is hidden for a moment comes back with the same id.
    """

    def __init__(self, iou_thr=0.3, forget_s=2.0, window_s=0.5, change=0.08, high_conf=0.45):
        self.iou_thr, self.forget_s, self.window_s, self.change = iou_thr, forget_s, window_s, change
        self.high_conf = high_conf
        self.tracks = {}          # id -> {'box', 'seen', 'vel': [vcx, vcy, vw, vh] per s, 'hist': [(t, height)]}
        self.next_id = 1

    def _predict(self, t, now):
        dt = min(max(now - t['seen'], 0.0), 1.0)
        b, v = t['box'], t['vel']
        cx = (b[0] + b[2]) / 2 + v[0] * dt
        cy = (b[1] + b[3]) / 2 + v[1] * dt
        w = max(1.0, (b[2] - b[0]) + v[2] * dt)
        h = max(1.0, (b[3] - b[1]) + v[3] * dt)
        return (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)

    def _score(self, b, p):
        """Match quality of box b against predicted box p, or None."""
        v = _iou(b, p)
        if v >= self.iou_thr:
            return v
        ph, bh = p[3] - p[1], b[3] - b[1]
        d = math.hypot((b[0] + b[2]) / 2 - (p[0] + p[2]) / 2, (b[1] + b[3]) / 2 - (p[1] + p[3]) / 2)
        if ph > 0 and bh > 0 and d <= 0.5 * ph and 0.7 <= bh / ph <= 1.43:
            return self.iou_thr * (1.0 - d / (0.5 * ph)) * 0.99
        return None

    def _assign(self, boxes, tids, now):
        pred = {tid: self._predict(self.tracks[tid], now) for tid in tids}
        pairs = []
        for bi, b in enumerate(boxes):
            for tid in tids:
                sc = self._score(b, pred[tid])
                if sc is not None:
                    pairs.append((-sc, bi, tid))
        pairs.sort()
        used_b, used_t, got = set(), set(), {}
        for _, bi, tid in pairs:
            if bi in used_b or tid in used_t:
                continue
            used_b.add(bi)
            used_t.add(tid)
            got[bi] = tid
        return got

    def update(self, boxes, now=None):
        now = time.monotonic() if now is None else now
        boxes = sorted(boxes, key=lambda b: -b[4])
        high = [b for b in boxes if b[4] >= self.high_conf]
        low = [b for b in boxes if b[4] < self.high_conf]
        live = sorted(self.tracks)
        m1 = self._assign(high, live, now)
        rest = [tid for tid in live if tid not in m1.values()]
        m2 = self._assign(low, rest, now)
        chosen = [(b, m1.get(i)) for i, b in enumerate(high)] + [(low[i], tid) for i, tid in sorted(m2.items())]
        out = []
        for b, tid in chosen:
            if tid is None:
                tid = self.next_id
                self.next_id += 1
                self.tracks[tid] = {'hist': [], 'vel': [0.0, 0.0, 0.0, 0.0]}
            else:
                t = self.tracks[tid]
                dt = now - t['seen']
                if dt > 1e-3:
                    o = t['box']
                    raw = [((b[0] + b[2]) - (o[0] + o[2])) / 2 / dt, ((b[1] + b[3]) - (o[1] + o[3])) / 2 / dt,
                           ((b[2] - b[0]) - (o[2] - o[0])) / dt, ((b[3] - b[1]) - (o[3] - o[1])) / dt]
                    t['vel'] = [0.5 * p + 0.5 * r for p, r in zip(t['vel'], raw)]
            t = self.tracks[tid]
            h = b[3] - b[1]
            t['box'], t['seen'] = b, now
            t['hist'] = [p for p in t['hist'] if now - p[0] <= self.window_s * 2] + [(now, h)]
            old = [p for p in t['hist'] if now - p[0] >= self.window_s]
            ref = old[-1][1] if old else t['hist'][0][1]
            growth = (h - ref) / ref if ref > 0 else 0.0
            out.append({'id': tid, 'box': [int(v) for v in b[:4]], 'conf': round(b[4], 2),
                        'approaching': growth > self.change, 'moving_away': growth < -self.change})
        for tid in [k for k, t in self.tracks.items() if now - t.get('seen', now) > self.forget_s]:
            del self.tracks[tid]
        return out


# ------------------------------------------------------------------ geometry
def bearing_deg(box, frame_w, hfov_deg):
    """Horizontal angle of the box centre: <0 = left of the camera axis, >0 = right."""
    cx = (box[0] + box[2]) / 2.0
    return (cx - frame_w / 2.0) / frame_w * hfov_deg


def lidar_distance(bearing, ranges, angle_min, angle_inc, laser_yaw=0.0, half_window_deg=3.0,
                   range_min=0.15, range_max=8.0):
    """Closest LiDAR range (m) within +-half_window of a camera bearing (deg, >0 = right).
    Camera bearing is clockwise, ROS angles counter-clockwise: base angle = -bearing."""
    if not ranges or angle_inc == 0:
        return None
    n = len(ranges)
    centre = -math.radians(bearing) - laser_yaw
    best = None
    steps = int(math.ceil(math.radians(half_window_deg) / abs(angle_inc)))
    i0 = (centre - angle_min) / angle_inc
    for k in range(-steps, steps + 1):
        i = int(round(i0 + k)) % n
        r = ranges[i]
        if r is not None and math.isfinite(r) and range_min <= r <= range_max:
            best = r if best is None else min(best, r)
    return best


# ------------------------------------------------------------------ decision
def _d(action, speed, reason, steer='none', in_path=0, closest=None):
    return {'action': action, 'speed_factor': round(max(0.0, min(1.0, speed)), 2), 'steer': steer,
            'reason': reason, 'people_in_path': in_path, 'closest': closest}


def decide(people, frame_h, cfg=None):
    """people: dicts with box, bearing_deg, approaching, optional distance_m (from the LiDAR)."""
    c = dict(DEFAULTS, **(cfg or {}))
    if not frame_h:
        return _d('STOP', 0.0, 'no camera picture')
    if not people:
        return _d('CONTINUE', 1.0, 'no people in view')

    info = []
    for p in people:
        ratio = (p['box'][3] - p['box'][1]) / float(frame_h)
        dist = p.get('distance_m')
        if dist is not None:
            level = 'very close' if dist <= c['stop_m'] else 'near' if dist <= c['slow_m'] else 'far'
        else:
            level = ('very close' if ratio >= c['very_close_ratio'] else
                     'near' if ratio >= c['near_ratio'] else 'far')
        info.append({'id': p['id'], 'bearing_deg': round(p['bearing_deg'], 1), 'distance_m': dist,
                     'level': level, 'in_path': abs(p['bearing_deg']) <= c['path_half_angle_deg'],
                     'approaching': bool(p.get('approaching'))})

    def key(i):                                     # closest first: metres if known, else box size level
        return (i['distance_m'] if i['distance_m'] is not None else
                {'very close': 0.5, 'near': 1.5, 'far': 5.0}[i['level']])
    closest = min(info, key=key)
    in_path = sorted([i for i in info if i['in_path']], key=key)
    n = len(in_path)
    steer = 'right' if closest['bearing_deg'] < 0 else 'left'      # pass on the side away from them

    def where(i):
        d = f" {i['distance_m']:.1f} m" if i['distance_m'] is not None else ''
        return f"person{d} {'ahead' if i['in_path'] else 'to the ' + ('left' if i['bearing_deg'] < 0 else 'right')}"

    for i in in_path:
        if i['level'] == 'very close':
            return _d('STOP', 0.0, f"{where(i)}: too close, waiting", in_path=n, closest=closest)
        if i['level'] == 'near' and i['approaching']:
            return _d('STOP', 0.0, f"{where(i)} coming towards me, waiting", in_path=n, closest=closest)
        if i['level'] == 'near':
            return _d('SLOW_DOWN', 0.4, f"{where(i)}: slowing down", steer, n, closest)
        if i['approaching']:
            return _d('SLOW_DOWN', 0.6, f"{where(i)} approaching: slowing down", steer, n, closest)
    if in_path:
        return _d('CONTINUE', 0.8, 'people far ahead: careful', in_path=n, closest=closest)
    close_side = closest['distance_m'] is not None and closest['distance_m'] <= c['side_slow_m']
    if close_side or (closest['distance_m'] is None and closest['level'] == 'very close'):
        return _d('SLOW_DOWN', 0.5, f"{where(closest)}, very close: slowing down", steer, 0, closest)
    return _d('CONTINUE', 1.0, 'people in view, none in my path', closest=closest)


# ------------------------------------------------------------------ phone detections + idle pause
class ActivityWatch:
    """Is people detection needed right now? Only while the robot is busy (driving, docking, ...) and for
    idle_pause_s after it stopped; the app's camera view or a phone asking for frames also count.
    No navigator status for 5 s (navigator not running) = always on, to be safe."""

    IDLE_STATES = ('IDLE', 'CHARGING')

    def __init__(self, idle_pause_s=30.0):
        self.idle_pause_s = idle_pause_s
        self.state, self.state_t, self.busy_t = None, -1e9, -1e9

    def update(self, state, now):
        self.state, self.state_t = state, now
        if state not in self.IDLE_STATES:
            self.busy_t = now

    def poke(self, now):
        """Someone is watching (camera view open, phone fetching frames)."""
        self.busy_t = now

    def wanted(self, now):
        if self.idle_pause_s <= 0 or now - self.state_t > 5.0:
            return True
        return now - self.busy_t < self.idle_pause_s


def clean_phone_detections(body, max_boxes=30):
    """Validate a POST /api/vision/phone body -> the /vision/phone_boxes message. Raises ValueError."""
    boxes = body.get('boxes')
    w, h = body.get('w'), body.get('h')
    if not isinstance(boxes, list) or len(boxes) > max_boxes:
        raise ValueError(f'boxes must be a list (at most {max_boxes})')
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and 0 < v <= 4096 for v in (w, h)):
        raise ValueError('w and h must be the frame size in pixels')
    clean = []
    for b in boxes:
        if not (isinstance(b, list) and len(b) >= 5 and
                all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in b[:5])):
            raise ValueError('each box is [x1, y1, x2, y2, conf]')
        clean.append([round(float(v), 2) for v in b[:5]])
    ms = body.get('ms')
    return {'boxes': clean, 'w': w, 'h': h,
            'ms': ms if isinstance(ms, (int, float)) and not isinstance(ms, bool) and math.isfinite(ms) else None}


def phone_boxes(msg, frame_w, frame_h, now, max_age_s=1.0, max_boxes=30):
    """Boxes from the phone detector (/vision/phone_boxes JSON) scaled to this camera's frame, or None when
    there is nothing fresh/valid (the Pi then runs its own YOLO). msg: {boxes: [[x1,y1,x2,y2,conf]], w, h, t_mono}."""
    if not isinstance(msg, dict) or now - float(msg.get('t_mono', -1e9)) > max_age_s:
        return None
    try:
        w, h = float(msg['w']), float(msg['h'])
        boxes = msg['boxes'][:max_boxes]
        if w <= 0 or h <= 0:
            return None
        sx, sy = (frame_w / w if frame_w else 1.0), (frame_h / h if frame_h else 1.0)
        out = []
        for b in boxes:
            x1, y1, x2, y2, c = (float(v) for v in b[:5])
            if not all(math.isfinite(v) for v in (x1, y1, x2, y2, c)) or x2 <= x1 or y2 <= y1:
                continue
            out.append((x1 * sx, y1 * sy, x2 * sx, y2 * sy, c))
        return out
    except (KeyError, TypeError, ValueError, IndexError):
        return None


# ------------------------------------------------------------------ navigator side
class VisionGate:
    """Applies the latest camera decision to the navigator's command.

    Only a FRESH decision counts (no camera / no detector / stale = no effect: the LiDAR and
    ultrasonic safety work as before). Only forward motion is limited; reversing (docking) is not.
    STOP also holds turning, so the front doesn't swing into the person.
    """

    def __init__(self, enabled=True, timeout_s=1.0):
        self.enabled, self.timeout_s = enabled, timeout_s
        self.last, self.last_t = None, -1e9
        self.note = ''                      # what the camera did last tick (for the app)

    def update(self, decision, now):
        if isinstance(decision, dict) and decision.get('action') in ACTIONS:
            self.last, self.last_t = decision, now
        else:
            self.last = None

    def fresh(self, now):
        return self.last if self.enabled and self.last and now - self.last_t <= self.timeout_s else None

    def apply(self, v, w, now):
        d = self.fresh(now)
        self.note = ''
        if d is None or v <= 0.0:
            return v, w
        if d['action'] == 'STOP':
            self.note = 'camera: ' + d['reason']
            return 0.0, 0.0
        if d['action'] == 'SLOW_DOWN' or d['speed_factor'] < 1.0:
            f = max(0.0, min(1.0, float(d['speed_factor'])))
            if f < 1.0:
                self.note = 'camera: ' + d['reason']
            return v * f, w
        return v, w
