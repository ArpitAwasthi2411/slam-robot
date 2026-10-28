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
    """Gives each person an id across frames (greedy IoU match) and says whether the box is
    growing (approaching) or shrinking (moving away) over the last ~0.5 s."""

    def __init__(self, iou_thr=0.3, forget_s=2.0, window_s=0.5, change=0.08):
        self.iou_thr, self.forget_s, self.window_s, self.change = iou_thr, forget_s, window_s, change
        self.tracks = {}          # id -> {'box', 'seen', 'hist': [(t, height)]}
        self.next_id = 1

    def update(self, boxes, now=None):
        now = time.monotonic() if now is None else now
        free = dict(self.tracks)
        out = []
        for b in sorted(boxes, key=lambda b: -b[4]):
            best, best_iou = None, self.iou_thr
            for tid, t in free.items():
                iou = _iou(b, t['box'])
                if iou >= best_iou:
                    best, best_iou = tid, iou
            if best is None:
                best = self.next_id
                self.next_id += 1
                self.tracks[best] = {'hist': []}
            else:
                free.pop(best)
            t = self.tracks[best]
            h = b[3] - b[1]
            t['box'], t['seen'] = b, now
            t['hist'] = [p for p in t['hist'] if now - p[0] <= self.window_s * 2] + [(now, h)]
            old = [p for p in t['hist'] if now - p[0] >= self.window_s]
            ref = old[-1][1] if old else t['hist'][0][1]
            growth = (h - ref) / ref if ref > 0 else 0.0
            out.append({'id': best, 'box': [int(v) for v in b[:4]], 'conf': round(b[4], 2),
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
