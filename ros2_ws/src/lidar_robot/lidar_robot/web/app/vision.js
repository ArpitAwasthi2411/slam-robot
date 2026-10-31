/* Pathik people-detection rules in JavaScript: the same logic as lidar_robot/vision_rules.py on the robot.
 *
 *   letterboxParams, decodeYolo (+ nms)   YOLO11 output -> person boxes in frame pixels
 *   PersonTracker                          ids across frames, approaching / moving away
 *   bearingDeg, cameraDistance             angle of a person; distance estimate from the box height
 *   decide(people, frameH, cfg)            STOP / SLOW_DOWN / CONTINUE + speed factor + reason
 *
 * Used by detect.html (the phone detector). test/test_phone_detector.py runs this file in Node and checks
 * every function against the Python original, so the phone and the robot decide the same way.
 */
(function (root) {
  'use strict';
  const DEFAULTS = {
    path_half_angle_deg: 15.0, stop_m: 0.8, slow_m: 1.8, side_slow_m: 0.6,
    very_close_ratio: 0.60, near_ratio: 0.35,
  };

  function letterboxParams(w, h, size) {
    const s = Math.min(size / w, size / h);
    const nw = Math.round(w * s), nh = Math.round(h * s);
    return [s, Math.floor((size - nw) / 2), Math.floor((size - nh) / 2), nw, nh];
  }

  function iou(a, b) {
    const ix = Math.max(0, Math.min(a[2], b[2]) - Math.max(a[0], b[0]));
    const iy = Math.max(0, Math.min(a[3], b[3]) - Math.max(a[1], b[1]));
    const inter = ix * iy;
    const union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter;
    return union > 0 ? inter / union : 0;
  }

  function nms(boxes, iouThr = 0.5) {
    const keep = [];
    for (const b of [...boxes].sort((p, q) => q[4] - p[4])) {
      if (keep.every(k => iou(b, k) < iouThr)) keep.push(b);
    }
    return keep;
  }

  /** out: Float32Array of a [1, 84, N] YOLO11 output (cx, cy, w, h, 80 class scores per column). */
  function decodeYolo(out, n, frameW, frameH, size, conf = 0.4, cls = 0) {
    const [s, px, py] = letterboxParams(frameW, frameH, size);
    const boxes = [];
    const row = (r, i) => out[r * n + i];
    for (let i = 0; i < n; i++) {
      const sc = row(4 + cls, i);
      if (sc < conf) continue;
      const cx = row(0, i), cy = row(1, i), bw = row(2, i), bh = row(3, i);
      const x1 = Math.max(0, (cx - bw / 2 - px) / s), y1 = Math.max(0, (cy - bh / 2 - py) / s);
      const x2 = Math.min(frameW, (cx + bw / 2 - px) / s), y2 = Math.min(frameH, (cy + bh / 2 - py) / s);
      if (x2 > x1 && y2 > y1) boxes.push([x1, y1, x2, y2, sc]);
    }
    return nms(boxes);
  }

  class PersonTracker {
    constructor(iouThr = 0.3, forgetS = 2.0, windowS = 0.5, change = 0.08) {
      Object.assign(this, { iouThr, forgetS, windowS, change, tracks: new Map(), nextId: 1 });
    }

    update(boxes, now) {
      const free = new Map(this.tracks), out = [];
      for (const b of [...boxes].sort((p, q) => q[4] - p[4])) {
        let best = null, bestIou = this.iouThr;
        for (const [tid, t] of free) {
          const v = iou(b, t.box);
          if (v >= bestIou) { best = tid; bestIou = v; }
        }
        if (best === null) { best = this.nextId++; this.tracks.set(best, { hist: [] }); }
        else free.delete(best);
        const t = this.tracks.get(best), h = b[3] - b[1];
        t.box = b; t.seen = now;
        t.hist = t.hist.filter(p => now - p[0] <= this.windowS * 2).concat([[now, h]]);
        const old = t.hist.filter(p => now - p[0] >= this.windowS);
        const ref = old.length ? old[old.length - 1][1] : t.hist[0][1];
        const growth = ref > 0 ? (h - ref) / ref : 0;
        out.push({ id: best, box: b.slice(0, 4).map(v => Math.trunc(v)), conf: Math.round(b[4] * 100) / 100,
                   approaching: growth > this.change, moving_away: growth < -this.change });
      }
      for (const [tid, t] of this.tracks) if (now - (t.seen ?? now) > this.forgetS) this.tracks.delete(tid);
      return out;
    }
  }

  function bearingDeg(box, frameW, hfovDeg) {
    return ((box[0] + box[2]) / 2 - frameW / 2) / frameW * hfovDeg;
  }

  /** Rough distance from the box height (a 1.7 m person, pinhole camera). Only shown, never used to decide:
   *  without the LiDAR the robot decides on the box size, exactly as vision_rules.decide does. */
  function cameraDistance(box, frameW, hfovDeg, personH = 1.7) {
    const h = box[3] - box[1];
    if (h <= 0) return null;
    const f = (frameW / 2) / Math.tan((hfovDeg * Math.PI / 180) / 2);
    return f * personH / h;
  }

  const r2 = v => Math.round(v * 100) / 100, r1 = v => Math.round(v * 10) / 10;
  function D(action, speed, reason, steer = 'none', inPath = 0, closest = null) {
    return { action, speed_factor: r2(Math.max(0, Math.min(1, speed))), steer, reason,
             people_in_path: inPath, closest };
  }

  function decide(people, frameH, cfg) {
    const c = Object.assign({}, DEFAULTS, cfg || {});
    if (!frameH) return D('STOP', 0, 'no camera picture');
    if (!people.length) return D('CONTINUE', 1, 'no people in view');
    const info = people.map(p => {
      const ratio = (p.box[3] - p.box[1]) / frameH;
      const dist = p.distance_m ?? null;
      const level = dist !== null
        ? (dist <= c.stop_m ? 'very close' : dist <= c.slow_m ? 'near' : 'far')
        : (ratio >= c.very_close_ratio ? 'very close' : ratio >= c.near_ratio ? 'near' : 'far');
      return { id: p.id, bearing_deg: r1(p.bearing_deg), distance_m: dist, level,
               in_path: Math.abs(p.bearing_deg) <= c.path_half_angle_deg, approaching: !!p.approaching };
    });
    const key = i => i.distance_m !== null ? i.distance_m : { 'very close': 0.5, near: 1.5, far: 5.0 }[i.level];
    const byKey = (a, b) => key(a) - key(b);
    const closest = info.reduce((a, b) => (key(b) < key(a) ? b : a));
    const inPath = info.filter(i => i.in_path).sort(byKey);
    const n = inPath.length;
    const steer = closest.bearing_deg < 0 ? 'right' : 'left';
    const where = i => `person${i.distance_m !== null ? ' ' + i.distance_m.toFixed(1) + ' m' : ''} ` +
      (i.in_path ? 'ahead' : 'to the ' + (i.bearing_deg < 0 ? 'left' : 'right'));
    for (const i of inPath) {
      if (i.level === 'very close') return D('STOP', 0, `${where(i)}: too close, waiting`, 'none', n, closest);
      if (i.level === 'near' && i.approaching) return D('STOP', 0, `${where(i)} coming towards me, waiting`, 'none', n, closest);
      if (i.level === 'near') return D('SLOW_DOWN', 0.4, `${where(i)}: slowing down`, steer, n, closest);
      if (i.approaching) return D('SLOW_DOWN', 0.6, `${where(i)} approaching: slowing down`, steer, n, closest);
    }
    if (n) return D('CONTINUE', 0.8, 'people far ahead: careful', 'none', n, closest);
    const closeSide = closest.distance_m !== null && closest.distance_m <= c.side_slow_m;
    if (closeSide || (closest.distance_m === null && closest.level === 'very close')) {
      return D('SLOW_DOWN', 0.5, `${where(closest)}, very close: slowing down`, steer, 0, closest);
    }
    return D('CONTINUE', 1, 'people in view, none in my path', 'none', 0, closest);
  }

  const api = { DEFAULTS, letterboxParams, iou, nms, decodeYolo, PersonTracker, bearingDeg, cameraDistance, decide };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.PathikVision = api;
})(typeof window !== 'undefined' ? window : globalThis);
