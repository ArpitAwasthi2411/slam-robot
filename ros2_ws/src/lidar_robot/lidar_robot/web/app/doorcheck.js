/* Door check with the camera: is the door of a place OPEN or CLOSED?
 *
 * YOLO has no "door" class, so this does not recognise doors in general. It recognises ONE door it was taught:
 * the robot stops at the same spot in front of a cabin every time, so the picture of that door is repeatable.
 *   teach(place, 'closed' | 'open', frame)   store what the door looks like (a few samples each, any lighting)
 *   check(place, frame) -> {state: 'open'|'closed'|'unknown', closed, open, margin, why}
 * The comparison uses a small picture "signature":
 *   - the centre 70 % of the frame, made grey, shrunk to 48 x 36, brightness/contrast normalised  -> layout
 *   - a histogram of edge directions (8 bins) in a 4 x 3 grid                                      -> structure
 * so a closed door (one flat panel, few edges) and an open one (the room behind: furniture, edges everywhere)
 * look different even when the light changes. On the robot the LiDAR doorway check backs this up.
 * Pure functions (signature, similarity, classify) take a grey Float32Array, so test/test_door_check.py can
 * run them in Node.
 */
(function (root) {
  'use strict';
  const W = 48, H = 36, GX = 4, GY = 3, BINS = 8, MAX_SAMPLES = 4;
  const MARGIN = 0.06, MIN_SIM = 0.55;

  /** grey: Float32Array w*h (0..255). Returns {pix: Float32Array W*H normalised, hog: Float32Array GX*GY*BINS}. */
  function signature(grey, w, h) {
    const crop = 0.15, x0 = w * crop, y0 = h * crop, cw = w * (1 - 2 * crop), ch = h * (1 - 2 * crop);
    const pix = new Float32Array(W * H);
    for (let y = 0; y < H; y++) {
      for (let x = 0; x < W; x++) {           // box-average the source pixels that fall in this cell
        const sx0 = Math.floor(x0 + x * cw / W), sx1 = Math.max(sx0 + 1, Math.floor(x0 + (x + 1) * cw / W));
        const sy0 = Math.floor(y0 + y * ch / H), sy1 = Math.max(sy0 + 1, Math.floor(y0 + (y + 1) * ch / H));
        let s = 0, n = 0;
        for (let yy = sy0; yy < sy1 && yy < h; yy++) for (let xx = sx0; xx < sx1 && xx < w; xx++) { s += grey[yy * w + xx]; n++; }
        pix[y * W + x] = n ? s / n : 0;
      }
    }
    const sm = new Float32Array(W * H);                 // 3 x 3 blur: sensor noise must not look like edges
    for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
      let t = 0, k = 0;
      for (let j = -1; j <= 1; j++) for (let i = -1; i <= 1; i++) {
        const yy = y + j, xx = x + i;
        if (yy >= 0 && yy < H && xx >= 0 && xx < W) { t += pix[yy * W + xx]; k++; }
      }
      sm[y * W + x] = t / k;
    }
    pix.set(sm);
    let mean = 0; for (const v of pix) mean += v; mean /= pix.length;
    let sd = 0; for (const v of pix) sd += (v - mean) * (v - mean); sd = Math.sqrt(sd / pix.length) || 1;
    for (let i = 0; i < pix.length; i++) pix[i] = (pix[i] - mean) / sd;
    const hog = new Float32Array(GX * GY * BINS);
    for (let y = 1; y < H - 1; y++) {
      for (let x = 1; x < W - 1; x++) {
        const gx = pix[y * W + x + 1] - pix[y * W + x - 1], gy = pix[(y + 1) * W + x] - pix[(y - 1) * W + x];
        const mag = Math.hypot(gx, gy);
        if (mag < 0.05) continue;
        let a = Math.atan2(gy, gx); if (a < 0) a += Math.PI;           // edge direction, 0..pi
        const b = Math.min(BINS - 1, Math.floor(a / Math.PI * BINS));
        const cx = Math.min(GX - 1, Math.floor(x / W * GX)), cy = Math.min(GY - 1, Math.floor(y / H * GY));
        hog[(cy * GX + cx) * BINS + b] += mag;
      }
    }
    let tot = 0; for (const v of hog) tot += v;
    if (tot > 0) for (let i = 0; i < hog.length; i++) hog[i] /= tot;
    return { pix, hog, edges: tot / ((W - 2) * (H - 2)) };
  }

  /** -1..1: correlation of the layouts (best over shifts of up to 3 cells = ~6 % of the picture: the robot never stops
   *  on exactly the same spot) + overlap of the edge histograms (mapped to -1..1), averaged. */
  const SHIFT = 3;
  function ncc(a, b, dx, dy) {
    let s = 0, sa = 0, sb = 0, saa = 0, sbb = 0, n = 0;
    for (let y = Math.max(0, dy); y < Math.min(H, H + dy); y++) {
      for (let x = Math.max(0, dx); x < Math.min(W, W + dx); x++) {
        const p = a[y * W + x], q = b[(y - dy) * W + (x - dx)];
        s += p * q; sa += p; sb += q; saa += p * p; sbb += q * q; n++;
      }
    }
    const cov = s / n - (sa / n) * (sb / n), va = saa / n - (sa / n) ** 2, vb = sbb / n - (sb / n) ** 2;
    return va > 1e-9 && vb > 1e-9 ? cov / Math.sqrt(va * vb) : 0;
  }
  function similarity(a, b) {
    let best = -1;
    for (let dy = -SHIFT; dy <= SHIFT; dy++) for (let dx = -SHIFT; dx <= SHIFT; dx++) best = Math.max(best, ncc(a.pix, b.pix, dx, dy));
    let inter = 0; for (let i = 0; i < a.hog.length; i++) inter += Math.min(a.hog[i], b.hog[i]);
    return 0.5 * best + 0.5 * (2 * inter - 1);
  }

  /** refs: {closed: [sig...], open: [sig...]}. */
  function classify(sig, refs) {
    const best = list => (list || []).reduce((m, r) => Math.max(m, similarity(sig, r)), -1);
    const closed = best(refs.closed), open = best(refs.open);
    const has = k => (refs[k] || []).length > 0;
    let state = 'unknown', why;
    if (!has('closed') && !has('open')) why = 'not taught yet: show me the door closed and open';
    else if (!has('closed') || !has('open')) {
      const k = has('closed') ? 'closed' : 'open', s = k === 'closed' ? closed : open;
      if (s >= 0.6) { state = k; why = `looks like the ${k} door I was shown`; }
      else why = `only the ${k} door was taught, and this does not match it well`;
    } else if (Math.max(closed, open) < MIN_SIM) why = "doesn't look like this door at all (wrong spot, or someone in front)";
    else if (closed - open >= MARGIN) { state = 'closed'; why = 'matches the closed door'; }
    else if (open - closed >= MARGIN) { state = 'open'; why = 'matches the open door'; }
    else why = 'too close to call (half open?)';
    const r = x => Math.round(x * 100) / 100;
    return { state, closed: r(closed), open: r(open), margin: r(closed - open), why };
  }

  // ------------------------------------------------------------------ browser side: frames + storage
  let cv = null, cx = null;
  function greyFromFrame(f) {
    const w = 160, h = Math.max(1, Math.round(160 * f.h / f.w));
    if (!cv) { cv = document.createElement('canvas'); cx = cv.getContext('2d', { willReadFrequently: true }); }
    cv.width = w; cv.height = h;
    cx.drawImage(f.img, 0, 0, f.w, f.h, 0, 0, w, h);
    const d = cx.getImageData(0, 0, w, h).data, g = new Float32Array(w * h);
    for (let i = 0, j = 0; i < g.length; i++, j += 4) g[i] = 0.299 * d[j] + 0.587 * d[j + 1] + 0.114 * d[j + 2];
    return { grey: g, w, h };
  }
  const KEY = 'pathik.doors';
  const pack = s => ({ pix: Array.from(s.pix, v => Math.round(v * 100) / 100), hog: Array.from(s.hog, v => Math.round(v * 1e4) / 1e4) });
  const unpack = s => ({ pix: Float32Array.from(s.pix), hog: Float32Array.from(s.hog) });
  function load() { try { return JSON.parse(localStorage.getItem(KEY) || '{}'); } catch (e) { return {}; } }
  function save(all) { try { localStorage.setItem(KEY, JSON.stringify(all)); } catch (e) { /* storage full / private */ } }
  const norm = place => String(place || '').trim().toLowerCase();

  function teach(place, state, f) {
    if (!['open', 'closed'].includes(state)) throw new Error('state must be open or closed');
    const { grey, w, h } = greyFromFrame(f), all = load(), k = norm(place);
    const d = all[k] = all[k] || { name: place, open: [], closed: [] };
    d[state] = d[state].concat([pack(signature(grey, w, h))]).slice(-MAX_SAMPLES);
    save(all);
    return { open: d.open.length, closed: d.closed.length };
  }
  function check(place, f) {
    const d = load()[norm(place)] || { open: [], closed: [] };
    const { grey, w, h } = greyFromFrame(f);
    return classify(signature(grey, w, h), { open: d.open.map(unpack), closed: d.closed.map(unpack) });
  }
  function taught(place) { const d = load()[norm(place)]; return d ? { open: d.open.length, closed: d.closed.length } : { open: 0, closed: 0 }; }
  function forget(place) { const all = load(); delete all[norm(place)]; save(all); }
  function places() { return Object.values(load()).map(d => ({ name: d.name, open: d.open.length, closed: d.closed.length })); }

  const api = { signature, similarity, classify, teach, check, taught, forget, places, MARGIN, MIN_SIM };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.PathikDoor = api;
})(typeof window !== 'undefined' ? window : globalThis);
