/* Pathik demo robot: a small simulated robot that runs inside the app (no robot, no laptop).
   Same floor as tools/dashboard_sim.py. It answers the app's API calls (window.PathikDemo.handle),
   drives along A* routes, casts a fake LiDAR scan, and supports places, rooms, missions,
   wait-for-OK, charging dock, schedules (stored only), the mission log and "why did you stop?".
   It is a demo: exploration, Tuning Lab tests and map saving need the real robot or the laptop simulator. */
(function () {
'use strict';
const RES = 0.05, W = 300, H = 200, OX = -0.5, OY = -0.5;
const HOME = { x: 1.0, y: 4.25, yaw: 0 }, DOCK = { x: 0.42, y: 4.25, yaw: Math.PI };   // front docking: nose on the dock
const FP = [0.07, 0.37, 0.21];
const occ = new Uint8Array(W * H);
function box(x0, y0, x1, y1) {
  for (let gy = Math.floor((y0 - OY) / RES); gy <= Math.floor((y1 - OY) / RES); gy++)
    for (let gx = Math.floor((x0 - OX) / RES); gx <= Math.floor((x1 - OX) / RES); gx++)
      if (gx >= 0 && gx < W && gy >= 0 && gy < H) occ[gy * W + gx] = 1;
}
const t = 0.08;
box(0, 0, 14, t); box(0, 9 - t, 14, 9); box(0, 0, t, 9); box(14 - t, 0, 14, 9);
[[0, 2.0], [3.0, 6.5], [7.5, 11.0], [12.0, 14]].forEach(([a, b]) => box(a, 3.5, b, 3.5 + t));
[[0, 2.5], [3.5, 7.0], [8.0, 12.0], [13.0, 14]].forEach(([a, b]) => box(a, 5.0, b, 5.0 + t));
box(4.5, 0, 4.5 + t, 3.5); box(9.0, 0, 9.0 + t, 3.5); box(5.0, 5.0, 5.0 + t, 9); box(10.0, 5.0, 10.0 + t, 9);
box(0.6, 0.6, 3.8, 1.1); box(0.6, 2.4, 1.6, 2.9); box(5.4, 0.5, 8.4, 1.0); box(9.6, 0.4, 10.6, 1.2);
box(12.8, 0.4, 13.7, 2.8); box(1.0, 7.8, 3.8, 8.6); box(6.0, 8.0, 9.4, 8.6); box(10.8, 7.9, 13.4, 8.4);
box(10.6, 6.2, 11.2, 6.6); box(13.6, 6.2, 13.9, 6.6);
const isOcc = (x, y) => { const gx = Math.floor((x - OX) / RES), gy = Math.floor((y - OY) / RES); return gx < 0 || gy < 0 || gx >= W || gy >= H || occ[gy * W + gx] === 1; };
let mapB64 = null;
function mapData() {
  if (mapB64) return mapB64;
  let s = '';
  for (let i = 0; i < W * H; i++) s += String.fromCharCode(occ[i] ? 101 : 1);
  return (mapB64 = btoa(s));
}

// ---------------------------------------------------------------- planner (A* on 0.1 m cells)
const PR = 0.10, PW = Math.floor(W * RES / PR), PH = Math.floor(H * RES / PR);
const dist = new Float32Array(PW * PH).fill(1e9);
(function distanceField() {
  const q = [];
  for (let cy = 0; cy < PH; cy++) for (let cx = 0; cx < PW; cx++) {
    let o = false;
    for (let dy = 0; dy < 2 && !o; dy++) for (let dx = 0; dx < 2 && !o; dx++) o = occ[(cy * 2 + dy) * W + cx * 2 + dx] === 1;
    if (o) { dist[cy * PW + cx] = 0; q.push(cy * PW + cx); }
  }
  for (let h = 0; h < q.length; h++) {
    const i = q[h], cx = i % PW, cy = (i / PW) | 0;
    for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) {
      const nx = cx + dx, ny = cy + dy; if (nx < 0 || ny < 0 || nx >= PW || ny >= PH) continue;
      const nd = dist[i] + (dx && dy ? 1.414 : 1) * PR, j = ny * PW + nx;
      if (nd < dist[j]) { dist[j] = nd; q.push(j); }
    }
  }
})();
const R_LETHAL = 0.30, R_SOFT = 0.65;
const cellOf = (x, y) => [Math.floor((x - OX) / PR), Math.floor((y - OY) / PR)];
const centre = (cx, cy) => [OX + (cx + 0.5) * PR, OY + (cy + 0.5) * PR];
const free = i => dist[i] >= R_LETHAL;
function snap(cx, cy) {
  for (let r = 0; r <= 6; r++) for (let dy = -r; dy <= r; dy++) for (let dx = -r; dx <= r; dx++) {
    const x = cx + dx, y = cy + dy;
    if (x >= 0 && y >= 0 && x < PW && y < PH && free(y * PW + x)) return [x, y];
  }
  return null;
}
function plan(a, b) {
  const s = snap(...cellOf(a[0], a[1])), g = snap(...cellOf(b[0], b[1]));
  if (!s) return { error: 'robot is too close to an obstacle to plan' };
  if (!g) return { error: 'goal is inside or too close to an obstacle' };
  const si = s[1] * PW + s[0], gi = g[1] * PW + g[0];
  const gs = new Float32Array(PW * PH).fill(Infinity), came = new Int32Array(PW * PH).fill(-1), closed = new Uint8Array(PW * PH);
  const heap = [[0, si]]; gs[si] = 0;
  const push = it => { heap.push(it); let i = heap.length - 1; while (i) { const p = (i - 1) >> 1; if (heap[p][0] <= heap[i][0]) break; [heap[p], heap[i]] = [heap[i], heap[p]]; i = p; } };
  const pop = () => { const top = heap[0], last = heap.pop(); if (heap.length) { heap[0] = last; let i = 0; for (;;) { const l = 2 * i + 1, r = l + 1; let m = i; if (l < heap.length && heap[l][0] < heap[m][0]) m = l; if (r < heap.length && heap[r][0] < heap[m][0]) m = r; if (m === i) break; [heap[m], heap[i]] = [heap[i], heap[m]]; i = m; } } return top; };
  while (heap.length) {
    const [, cur] = pop();
    if (cur === gi) break;
    if (closed[cur]) continue; closed[cur] = 1;
    const cx = cur % PW, cy = (cur / PW) | 0;
    for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) {
      if (!dx && !dy) continue;
      const nx = cx + dx, ny = cy + dy; if (nx < 0 || ny < 0 || nx >= PW || ny >= PH) continue;
      const j = ny * PW + nx; if (!free(j) || closed[j]) continue;
      if (dx && dy && (!free(cy * PW + nx) || !free(ny * PW + cx))) continue;
      const soft = Math.max(0, (R_SOFT - dist[j]) / (R_SOFT - R_LETHAL));
      const ng = gs[cur] + (dx && dy ? 1.414 : 1) * (1 + 5 * soft * soft);
      if (ng < gs[j]) { gs[j] = ng; came[j] = cur; const hx = Math.abs(nx - g[0]), hy = Math.abs(ny - g[1]); push([ng + hx + hy - 0.586 * Math.min(hx, hy), j]); }
    }
  }
  if (!isFinite(gs[gi])) return { error: 'no free path to the goal' };
  let cells = []; for (let c = gi; c !== -1; c = came[c]) cells.push(c); cells.reverse();
  // line-of-sight smoothing that stays out of the wall margin
  const los = (i, j) => { const [x0, y0] = [cells[i] % PW, (cells[i] / PW) | 0], [x1, y1] = [cells[j] % PW, (cells[j] / PW) | 0], n = Math.max(Math.abs(x1 - x0), Math.abs(y1 - y0)); for (let k = 0; k <= n; k++) { const x = Math.round(x0 + (x1 - x0) * k / n), y = Math.round(y0 + (y1 - y0) * k / n); if (dist[y * PW + x] < R_LETHAL + 0.12) return false; } return true; };
  const keep = [0]; let i = 0;
  while (i < cells.length - 1) { let j = cells.length - 1; while (j > i + 1 && !los(i, j)) j--; keep.push(j); i = j; }
  const pts = keep.map(k => centre(cells[k] % PW, (cells[k] / PW) | 0));
  pts[0] = [a[0], a[1]]; pts[pts.length - 1] = [b[0], b[1]];
  return { path: densify(pts) };
}
function densify(pts, step = 0.1) {
  const out = [pts[0]];
  for (let k = 1; k < pts.length; k++) { const [x0, y0] = pts[k - 1], [x1, y1] = pts[k], n = Math.max(1, Math.ceil(Math.hypot(x1 - x0, y1 - y0) / step)); for (let s = 1; s <= n; s++) out.push([x0 + (x1 - x0) * s / n, y0 + (y1 - y0) * s / n]); }
  return out;
}
const plen = p => p.reduce((s, q, i) => i ? s + Math.hypot(q[0] - p[i - 1][0], q[1] - p[i - 1][1]) : 0, 0);
const decim = p => { if (!p || !p.length) return []; const o = [p[0]]; for (const q of p) if (Math.hypot(q[0] - o[o.length - 1][0], q[1] - o[o.length - 1][1]) >= 0.2) o.push(q); o.push(p[p.length - 1]); return o.map(q => [+q[0].toFixed(3), +q[1].toFixed(3)]); };
const wrap = a => Math.atan2(Math.sin(a), Math.cos(a));

// ---------------------------------------------------------------- world state
const now = () => performance.now() / 1000;
const R = { x: HOME.x, y: HOME.y, yaw: 0, v: 0, w: 0, teleop: null, estop: false, hold: false };
const places = {
  map: 'demo CSE floor', home: { ...HOME }, dock: { ...DOCK }, version: 1,
  places: [
    ['HOD office', 2.5, 7.2, 1.57, ['hod', 'head of department']], ['Lab 3', 2.2, 1.8, -1.57, ['computer lab', 'lab three']],
    ['Lab 2', 6.8, 1.6, -1.57, ['lab two']], ['Store', 12.0, 1.8, -1.57, ['store room']],
    ['Faculty room', 7.5, 7.0, 1.57, ['staff room']], ['Seminar hall', 12.4, 7.2, 1.57, ['auditorium']],
    ['Water cooler', 13.2, 4.25, 0, ['water']],
  ].map(([name, x, y, yaw, aliases]) => ({ name, x, y, yaw, aliases, note: '' })),
  rooms: [
    ['Lab 3', [[0.1, 0.1], [4.5, 0.1], [4.5, 3.5], [0.1, 3.5]], ['computers', 'projector', 'whiteboard']],
    ['Lab 2', [[4.6, 0.1], [9.0, 0.1], [9.0, 3.5], [4.6, 3.5]], ['3D printer', 'soldering station', 'oscilloscope']],
    ['Store', [[9.1, 0.1], [13.9, 0.1], [13.9, 3.5], [9.1, 3.5]], ['spare batteries', 'toolbox', 'cables']],
    ['Corridor', [[0.1, 3.6], [13.9, 3.6], [13.9, 4.95], [0.1, 4.95]], ['water cooler', 'notice board']],
    ['HOD office', [[0.1, 5.1], [5.0, 5.1], [5.0, 8.9], [0.1, 8.9]], ['printer', 'files']],
    ['Faculty room', [[5.1, 5.1], [10.0, 5.1], [10.0, 8.9], [5.1, 8.9]], ['coffee machine', 'lockers']],
    ['Seminar hall', [[10.1, 5.1], [13.9, 5.1], [13.9, 8.9], [10.1, 8.9]], ['stage', 'speakers', 'chairs']],
  ].map(([name, polygon, contents]) => ({ name, polygon, contents, aliases: [] })),
};
let nextId = 1;
const N = { state: 'IDLE', message: 'ready (demo robot)', active: null, queue: [], history: [], events: [], path: [], preview: null,
  last_failure: null, dwellUntil: 0, confirmUntil: 0, docked: false, dockRev: false, schedules: [], log: [], bat: 74, blockedSince: 0 };
function log(text, level = 'info') { N.events.unshift({ t: Date.now() / 1000, text, level }); N.events.length = Math.min(N.events.length, 40); }
const norm = s => String(s || '').toLowerCase().replace(/[^a-z0-9 ]+/g, ' ').replace(/\s+/g, ' ').trim();
function findPlace(name) {
  const n = norm(name); if (!n) return null;
  return places.places.find(p => norm(p.name) === n) || places.places.find(p => [p.name, ...p.aliases].some(a => norm(a).includes(n) || n.includes(norm(a))));
}
const pip = (x, y, P) => { let inside = false; for (let i = 0, j = P.length - 1; i < P.length; j = i++) { const [xi, yi] = P[i], [xj, yj] = P[j]; if ((yi > y) !== (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi) inside = !inside; } return inside; };
const roomAt = (x, y) => places.rooms.find(r => pip(x, y, r.polygon)) || null;
const roomTarget = r => { const p = findPlace(r.name); if (p && norm(p.name) === norm(r.name)) return { label: p.name, x: p.x, y: p.y, yaw: p.yaw }; const P = r.polygon; return { label: r.name, x: P.reduce((s, q) => s + q[0], 0) / P.length, y: P.reduce((s, q) => s + q[1], 0) / P.length, yaw: null }; };
function findThing(q) {
  const n = norm(q).replace(/^(the|a|an|my) /, ''); if (!n) return [];
  const hits = [];
  for (const r of places.rooms) for (const c of r.contents) { const cn = norm(c); if (cn === n) hits.push([1, r, c]); else if (cn.includes(n) || n.includes(cn)) hits.push([0.9, r, c]); }
  return hits.sort((a, b) => b[0] - a[0]);
}
const summary = m => m && ({ id: m.id, priority: m.priority, source: m.source, text: m.text, state: m.state, message: m.message || '', stops: m.stops.map(s => s.label), next_stop: m.next, repeat: m.repeat, round: m.round, confirm: m.stops.map((s, i) => s.confirm ? i : -1).filter(i => i >= 0) });
function enqueue(stops, priority = 1, source = 'dashboard', text = '', replace = false) {
  if (R.estop) throw new Error('release the e-stop first');
  if (replace) cancelAll('replaced');
  const m = { id: nextId++, priority, source, text: text || stops.map(s => s.label).join(' → '), state: 'QUEUED', stops, next: 0, repeat: 0, round: 1, t0: now(), dist: 0, planned: 0, replans: 0 };
  N.queue.push(m); N.queue.sort((a, b) => b.priority - a.priority || a.id - b.id);
  if (N.active && priority >= 3 && priority > N.active.priority) { N.active.state = 'QUEUED'; N.queue.push(N.active); N.active = null; N.path = []; log(`urgent mission #${m.id} pre-empted the current one`, 'warn'); }
  log(`mission #${m.id} queued: ${stops.map(s => s.label).join(' -> ')}`);
  return { ok: true, mission: summary(m) };
}
function finish(m, state, msg) {
  m.state = state; m.message = msg || ''; N.history.unshift(m); N.history.length = Math.min(N.history.length, 20);
  N.log.push({ id: m.id, text: m.text, source: m.source, priority: m.priority, stops: m.stops.map(s => s.label), started: Date.now() / 1000 - (now() - m.t0), ended: Date.now() / 1000, duration_s: +(now() - m.t0).toFixed(1), result: state, reason: msg || '', distance_m: +m.dist.toFixed(2), planned_m: +m.planned.toFixed(2), straight_m: 0, replans: m.replans, blocked_s: 0, estops: 0, bumps: 0, skipped: 0, min_gap_m: 0.18 });
}
function cancelAll(why) {
  let n = 0;
  if (N.rec && ['deciding', 'running', 'asking'].includes(N.rec.status)) recResolve(why, 'failed');
  if (N.active) { finish(N.active, 'CANCELED', why); N.active = null; n++; }
  N.queue.forEach(m => { finish(m, 'CANCELED', why); n++; }); N.queue = [];
  N.path = []; N.state = 'IDLE'; N.message = why; return n;
}
// ---------------------------------------------------------------- error recovery (same menu and card as recovery.py)
const REC_WHAT = { BLOCKED: 'The way to {p} stayed blocked.', NO_PATH: "I couldn't find a free route to {p}.",
  GOAL_IN_OBSTACLE: '{p} is inside or too close to an obstacle on the map.' };
const REC_OPTS = { retry: 'Try again', wait: 'Wait 1 min, then try', skip: 'Skip this stop', home: 'Go home', dock: 'Go and charge', cancel: 'Cancel the mission' };
let recId = 0;
N.rec = null; N.recHist = [];
function startRecovery(code, text) {
  const m = N.active; if (!m || !REC_WHAT[code]) return false;
  if (N.recHist.filter(h => h.mission === m.id).length >= 3) return false;
  if (N.rec && ['deciding', 'running', 'asking'].includes(N.rec.status)) recResolve('failed_again', 'failed');
  const place = m.stops[m.next].label;
  N.rec = { id: ++recId, t: Date.now() / 1000, code, place, mission: m.id, what_happened: REC_WHAT[code].replace('{p}', place),
    doing: 'Deciding what to do…', why: '', source: 'rules', status: 'deciding', action: null, params: {}, note: 'demo robot: the safety rules decide (no language model in the demo)',
    outcome: null, left_s: null, ask: null, _at: now() + 1.2, _until: null, _text: text };
  N.state = 'RECOVERING'; N.message = `${code} at ${place}: deciding what to do`; N.path = [];
  log(`${place}: ${text} -> recovering (rules)`, 'warn');
  return true;
}
function recResolve(outcome, status = 'done') {
  const r = N.rec; if (!r || ['done', 'failed'].includes(r.status)) return;
  r.outcome = outcome; r.status = status; r.recovered_s = +(Date.now() / 1000 - r.t).toFixed(1);
  N.recHist.unshift({ ...r }); N.recHist.length = Math.min(N.recHist.length, 20);
}
function recApply(action, params, why) {
  const r = N.rec, m = N.active, t = now();
  r.action = action; r.params = params; r.why = why;
  const doing = { wait_retry: `Waiting ${params.wait_s} s, then trying ${r.place} again.`, skip_stop: `Skipping ${r.place} and carrying on with the next stop.`,
    ask_person: 'Waiting for someone to choose in the app.', go_home: 'Giving up this mission and going home.', go_dock: 'Giving up this mission and going to charge.' }[action];
  r.doing = doing; log(`recovery (${r.source}): ${doing}`, 'warn');
  if (action === 'wait_retry') { r.status = 'running'; r._until = t + params.wait_s; return; }
  if (action === 'ask_person') {
    const more = m && m.stops.length - m.next > 1;
    r.status = 'asking'; r._until = t + 120;
    r.ask = { question: params.question || `I can't get to ${r.place}. What should I do?`,
      options: Object.entries(REC_OPTS).filter(([k]) => k !== 'skip' || more).map(([id, label]) => ({ id, label, primary: id === 'retry' })) };
    return;
  }
  if (action === 'skip_stop') { recResolve('skipped'); N.state = 'DWELL'; N.dwellUntil = t; return; }
  if (action === 'go_home' || action === 'go_dock') {
    recResolve('gave_up_' + action.slice(3));
    finish(m, 'FAILED', `${r.code}: ${r._text} (recovery: ${action})`); N.active = null; N.path = [];
    if (action === 'go_home') enqueue([{ label: 'home', ...places.home }], 2, 'recovery', 'home'); else goDock(2, 'recovery');
    N.state = 'IDLE'; N.message = doing;
  }
}
function recTick(t) {
  const r = N.rec, m = N.active;
  if (!m) { recResolve('mission gone', 'failed'); N.state = 'IDLE'; return [0, 0]; }
  if (r.status === 'deciding') {
    if (t < r._at) return [0, 0];
    const more = m.stops.length - m.next > 1, tries = N.recHist.filter(h => h.mission === m.id).length;
    if (r.code === 'GOAL_IN_OBSTACLE') recApply(more ? 'skip_stop' : 'ask_person', {}, `rule ${r.code}: the stop cannot be reached`);
    else if (tries === 0) recApply('wait_retry', { wait_s: r.code === 'BLOCKED' ? 8 : 10 }, `rule ${r.code} step 1: people and trolleys usually move`);
    else recApply('ask_person', {}, `rule ${r.code} step 2: ask a person`);
    return [0, 0];
  }
  if (r.status === 'asking') {
    if (t > r._until) { r.why = 'nobody answered in the app'; recApply('go_home', {}, r.why); }
    else N.message = 'waiting for an answer in the app';
    return [0, 0];
  }
  if (r.status === 'running' && r._until && t < r._until) { N.message = r.doing; return [0, 0]; }
  N.state = 'PLANNING'; N.message = `trying ${r.place} again`; N.path = [];
  return [0, 0];
}
function fail(code, text) {
  if (startRecovery(code, text)) return;
  if (N.rec && ['deciding', 'running', 'asking'].includes(N.rec.status)) recResolve('failed', 'failed');
  const m = N.active; if (!m) return;
  finish(m, 'FAILED', `${code}: ${text}`); N.active = null; N.path = [];
  N.last_failure = { code, text, mission: summary(m), t: Date.now() / 1000 };
  N.state = 'IDLE'; N.message = `failed: ${text}`; log(`mission #${m.id} failed: ${text}`, 'error');
}

// ---------------------------------------------------------------- simulation loop (20 Hz)
let scanPts = [], lastScan = 0;
function scan() {
  const pts = [];
  for (let i = 0; i < 160; i++) {
    const a = -Math.PI + i * 2 * Math.PI / 160, ca = Math.cos(R.yaw + a), sa = Math.sin(R.yaw + a);
    for (let r = 0.15; r < 7; r += 0.035) if (isOcc(R.x + r * ca, R.y + r * sa)) { pts.push([+(r * Math.cos(a)).toFixed(3), +(r * Math.sin(a)).toFixed(3)]); break; }
  }
  scanPts = pts;
}
function step(dt) {
  const t = now();
  let v = 0, w = 0;
  if (R.estop) { R.v = R.w = 0; N.state = 'WAITING'; N.message = 'E-STOP engaged: missions paused'; return; }
  if (R.teleop && t - R.teleop.t < 0.5) { v = R.teleop.v; w = R.teleop.w; N.docked = false; }
  else if (!R.hold) [v, w] = navStep(t, dt);
  // move with a simple body check (no driving through walls)
  const nx = R.x + v * Math.cos(R.yaw) * dt, ny = R.y + v * Math.sin(R.yaw) * dt;
  const hit = [0, 1, 2, 3, 4, 5, 6, 7].some(k => isOcc(nx + 0.2 * Math.cos(k * 0.785), ny + 0.2 * Math.sin(k * 0.785)));
  if (!hit || N.dockIn) { if (N.active) N.active.dist += Math.hypot(nx - R.x, ny - R.y); R.x = nx; R.y = ny; } else v = 0;
  R.yaw = wrap(R.yaw + w * dt); R.v = v; R.w = w;
  // battery: drains while driving, charges on the dock
  N.bat = Math.max(5, Math.min(100, N.bat + (N.docked ? 0.4 : (Math.abs(v) + Math.abs(w) > 0.01 ? -0.03 : -0.004)) * dt));
  if (t - lastScan > 0.12) { scan(); lastScan = t; }
}
function navStep(t, dt) {
  if (N.state === 'CONFIRM') {
    if (t > N.confirmUntil) { log('nobody pressed OK: continuing', 'warn'); advance(t); }
    return [0, 0];
  }
  if (N.state === 'DWELL') { if (t >= N.dwellUntil) advance(t); return [0, 0]; }
  if (N.state === 'RECOVERING') return recTick(t);
  if (!N.active) {
    if (!N.queue.length) { if (N.docked) { N.state = 'CHARGING'; N.message = `on the dock, charging (${Math.round(N.bat)} %)`; } else if (N.state !== 'IDLE') { N.state = 'IDLE'; } return [0, 0]; }
    N.active = N.queue.shift(); N.active.state = 'ACTIVE'; N.path = [];
  }
  const m = N.active, st = m.stops[m.next];
  if (N.docked) { N.docked = false; log('off the dock'); }
  if (N.dockIn) {                                   // front docking: align on the marker, creep in, contact
    const D = N.dockIn, c = Math.cos(DOCK.yaw), s = Math.sin(DOCK.yaw);
    const along = -((R.x - DOCK.x) * c + (R.y - DOCK.y) * s), lat = -(R.x - DOCK.x) * s + (R.y - DOCK.y) * c;
    const head = wrap(DOCK.yaw - R.yaw);
    D.marker = along > 0.18 ? { found: true, x_m: +(lat).toFixed(3), y_m: 0, z_m: +(along + 0.05).toFixed(3), yaw_deg: +(-head * 57.3).toFixed(1) } : { found: false };
    if (D.phase === 'align') {
      N.message = 'turning to the marker';
      if (Math.abs(head) < 0.05 && t - D.t0 > 1.2) { D.phase = 'creep'; log('marker found: creeping in'); }
      return [0, Math.sign(head) * Math.min(0.3, Math.abs(head) * 1.5)];
    }
    if (D.phase === 'creep') {
      if (along <= 0.20) { D.phase = 'contact'; D.t0 = t; }
      N.message = `creeping in: ${Math.round(along * 100)} cm to go, ${(lat * 100).toFixed(1)} cm off the line`;
      return [0.04, Math.max(-0.3, Math.min(0.3, 2 * head - 3 * lat))];
    }
    if (D.phase === 'contact') {
      if (along <= 0.004) { if (!D.touch) D.touch = t; if (t - D.touch > 1) { N.dockIn = null; N.docked = true; finish(m, 'DONE', 'charging started'); N.active = null; N.state = 'CHARGING'; N.message = 'charging started'; log('docked: charging'); return [0, 0]; } N.message = 'touching the contacts…'; return [0, 0]; }
      N.message = 'touching the contacts…';
      return [0.03, 2 * head];
    }
  }
  if (N.dockRev) {                                  // (old reverse docking, unused)
    const along = (R.x - DOCK.x) * Math.cos(DOCK.yaw) + (R.y - DOCK.y) * Math.sin(DOCK.yaw);
    if (along <= 0.005) { N.dockRev = false; N.docked = true; finish(m, 'DONE', 'charging started'); N.active = null; N.state = 'CHARGING'; N.message = 'charging started'; log('charging started'); return [0, 0]; }
    N.state = 'DOCKING'; N.message = `reversing onto the dock (${Math.round(along * 100)} cm to go)`;
    return [-0.05, 1.5 * wrap(DOCK.yaw - R.yaw)];
  }
  if (!N.path.length) {
    const r = st.path ? { path: st.path } : plan([R.x, R.y], [st.x, st.y]);
    if (!r.path) { fail(r.error.includes('inside') ? 'GOAL_IN_OBSTACLE' : 'NO_PATH', `cannot reach ${st.label}: ${r.error}`); return [0, 0]; }
    N.path = r.path; N.idx = 0; m.planned += plen(r.path); N.state = 'DRIVING'; N.message = `driving to ${st.label}`;
    log(`route to ${st.label}: ${plen(r.path).toFixed(1)} m`);
  }
  // a trolley parks in the Seminar hall door the first time Pathik goes there (shows error recovery)
  if (st.label === 'Seminar hall' && !N.trolleyDone && R.x > 10.6 && R.y > 3.6 && R.y < 4.95) {
    N.trolleyDone = true; fail('BLOCKED', 'a trolley stays in the Seminar hall door'); return [0, 0];
  }
  // pure pursuit
  const P = N.path;
  let best = N.idx, bd = 1e9;
  for (let i = N.idx; i < Math.min(P.length, N.idx + 40); i++) { const d = Math.hypot(P[i][0] - R.x, P[i][1] - R.y); if (d < bd) { bd = d; best = i; } }
  N.idx = best;
  const end = P[P.length - 1], dEnd = Math.hypot(end[0] - R.x, end[1] - R.y);
  if (dEnd < 0.1) {
    if (st.yaw != null && Math.abs(wrap(st.yaw - R.yaw)) > 0.1) { N.message = `turning at ${st.label}`; return [0, Math.sign(wrap(st.yaw - R.yaw)) * Math.max(0.2, Math.min(0.6, Math.abs(wrap(st.yaw - R.yaw))))]; }
    return arrive(t);
  }
  let la = P[P.length - 1];
  for (let i = N.idx; i < P.length; i++) if (Math.hypot(P[i][0] - R.x, P[i][1] - R.y) >= 0.45) { la = P[i]; break; }
  const alpha = wrap(Math.atan2(la[1] - R.y, la[0] - R.x) - R.yaw);
  if (Math.abs(alpha) > 1.0) return [0, Math.sign(alpha) * 0.7];
  const v = Math.min(0.25, Math.max(0.06, 0.25 * dEnd / 0.6)) * Math.max(0.3, 1 - Math.abs(alpha) / 1.2);
  return [v, Math.max(-0.8, Math.min(0.8, 2 * v * Math.sin(alpha) / Math.max(0.2, Math.hypot(la[0] - R.x, la[1] - R.y))))];
}
function arrive(t) {
  const m = N.active, st = m.stops[m.next];
  N.path = []; log(`arrived at ${st.label}`);
  if (N.rec && N.rec.status === 'running' && N.rec.mission === m.id) { recResolve('recovered'); log(`recovered: reached ${st.label}`); }
  if (st.dock) { N.dockIn = { phase: 'align', t0: t, try: 1 }; N.state = 'DOCKING'; N.message = 'in front of the dock: looking for the marker'; log('in front of the dock: aligning with the marker'); return [0, 0]; }
  if (st.confirm) { N.state = 'CONFIRM'; N.message = `at ${st.label}: waiting for OK`; N.confirmUntil = t + 300; log(`waiting for OK at ${st.label}`); return [0, 0]; }
  N.state = 'DWELL'; N.message = `at ${st.label}`; N.dwellUntil = t + (m.next < m.stops.length - 1 ? 2 : 0) + (st.wait_s || 0);
  return [0, 0];
}
function advance(t) {
  const m = N.active; if (!m) { N.state = 'IDLE'; return; }
  m.next++;
  if (m.next >= m.stops.length && m.repeat !== 0) { if (m.repeat > 0) m.repeat--; m.round++; m.next = 0; log(`mission #${m.id}: round ${m.round}`); }
  if (m.next >= m.stops.length) { finish(m, 'DONE', 'all stops reached'); log(`mission #${m.id} done`); N.active = null; N.state = 'IDLE'; N.message = `mission #${m.id} done`; return; }
  N.state = 'PLANNING'; N.message = `next stop: ${m.stops[m.next].label}`; N.path = [];
}
setInterval(() => { if (window.PathikDemo.running) step(0.05); }, 50);

// ---------------------------------------------------------------- commands (offline parser)
const NUMW = { a: 1, an: 1, one: 1, two: 2, three: 3, four: 4, five: 5, ten: 10, fifteen: 15, twenty: 20, thirty: 30 };
function parse(text) {
  const t = norm(text);
  const res = { intent: 'unknown', targets: [], priority: /\b(urgent|asap|immediately|quickly)\b/.test(t) ? 3 : 1, reply: '', source: 'rules', notes: [], steps: [], then_home: false };
  if (/^(ok|okay|done|confirm(ed)?|go ahead)$/.test(t)) return { ...res, intent: 'confirm', reply: 'Thank you.' };
  if (/\bwhy\b|what happened|what went wrong/.test(t)) return { ...res, intent: 'explain' };
  if (/\b(stop|halt|cancel|abort|freeze)\b/.test(t) && !/wait for/.test(t)) return { ...res, intent: 'stop', reply: 'Stopping.', priority: 3 };
  if (/\b(charge|dock|recharge)\b/.test(t) && !/\b(take|bring)\b/.test(t)) return { ...res, intent: 'dock', reply: 'Going to charge.' };
  const fm = t.match(/\b(?:where is|where are|where can i find|which room has|find)\s+(?:the |a |my )?(.+)$/);
  if (fm && !/\b(go|take|bring)\b/.test(t.slice(0, fm.index))) return { ...res, intent: 'find', query: fm[1] };
  if (/where are you|status|battery|what are you doing/.test(t)) return { ...res, intent: 'status' };
  const found = [];
  for (const p of places.places) for (const a of [p.name, ...p.aliases]) { const i = t.indexOf(norm(a)); if (i >= 0) { found.push([i, p.name]); break; } }
  for (const r of places.rooms) { if (findPlace(r.name)) continue; const i = t.indexOf(norm(r.name)); if (i >= 0) found.push([i, r.name]); }
  for (const r of places.rooms) for (const c of r.contents) { const i = t.indexOf(norm(c)); if (i >= 0 && !found.some(f => f[1] === roomTarget(r).label)) { found.push([i, roomTarget(r).label]); res.notes.push(`"${c}" is in ${r.name}`); } }
  found.sort((a, b) => a[0] - b[0]);
  const names = [...new Set(found.map(f => f[1]))];
  if (names.length) {
    const steps = names.map(n => ({ place: n, wait_s: 0, confirm: false }));
    const pos = n => t.indexOf(norm(n));
    for (const m of t.matchAll(/\bwait\s+(?:for\s+)?(\d+|a|an|one|two|three|four|five|ten|fifteen|twenty|thirty)\s*(seconds?|secs?|minutes?|mins?)\b/g)) {
      const val = /\d/.test(m[1]) ? +m[1] : NUMW[m[1]], secs = val * (m[2][0] === 'm' ? 60 : 1);
      const before = steps.filter(s => { const p = pos(s.place); return p >= 0 && p < m.index; });
      (before.length ? before[before.length - 1] : steps[0]).wait_s = secs;
    }
    for (const m of t.matchAll(/\bwait (?:for|until) (?:an? )?(?:ok|okay|signature|confirmation|someone|reply)/g)) {
      const before = steps.filter(s => { const p = pos(s.place); return p >= 0 && p < m.index; });
      (before.length ? before[before.length - 1] : steps[0]).confirm = true;
    }
    const thenHome = /\b(come back|return|go back|then home)\b/.test(t);
    return { ...res, intent: 'navigate', targets: names, steps, then_home: thenHome, reply: 'Going to ' + names.join(' then ') + (thenHome ? ', then coming back.' : '.') };
  }
  if (/\b(home|come back|return|go back)\b/.test(t)) return { ...res, intent: 'return_home', reply: 'Returning home.' };
  return { ...res, reply: 'I could not find a known place in that.' };
}
function target(name) { const p = places.places.find(q => q.name === name); if (p) return { label: p.name, x: p.x, y: p.y, yaw: p.yaw }; const r = places.rooms.find(q => q.name === name); return r ? roomTarget(r) : null; }
function explain() {
  const m = N.active, goal = m ? m.stops[m.next].label : 'the goal';
  let a;
  if (R.estop) a = 'The emergency stop is pressed. Release it, then press Resume.';
  else if (R.hold && (N.active || N.queue.length)) a = 'I was stopped by the e-stop earlier. My tasks are paused until someone presses Resume.';
  else if (N.rec && ['deciding', 'running', 'asking'].includes(N.rec.status)) a = `${N.rec.what_happened} ${N.rec.doing} (decided by my safety rules)`;
  else if (N.state === 'CONFIRM') a = `I'm at ${goal} and waiting for someone to press OK before I continue.`;
  else if (N.state === 'DOCKING') a = 'I am moving onto the charging dock.';
  else if (N.state === 'CHARGING') a = `I'm charging on the dock (${Math.round(N.bat)} %).`;
  else if (N.state === 'DWELL' && m) a = `I'm at ${goal}, waiting ${Math.max(0, Math.round(N.dwellUntil - now()))} s more as asked.`;
  else if (N.state === 'DRIVING') a = `I'm not stopped: I'm driving to ${goal}.`;
  else if (N.last_failure && Date.now() / 1000 - N.last_failure.t < 600) a = `My last task failed: ${N.last_failure.text}.`;
  else a = "I'm not doing anything right now: no tasks are queued.";
  return { ok: true, answer: a + ' (This is the demo robot.)', details: N.events.filter(e => e.level !== 'info').slice(0, 4).map(e => e.text), state: N.state };
}
function find(q) {
  const hits = findThing(q);
  if (!hits.length) { const p = findPlace(q); return { ok: true, answer: p ? `${p.name} is a saved place on the map.` : `I don't know where "${q}" is. Add it to a room's contents list.`, rooms: [] }; }
  const best = hits.filter(h => h[0] >= hits[0][0]).map(h => h[1].name);
  return { ok: true, answer: `"${hits[0][2]}" is in ${[...new Set(best)].join(' or ')}.`, rooms: hits.map(h => ({ name: h[1].name, item: h[2], score: h[0] })) };
}
function stopsFrom(c) {
  const stops = c.steps.map(s => { const tg = target(s.place); return { ...tg, wait_s: s.wait_s, confirm: s.confirm }; });
  if (c.then_home) stops.push({ label: 'home', ...places.home });
  return stops;
}
const goDock = (prio, src) => enqueue([{ label: 'dock', x: DOCK.x - 0.6 * Math.cos(DOCK.yaw), y: DOCK.y - 0.6 * Math.sin(DOCK.yaw), yaw: DOCK.yaw, dock: true }], prio, src, 'charge on the dock');

// ---------------------------------------------------------------- navigator requests
function navReq(q) {
  const T = q.type, pose = [R.x, R.y];
  switch (T) {
    case 'goto_place': { const p = findPlace(q.name); if (!p) throw new Error(`no place matches "${q.name}"`); return enqueue([{ label: p.name, x: p.x, y: p.y, yaw: p.yaw }], q.priority || 1, 'dashboard', p.name, q.replace); }
    case 'goto_pose': return enqueue([{ label: q.label || 'map point', x: +q.x, y: +q.y, yaw: q.yaw == null ? null : +q.yaw }], q.priority || 1, 'dashboard', '', q.replace);
    case 'goto_poses': { const r = enqueue(q.waypoints.map((w, i) => ({ label: w.label || `point ${i + 1}`, x: +w.x, y: +w.y, yaw: w.yaw == null ? null : +w.yaw })), q.priority || 1, 'dashboard', q.label || '', q.replace); const m = [N.active, ...N.queue].find(x => x && x.id === r.mission.id); m.repeat = +q.repeat || 0; return r; }
    case 'goto_room': { let room; if (q.thing) { const h = findThing(q.thing); if (!h.length) throw new Error(`no room has "${q.thing}" in its contents list`); room = h[0][1]; } else room = places.rooms.find(r => norm(r.name) === norm(q.name)); if (!room) throw new Error(`no room called "${q.name}"`); return enqueue([roomTarget(room)], q.priority || 1, 'dashboard', room.name); }
    case 'go_home': return enqueue([{ label: 'home', ...places.home }], q.priority || 1, 'dashboard', 'home', q.replace);
    case 'go_dock': if (q.replace) cancelAll('going to charge'); return goDock(q.priority || 1, 'dashboard');
    case 'recovery_answer': {
      const r = N.rec;
      if (!r || r.status !== 'asking') throw new Error('nothing is waiting for an answer');
      if (q.id != null && +q.id !== r.id) throw new Error('that question was already answered');
      if (!REC_OPTS[q.choice]) throw new Error('unknown choice');
      r.source = 'person'; r.why = `${q.by || 'someone'} chose "${REC_OPTS[q.choice]}" in the app`; log(r.why);
      if (q.choice === 'cancel') { recResolve('canceled_by_person'); const m = N.active; if (m) { finish(m, 'CANCELED', 'canceled from the app'); N.active = null; } N.state = 'IDLE'; N.message = 'mission canceled from the app'; return { ok: true, canceled: true }; }
      const act = { retry: ['wait_retry', { wait_s: 3 }], wait: ['wait_retry', { wait_s: 60 }], skip: ['skip_stop', {}], home: ['go_home', {}], dock: ['go_dock', {}] }[q.choice];
      recApply(act[0], act[1], r.why);
      return { ok: true, action: act[0] };
    }
    case 'cancel': { const n = cancelAll('canceled'); log(`canceled ${n} mission(s)`); return { ok: true, canceled: n }; }
    case 'resume': if (R.estop) throw new Error('release the e-stop first'); R.hold = false; N.state = 'IDLE'; N.message = 'resumed'; log('missions resumed'); return { ok: true };
    case 'confirm_localization': return { ok: true };
    case 'confirm': if (N.state !== 'CONFIRM') return { ok: false, error: 'nothing is waiting for OK' }; log('OK received'); advance(now()); return { ok: true };
    case 'explain': return explain();
    case 'find': return find(q.q || '');
    case 'preview': { const tg = q.name ? findPlace(q.name) : { x: +q.x, y: +q.y, name: q.label }; if (!tg) throw new Error('unknown place'); const r = plan(pose, [tg.x, tg.y]); if (!r.path) { N.preview = { label: tg.name, path: [], error: r.error }; return { ok: false, error: r.error }; } const L = plen(r.path); N.preview = { label: tg.name || 'map point', path: decim(r.path), length: +L.toFixed(2), eta_s: Math.round(L / 0.16), note: '' }; return { ok: true, ...N.preview }; }
    case 'check_path': { const pts = densify(q.points || []); const L = plen(pts); return { ok: true, path: pts.filter((_, i) => i % 2 === 0).map(p => [+p[0].toFixed(3), +p[1].toFixed(3)]), length: +L.toFixed(2), blocked: [], eta_s: Math.round(L / 0.16) }; }
    case 'follow_path': { const pts = densify(q.points || []); const e = pts[pts.length - 1]; const r = enqueue([{ label: q.label || 'drawn path', x: e[0], y: e[1], yaw: null, path: pts }], 1, 'dashboard', `drawn path ${plen(pts).toFixed(1)} m`, q.replace); const m = [N.active, ...N.queue].find(x => x && x.id === r.mission.id); m.repeat = +q.repeat || 0; return r; }
    case 'command': {
      const c = parse(q.text);
      const res = { ok: true, interpretation: c };
      if (q.dry_run) {
        let steps = [];
        if (c.intent === 'navigate') { steps = c.steps.map(s => ({ kind: 'go', label: s.place, wait_s: s.wait_s, confirm: s.confirm })); if (c.then_home) steps.push({ kind: 'home', label: 'home', wait_s: 0, missing: false }); }
        else if (c.intent === 'return_home') steps = [{ kind: 'home', label: 'home', wait_s: 0, missing: false }];
        else if (c.intent === 'dock') steps = [{ kind: 'dock', label: 'charging dock', wait_s: 0, missing: false }];
        res.plan = { intent: c.intent, priority: c.priority, steps }; return res;
      }
      if (c.intent === 'stop') cancelAll('stopped by command');
      else if (c.intent === 'return_home') Object.assign(res, enqueue([{ label: 'home', ...places.home }], c.priority, 'command', q.text));
      else if (c.intent === 'dock') Object.assign(res, goDock(c.priority, 'command'));
      else if (c.intent === 'navigate') Object.assign(res, enqueue(stopsFrom(c), c.priority, 'command', q.text));
      else if (c.intent === 'status') res.status = `${N.message} · battery ${Math.round(N.bat)} %`;
      else if (c.intent === 'explain') { res.answer = explain().answer; c.reply = res.answer; }
      else if (c.intent === 'find') { res.answer = find(c.query).answer; c.reply = res.answer; }
      else if (c.intent === 'confirm') { if (N.state === 'CONFIRM') advance(now()); else c.reply = 'Nothing is waiting for an OK.'; }
      log(`command "${q.text}" -> ${c.intent}`);
      return res;
    }
    case 'add_place': { const [x, y, yaw] = q.here ? [R.x, R.y, R.yaw] : [+q.x, +q.y, +(q.yaw || 0)]; places.places = places.places.filter(p => norm(p.name) !== norm(q.name)); const rec = { name: q.name.trim(), x: +x.toFixed(3), y: +y.toFixed(3), yaw: +yaw.toFixed(3), aliases: q.aliases || [], note: '' }; places.places.push(rec); places.version++; log(`place saved: ${rec.name}`); return { ok: true, place: rec }; }
    case 'delete_place': places.places = places.places.filter(p => norm(p.name) !== norm(q.name)); places.version++; return { ok: true };
    case 'set_home': { const [x, y, yaw] = q.here ? [R.x, R.y, R.yaw] : [+q.x, +q.y, +(q.yaw || 0)]; places.home = { x, y, yaw }; places.version++; log('home position set'); return { ok: true, home: places.home }; }
    case 'set_dock': return { ok: false, error: 'the demo robot has a fixed dock (left end of the corridor)' };
    case 'add_room': { const list = c => (typeof c === 'string' ? c.split(',') : (c || [])).map(s => s.trim()).filter(Boolean); places.rooms = places.rooms.filter(r => norm(r.name) !== norm(q.name)); const rec = { name: q.name.trim(), polygon: q.polygon, contents: list(q.contents), aliases: list(q.aliases) }; places.rooms.push(rec); places.version++; log(`room saved: ${rec.name}`); return { ok: true, room: rec }; }
    case 'update_room': { const r = places.rooms.find(x => norm(x.name) === norm(q.name)); if (!r) throw new Error(`no room called "${q.name}"`); const list = c => (typeof c === 'string' ? c.split(',') : (c || [])).map(s => s.trim()).filter(Boolean); if (q.contents != null) r.contents = list(q.contents); if (q.aliases != null) r.aliases = list(q.aliases); if (q.new_name) r.name = q.new_name; places.version++; return { ok: true, room: r }; }
    case 'delete_room': places.rooms = places.rooms.filter(r => norm(r.name) !== norm(q.name)); places.version++; return { ok: true };
    case 'schedule_add': { if (!/^\d{1,2}:\d{2}$/.test(q.at || '')) throw new Error('time must look like 09:30'); const days = (q.days || []).slice().sort(); const rec = { id: q.id || (Math.max(0, ...N.schedules.map(s => s.id)) + 1), name: q.name || q.action.name || q.action.text || q.action.type, at: q.at, days, action: q.action, enabled: q.enabled !== false, last: null }; N.schedules = N.schedules.filter(s => s.id !== rec.id).concat(rec).sort((a, b) => a.at.localeCompare(b.at)); return { ok: true, schedule: rec, schedules: schedSummary() }; }
    case 'schedule_delete': N.schedules = N.schedules.filter(s => s.id !== +q.id); return { ok: true, schedules: schedSummary() };
    case 'schedule_enable': N.schedules.forEach(s => { if (s.id === +q.id) s.enabled = !!q.on; }); return { ok: true, schedules: schedSummary() };
    case 'schedule_list': return { ok: true, schedules: schedSummary() };
    case 'mission_log': return { ok: true, missions: N.log.slice(-(q.n || 50)).reverse(), stats: stats() };
    case 'get_tune': return { ok: true, follower: { ...FOLLOW }, ranges: FOLLOW_R };
    case 'tune': Object.assign(FOLLOW, q.follower || {}); return { ok: true, follower: { ...FOLLOW }, saved: !!q.save };
    case 'search': { const n = norm(q.q); return { ok: true, results: places.places.filter(p => [p.name, ...p.aliases].some(a => norm(a).includes(n))).map(p => ({ score: 0.9, ...p })) }; }
    case 'explore_start': throw new Error('Map by itself needs the real robot (or the laptop simulator: python3 tools/dashboard_sim.py --explore)');
    case 'explore_stop': return { ok: true, explore: { active: false, state: 'off' } };
    default: return { ok: false, error: `the demo robot does not do "${T}"` };
  }
}
const DAYN = ['Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat', 'Sun'];
const schedSummary = () => N.schedules.map(s => ({ ...s, days_text: !s.days.length ? 'every day' : s.days.join() === '0,1,2,3,4' ? 'weekdays' : s.days.map(d => DAYN[d]).join(', ') }));
function stats() {
  const rows = N.log, done = rows.filter(r => r.result === 'DONE'), failed = rows.filter(r => r.result === 'FAILED');
  const avg = a => a.length ? +(a.reduce((s, x) => s + x, 0) / a.length).toFixed(2) : null;
  return { missions: rows.length, done: done.length, failed: failed.length, canceled: rows.length - done.length - failed.length,
    success_rate: done.length + failed.length ? +(done.length / (done.length + failed.length)).toFixed(3) : null,
    avg_duration_s: avg(done.map(r => r.duration_s)), avg_speed_mps: avg(done.filter(r => r.duration_s > 1).map(r => r.distance_m / r.duration_s)),
    path_efficiency: avg(done.filter(r => r.distance_m > 0.3).map(r => Math.min(1, r.planned_m / r.distance_m))), avg_replans: avg(rows.map(r => r.replans)),
    total_distance_m: +rows.reduce((s, r) => s + r.distance_m, 0).toFixed(1), bumps: 0, estops: 0, min_gap_m: rows.length ? 0.18 : null, failure_reasons: {} };
}
const FOLLOW = { max_linear: 0.22, max_angular: 0.6, lookahead: 0.5, k_angular: 1.0, linear_accel: 0.3, angular_accel: 1.2, rotate_in_place_above: 1.2, rotate_exit_below: 0.3 };
const mk = (min, max, def, unit, help) => ({ min, max, default: def, unit, help });
const FOLLOW_R = { max_linear: mk(0.05, 0.6, 0.22, 'm/s', 'speed on straight stretches'), max_angular: mk(0.2, 2, 0.6, 'rad/s', 'limit for turning speed'), lookahead: mk(0.15, 1.5, 0.5, 'm', 'bigger = smoother'), k_angular: mk(0.2, 4, 1, '', 'turning gain'), linear_accel: mk(0.05, 2, 0.3, 'm/s²', 'speed-up'), angular_accel: mk(0.2, 5, 1.2, 'rad/s²', 'turn start'), rotate_in_place_above: mk(0.3, 3, 1.2, 'rad', 'turn on the spot above'), rotate_exit_below: mk(0.05, 1.5, 0.3, 'rad', 'drive on below') };
const WHEEL_R = { kp: mk(0, 2, 0.15, 'PWM per mm/s', 'proportional gain'), ki: mk(0, 5, 0.4, 'PWM per mm', 'integral gain'), pwm_min: mk(0, 120, 15, 'PWM', 'start-up PWM'), max_mms: mk(100, 2000, 493, 'mm/s', 'top speed'), accel: mk(0, 5000, 600, 'mm/s²', 'ramp') };
const tune = { kp: 0.15, ki: 0.4, pwm_min: 15, max_mms: 493, accel: 600 };

function navStatus() {
  const m = N.active, st = m && m.stops[m.next];
  const rem = N.path.length ? N.path.slice(N.idx || 0) : [];
  const d = rem.length ? plen([[R.x, R.y], ...rem]) : null;
  const room = roomAt(R.x, R.y);
  const pct = Math.round(N.bat);
  return {
    state: N.state, message: N.message, estop: R.estop, hold: R.hold, localized: true, last_failure: N.last_failure,
    goal: st ? { label: st.label, x: st.x, y: st.y, yaw: st.yaw } : null,
    path: decim(rem), distance: d == null ? null : +d.toFixed(2), eta_s: d == null ? null : Math.round(d / 0.16),
    blocked: false, preview: N.state === 'DRIVING' ? null : N.preview,
    missions: { active: summary(m), queue: N.queue.map(summary), history: N.history.slice(0, 8).map(summary) },
    events: N.events.slice(0, 15), has_map: true, explore: { active: false, state: 'off' },
    footprint: FP, tight: [], docked: N.docked, room: room ? room.name : null,
    battery: { v: +(9.9 + 2.7 * pct / 100).toFixed(2), pct, a: N.docked ? -1.5 : (Math.abs(R.v) > 0.01 ? 1.6 : 0.45), charging: N.docked, level: N.docked ? 'charging' : pct < 12 ? 'critical' : pct < 25 ? 'low' : 'ok', src: 'demo' },
    confirm: N.state === 'CONFIRM' && st ? { label: st.label, left_s: Math.max(0, Math.round(N.confirmUntil - now())) } : null,
    schedules: schedSummary(), log_stats: stats(),
    dock_direction: 'front',
    docking: N.dockIn ? { phase: N.dockIn.phase, try: N.dockIn.try, tries: 3, message: N.message, marker: N.dockIn.marker || null }
      : (m && st && st.dock) ? { phase: 'predock', try: null, tries: 3, message: 'driving to the front of the dock', marker: null }
      : (N.docked && N.state === 'CHARGING') ? { phase: 'charging', try: null, tries: 3, message: `on the dock, ${pct} %`, marker: null } : null,
    recovery: N.rec ? Object.fromEntries(Object.entries({ ...N.rec, left_s: N.rec._until && ['running', 'asking'].includes(N.rec.status) ? Math.max(0, Math.round(N.rec._until - now())) : null })
      .filter(([k]) => !k.startsWith('_'))) : null, recovery_mode: 'rules',
  };
}
function demoVision() {                      // a person walks towards the robot and away again (20 s)
  const ph = (now() % 20) / 20, d = +(0.6 + 2.6 * Math.abs(1 - 2 * ph)).toFixed(2), b = +(6 * Math.sin(ph * 6.28)).toFixed(1);
  const approaching = ph < 0.5, p = { id: 1, bearing_deg: b, distance_m: d, approaching, moving_away: !approaching };
  let action = 'CONTINUE', speed = 0.8, reason = 'people far ahead: careful';
  if (d <= 0.8) { action = 'STOP'; speed = 0; reason = `person ${d.toFixed(1)} m ahead: too close, waiting`; }
  else if (d <= 1.8 && approaching) { action = 'STOP'; speed = 0; reason = `person ${d.toFixed(1)} m ahead coming towards me, waiting`; }
  else if (d <= 1.8) { action = 'SLOW_DOWN'; speed = 0.4; reason = `person ${d.toFixed(1)} m ahead: slowing down`; }
  return { action, speed_factor: speed, reason, steer: 'none', people: [p], people_in_path: 1, camera_ok: true,
           camera: 'demo', detector: true, fps: 4.0, lidar: true, frame_w: 640, frame_h: 360 };
}
function state() {
  const moving = Math.abs(R.v) + Math.abs(R.w) > 0.001;
  return {
    time: Date.now() / 1000, demo: true,
    link: { connected: true, port: 'demo', rx_hz: 50, rx_age: 0.02, esp_mode: R.estop ? 'ESTOP' : moving ? 'AUTO' : 'IDLE', esp_info: 'demo robot', cmd_source: R.teleop && now() - R.teleop.t < 0.5 ? 'teleop' : N.active ? 'nav' : 'none', bad_lines: 0 },
    bridge_alive: true, estop: R.estop,
    pose: { frame: 'map', x: +R.x.toFixed(3), y: +R.y.toFixed(3), yaw: +R.yaw.toFixed(4) },
    odom: { v: +R.v.toFixed(3), w: +R.w.toFixed(3) }, cmd: { v: +R.v.toFixed(3), w: +R.w.toFixed(3) },
    scan: { hz: 8, points: scanPts }, goal: {},
    map: { version: 1, width: W, height: H, resolution: RES }, limits: { max_v: 0.3, max_w: 1.2 }, nav_mode: 'planner',
    sensors: { us: [null, null, null], imu_ok: true, yaw_source: 'gyro', us_mounts: [[0.07, 0.12, 0.52], [0.07, 0, 0], [0.07, -0.12, -0.52]], bumper: false },
    nav: navStatus(), places: { map: places.map, home: places.home, dock: places.dock, places: places.places, rooms: places.rooms, version: places.version },
    tune: { ...tune }, fw: '2.4-demo', telemetry: false, turns: 0, sep: 0.3, test_running: false, autosave: {}, slam_mode: 'localization',
    vision: demoVision(), camera: true,
  };
}

// ---------------------------------------------------------------- accounts (same rules as lidar_robot/auth.py)
const ROLES = ['staff', 'operator', 'admin'];
const USERS = {
  admin: { name: 'Asha Verma', role: 'admin', status: 'active', pw: 'admin123', created: 1729000000 },
  operator: { name: 'Robot team', role: 'operator', status: 'active', pw: 'operator123', created: 1729100000 },
  staff: { name: 'Prof. Sharma', role: 'staff', status: 'active', pw: 'staff123', created: 1729200000 },
  'dr.mehta': { name: 'Dr. Mehta', role: 'staff', status: 'pending', pw: 'mehta123', created: 1729300000 },
};
const STAFF_NAV = new Set(['goto_place', 'goto_room', 'command', 'preview', 'cancel', 'resume', 'go_home', 'go_dock',
  'confirm', 'explain', 'find', 'search', 'schedule_list', 'mission_log', 'recovery_answer']);
function needRole(p, body) {
  if (['/api/ping', '/api/auth/login', '/api/auth/signup'].includes(p)) return null;
  if (p.startsWith('/api/users')) return 'admin';
  if (['/api/state', '/api/map', '/api/auth/me', '/api/auth/logout', '/api/auth/password'].includes(p)) return 'staff';
  if (p === '/api/estop') return (body || {}).on === false ? 'operator' : 'staff';
  if (p === '/api/nav') return STAFF_NAV.has((body || {}).type) ? 'staff' : 'operator';
  return 'operator';
}
const pub = (n, u) => ({ username: n, name: u.name, role: u.role, status: u.status, created: u.created, last_login: u.last_login || null });
const deny = (msg, code) => { const e = new Error(msg); e.code = code; throw e; };
function authApi(p, body, me) {
  body = body || {};
  if (p === '/api/auth/login') {
    const n = String(body.username || '').trim().toLowerCase(), u = USERS[n];
    if (!u || u.pw !== body.password) deny('Wrong username or password', 401);
    if (u.status === 'pending') deny('Your account is waiting for an admin to approve it', 403);
    if (u.status === 'disabled') deny('This account has been disabled. Ask an admin.', 403);
    u.last_login = Date.now() / 1000;
    return { token: 'demo-' + n, user: pub(n, u) };
  }
  if (p === '/api/auth/signup') {
    const n = String(body.username || '').trim().toLowerCase();
    if (!/^[a-z0-9._-]{3,32}$/.test(n)) deny('Username: 3-32 characters, letters, digits, dot, dash or underscore', 400);
    if ((body.password || '').length < 6) deny('Password must be at least 6 characters', 400);
    if (USERS[n]) deny('That username is taken', 409);
    USERS[n] = { name: body.name || n, role: 'staff', status: 'pending', pw: body.password, created: Date.now() / 1000 };
    return { token: null, user: pub(n, USERS[n]), message: 'Account created. An admin has to approve it before you can log in. (Demo: log in as admin to approve it.)' };
  }
  if (p === '/api/auth/me') return { user: me, pending: me.role === 'admin' ? Object.values(USERS).filter(u => u.status === 'pending').length : 0 };
  if (p === '/api/auth/logout') return { ok: true };
  if (p === '/api/auth/password') {
    if (USERS[me.username].pw !== body.old) deny('Current password is wrong', 401);
    if ((body.new || '').length < 6) deny('Password must be at least 6 characters', 400);
    USERS[me.username].pw = body.new; return { ok: true, message: 'Password changed.' };
  }
  if (p === '/api/users') {
    const order = { pending: 0, active: 1, disabled: 2 };
    return { users: Object.entries(USERS).map(([n, u]) => pub(n, u)).sort((a, b) => order[a.status] - order[b.status] || ROLES.indexOf(b.role) - ROLES.indexOf(a.role) || a.username.localeCompare(b.username)) };
  }
  const u = USERS[body.username];
  if (!u) deny('No such user', 404);
  const admins = Object.entries(USERS).filter(([, x]) => x.role === 'admin' && x.status === 'active').map(([n]) => n);
  const lastAdmin = u.role === 'admin' && u.status === 'active' && admins.length === 1;
  if (p === '/api/users/update') {
    if (lastAdmin && ((body.role && body.role !== 'admin') || body.status === 'disabled')) deny('This is the only admin: make someone else admin first', 409);
    if (body.role) u.role = body.role;
    if (body.status) u.status = body.status;
    return { ok: true, user: pub(body.username, u) };
  }
  if (p === '/api/users/reset') { if ((body.password || '').length < 6) deny('Password must be at least 6 characters', 400); u.pw = body.password; return { ok: true }; }
  if (p === '/api/users/delete') {
    if (body.username === me.username) deny('You cannot remove your own account', 400);
    if (lastAdmin) deny('This is the only admin: make someone else admin first', 409);
    delete USERS[body.username]; return { ok: true };
  }
  deny('not found', 404);
}

// ---------------------------------------------------------------- the app's HTTP API, in-process
window.PathikDemo = {
  running: false,
  info: { base: 'demo:', robot: 'Pathik demo (simulated)', host: 'demo', api: 2, fw: '2.4-demo', ms: 0, auth: true, setup_needed: false },
  accounts: [['admin', 'admin123', 'Admin'], ['operator', 'operator123', 'Operator'], ['staff', 'staff123', 'Staff']],
  handle(path, method, body, token) {
    this.running = true;
    const p = path.split('?')[0];
    if (p === '/api/ping') return { ok: true, robot: this.info.robot, api: 2, fw: '2.4-demo', host: 'demo', nav_mode: 'planner', wheel_ranges: WHEEL_R, auth: true, setup_needed: false };
    const need = needRole(p, body);
    const n = (token || '').startsWith('demo-') ? token.slice(5) : '', u = USERS[n];
    const me = u && u.status === 'active' ? pub(n, u) : null;
    if (need) {
      if (!me) deny('Please log in again', 401);
      if (ROLES.indexOf(me.role) < ROLES.indexOf(need)) deny(`Your role (${me.role}) cannot do this. Ask an ${need}.`, 403);
    }
    if (p.startsWith('/api/auth/') || p.startsWith('/api/users')) return authApi(p, body, me);
    if (p === '/api/nav' && body) body.by = n;
    if (p === '/api/state') return state();
    if (p === '/api/map') return path.includes('since=1') ? { unchanged: true } : { version: 1, width: W, height: H, resolution: RES, origin: [OX, OY, 0], data: mapData() };
    if (p === '/api/telemetry') return { seq: 0, samples: [] };
    if (p === '/api/cmd') { R.teleop = { v: Math.max(-0.3, Math.min(0.3, +body.v || 0)), w: Math.max(-1.2, Math.min(1.2, +body.w || 0)), t: now() }; return { ok: true }; }
    if (p === '/api/estop') { const on = body.on !== false; if (on && !R.estop) { R.hold = true; if (N.active) { N.active.state = 'QUEUED'; N.queue.unshift(N.active); N.active = null; N.path = []; } log('E-stop engaged, missions paused', 'warn'); } if (!on && R.estop) { N.state = 'WAITING'; N.message = 'E-stop released: press Resume to continue missions'; } R.estop = on; return { ok: true }; }
    if (p === '/api/nav') { try { return navReq(body || {}); } catch (e) { return { ok: false, error: e.message }; } }
    if (p === '/api/goal') return navReq({ type: 'goto_pose', ...body });
    if (p === '/api/cancel') return navReq({ type: 'cancel' });
    if (p === '/api/relocalize') { R.x = +body.x; R.y = +body.y; R.yaw = +(body.yaw || 0); return { ok: true }; }
    if (p === '/api/esp') { if (body.op === 'tune') Object.assign(tune, body.values || {}); return { ok: true }; }
    if (p === '/api/save_map') throw new Error('Saving maps needs the real robot (this is the demo floor).');
    if (p === '/api/test') { if (method === 'GET') return { running: false }; throw new Error('Tuning Lab tests need the real robot.'); }
    throw new Error(`demo robot: ${p} not available`);
  },
};
})();
