/* Pathik: mobile app for the SLAM robot (no dependencies, works offline inside the APK
   and when served by the robot itself at http://<robot>:8080/). */
(() => {
'use strict';

// ------------------------------------------------------------------ utilities
const $ = (s, r = document) => r.querySelector(s);
const $$ = (s, r = document) => [...r.querySelectorAll(s)];
const native = window.PathikNative || null;
const store = {
  get(k, d) { try { const v = localStorage.getItem('pathik.' + k); return v == null ? d : JSON.parse(v); } catch (e) { return d; } },
  set(k, v) { try { localStorage.setItem('pathik.' + k, JSON.stringify(v)); } catch (e) { /* private mode */ } },
};
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const fin = v => typeof v === 'number' && isFinite(v);
const fmt = (v, d = 2) => fin(v) ? v.toFixed(d) : '–';
const esc = s => String(s ?? '').replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const sleep = ms => new Promise(r => setTimeout(r, ms));
function haptic(ms = 12) {
  try { if (native && native.haptic) native.haptic(ms); else if (navigator.vibrate) navigator.vibrate(ms); } catch (e) { /* no-op */ }
}
let toastT = 0;
function toast(msg, err = false) {
  const t = $('#toast');
  t.textContent = msg; t.classList.toggle('err', err); t.classList.add('show');
  clearTimeout(toastT); toastT = setTimeout(() => t.classList.remove('show'), err ? 4200 : 2600);
}
function debounce(fn, ms) { let t; return (...a) => { clearTimeout(t); t = setTimeout(() => fn(...a), ms); }; }
function secs(s) { if (!fin(s)) return '–'; return s < 60 ? `${Math.round(s)} s` : `${Math.floor(s / 60)} min ${Math.round(s % 60)} s`; }
function clock(epoch) { const d = new Date(epoch * 1000); return d.toTimeString().slice(0, 5); }

// ------------------------------------------------------------------ API
let BASE = '';
async function req(path, { method = 'GET', body, timeout = 4000 } = {}) {
  const ctl = new AbortController();
  const t = setTimeout(() => ctl.abort(), timeout);
  try {
    // text/plain avoids a CORS preflight; the robot parses the body as JSON regardless
    const r = await fetch(BASE + path, {
      method, signal: ctl.signal, cache: 'no-store',
      body: body === undefined ? undefined : JSON.stringify(body),
      headers: body === undefined ? undefined : { 'Content-Type': 'text/plain' },
    });
    const j = await r.json().catch(() => ({}));
    if (!r.ok) throw new Error(j.error || `robot answered ${r.status}`);
    return j;
  } catch (e) {
    if (e.name === 'AbortError') throw new Error('robot did not answer in time');
    throw e;
  } finally { clearTimeout(t); }
}
const get = (p, o) => req(p, o);
const post = (p, b = {}, o = {}) => req(p, { ...o, method: 'POST', body: b });
async function nav(body) {
  const r = await post('/api/nav', body);
  if (r.ok === false) throw new Error(r.error || 'navigator refused');
  return r;
}

// ------------------------------------------------------------------ discovery
async function probe(base, timeout = 900) {
  const ctl = new AbortController();
  const t = setTimeout(() => ctl.abort(), timeout);
  try {
    const r = await fetch(base + '/api/ping', { signal: ctl.signal, cache: 'no-store' });
    const j = await r.json();
    if (j && j.api) return { base, ...j };
  } catch (e) { /* not a robot */ } finally { clearTimeout(t); }
  return null;
}
function subnetsToScan() {
  let list = [];
  try { if (native && native.subnets) list = JSON.parse(native.subnets()); } catch (e) { list = []; }
  if (!list.length) list = ['192.168.43', '192.168.137', '192.168.1', '192.168.0', '192.168.4', '172.20.10'];
  return [...new Set(['10.42.0', ...list])];
}
let scanning = false;
async function scan() {
  if (scanning) return;
  scanning = true;
  const btn = $('#c-scan'), prog = $('#c-progress'), list = $('#c-list');
  btn.disabled = true; btn.textContent = 'Looking…';
  const found = new Map();
  const add = r => {
    if (!r || found.has(r.base)) return;
    found.set(r.base, r);
    renderFound([...found.values()]);
    haptic(20);
  };
  list.innerHTML = '';
  const quick = [...new Set([
    ...store.get('robots', []).map(r => r.base),
    'http://10.42.0.1:8080', 'http://169.254.1.2:8080', 'http://robot.local:8080', 'http://robot:8080',
    'http://slam-robot:8080',                       // Tailscale MagicDNS name (robot far away on mobile data)
  ])];
  prog.textContent = 'Checking the usual addresses…';
  (await Promise.all(quick.map(b => probe(b, 1500)))).forEach(add);
  const nets = subnetsToScan();
  for (const net of nets) {
    if (found.size && !native) break;              // without native help, stop at the first hit
    prog.textContent = `Scanning ${net}.x…`;
    const hosts = Array.from({ length: 254 }, (_, i) => `http://${net}.${i + 1}:8080`);
    let i = 0;
    const worker = async () => { while (i < hosts.length) add(await probe(hosts[i++], 700)); };
    await Promise.all(Array.from({ length: 48 }, worker));
  }
  prog.textContent = found.size ? `${found.size} robot${found.size > 1 ? 's' : ''} found` :
    'No robot found. Check the Wi‑Fi and that robot_up is running, or type the address.';
  btn.disabled = false; btn.textContent = 'Find again';
  scanning = false;
}
function renderFound(robots) {
  $('#c-list').innerHTML = robots.map((r, i) => `
    <li><button data-i="${i}">
      <svg viewBox="0 0 64 64"><use href="#mark"/></svg>
      <span><b>${esc(r.robot || 'robot')}</b><small>${esc(r.base.replace('http://', ''))}${r.fw ? `, firmware ${esc(r.fw)}` : ''}</small></span>
      <span class="go">Connect</span>
    </button></li>`).join('');
  $$('#c-list button').forEach(b => b.onclick = () => connect(robots[+b.dataset.i]));
}
function normalizeAddr(s) {
  s = s.trim();
  if (!s) return '';
  if (!/^https?:\/\//.test(s)) s = 'http://' + s;
  if (!/:\d+$/.test(s.replace(/\/+$/, ''))) s = s.replace(/\/+$/, '') + ':8080';
  return s.replace(/\/+$/, '');
}

// ------------------------------------------------------------------ app state
const S = {
  st: null, gen: 0, fails: 0, lastOk: 0, robot: null,
  map: null, mapVer: -1, mapImg: null,
  tab: 'drive', lab: 'wheels',
};

async function connect(robot) {
  BASE = robot.base;
  S.robot = robot;
  const saved = store.get('robots', []).filter(r => r.base !== robot.base);
  store.set('robots', [{ base: robot.base, robot: robot.robot }, ...saved].slice(0, 5));
  store.set('last', robot.base);
  $('#t-host').textContent = robot.host && robot.host !== 'simulator' ? robot.host : (robot.robot || 'robot');
  $('#connect').hidden = true; $('#main').hidden = false;
  S.gen++; S.fails = 0; S.mapVer = -1; S.mapImg = null; S.st = null;
  const g = S.gen;
  pollState(g); pollMap(g);
  resize();
  $('#m-hint').classList.remove('gone');
  setTimeout(() => $('#m-hint').classList.add('gone'), 7000);
  if (native && native.keepAwake) native.keepAwake(true);
  setTab(S.tab);
}
function disconnect() {
  S.gen++;
  telemetry(false);
  renderFound(store.get('robots', []));
  $('#c-progress').textContent = '';
  $('#main').hidden = true; $('#connect').hidden = false;
  if (native && native.keepAwake) native.keepAwake(false);
}

async function pollState(g) {
  while (g === S.gen) {
    try {
      const t0 = performance.now();
      const st = await get('/api/state', { timeout: 2500 });
      if (g !== S.gen) return;
      const rtt = performance.now() - t0;
      S.rtt = S.rtt ? S.rtt * 0.8 + rtt * 0.2 : rtt;
      S.st = st; S.fails = 0; S.lastOk = performance.now();
      onState(st);
    } catch (e) {
      S.fails++;
      onLinkLost();
    }
    // fast on local Wi-Fi; slower over mobile data (high delay) or when the app is in the background
    const busy = joy.active || S.tab === 'lab';
    const every = document.hidden ? 2000 : busy ? 140 : (S.rtt > 150 ? 400 : 140);
    await sleep(S.fails ? Math.min(2000, 250 * S.fails) : every);
  }
}
async function pollMap(g) {
  while (g === S.gen) {
    try {
      const m = await get('/api/map?since=' + S.mapVer, { timeout: 9000 });
      if (g === S.gen && m && m.data) buildMap(m);
    } catch (e) { /* state poll reports link problems */ }
    await sleep(1500);
  }
}

// ------------------------------------------------------------------ state -> UI
function onState(st) {
  // connection dot + mode chip
  const link = st.link || {};
  const ok = st.bridge_alive && link.connected && (link.rx_hz || 0) > 15;
  $('#t-dot').className = 'dot ' + (ok ? 'ok' : st.bridge_alive ? 'warn' : 'bad');
  const navs = st.nav || {};
  const chip = $('#t-mode');
  let mode = 'Idle', cls = '';
  if (st.estop) { mode = 'Stopped'; cls = 'estop'; }
  else if (link.esp_mode === 'RC') { mode = 'Remote control'; cls = 'rc'; }
  else if (st.test_running) { mode = 'Running test'; cls = 'auto'; }
  else if (navs.state === 'SCANNING') { mode = 'Scanning'; cls = 'auto'; }
  else if (navs.explore && navs.explore.active) { mode = 'Mapping'; cls = 'auto'; }
  else if (navs.missions && navs.missions.active && navs.missions.active.repeat === -1) { mode = `Patrol, round ${navs.missions.active.round}`; cls = 'auto'; }
  else if (navs.state === 'DRIVING') { mode = navs.goal ? `To ${navs.goal.label}` : 'Driving'; cls = 'auto'; }
  else if (navs.state === 'PLANNING' || (navs.state === 'WAITING' && navs.missions && navs.missions.active)) { mode = 'Planning'; cls = 'auto'; }
  else if (navs.state === 'DWELL') { mode = 'At a stop'; cls = 'auto'; }
  else if (link.cmd_source === 'teleop') { mode = 'Driving'; cls = 'auto'; }
  else if (!st.bridge_alive) { mode = 'No motor link'; }
  chip.textContent = mode; chip.className = 'chip ' + cls;

  $('#estopped').hidden = !st.estop;
  $('#estop').hidden = !!st.estop;

  // banner on the map for things that block driving
  const banner = $('#m-banner');
  let b = '';
  if (!st.bridge_alive) b = '<b>Motor board not talking.</b> Check the ESP32 USB cable, then restart with robot_up.';
  else if (!st.pose) b = '<b>Waiting for the robot’s position.</b> Is the LiDAR spinning?';
  else if (navs && navs.localized === false) b = '<b>Confirm the position.</b> Drive a little until the orange dots sit on the walls, then press Position OK.';
  banner.innerHTML = b; banner.hidden = !b; banner.classList.remove('link');
  $('#d-locok').hidden = !(navs && navs.localized === false);

  // drive readout
  const od = st.odom || {};
  $('#d-v').textContent = fmt(od.v); $('#d-w').textContent = fmt(od.w);

  renderExplore(st);
  if (S.tab === 'go') renderGo(st);
  if (S.tab === 'status') renderStatus(st);
  if (S.tab === 'lab') labOnState(st);
  mapDirty = true;
}
function onLinkLost() {
  if (S.fails === 3) toast('Lost contact with the robot. Retrying…', true);
  if (S.fails >= 3) {
    const banner = $('#m-banner'), busy = S.st && S.st.nav && ((S.st.nav.missions && S.st.nav.missions.active) || (S.st.nav.explore && S.st.nav.explore.active));
    banner.innerHTML = `<b>Link to the robot lost.</b> ${busy ? 'It carries on with its job by itself; ' : ''}reconnecting… The STOP button works again once the link is back.`;
    banner.classList.add('link'); banner.hidden = false;
  }
  $('#t-dot').className = 'dot bad';
  if (S.fails >= 3) { $('#t-mode').textContent = 'Offline'; $('#t-mode').className = 'chip'; }
}

// ------------------------------------------------------------------ tabs & sheet
function setTab(tab) {
  S.tab = tab;
  $$('.tab').forEach(t => { const on = t.dataset.tab === tab; t.classList.toggle('on', on); t.setAttribute('aria-selected', on); });
  ['drive', 'go', 'lab', 'status'].forEach(n => { $('#p-' + n).hidden = n !== tab; });
  const sheet = $('#sheet');
  sheet.classList.toggle('tall', tab === 'lab');
  sheet.classList.toggle('mid', tab === 'go' || tab === 'status');
  document.body.classList.toggle('sheet-tall', tab === 'lab');
  document.body.classList.toggle('sheet-mid', tab === 'go' || tab === 'status');
  sheet.scrollTop = 0;
  if (tab === 'lab') labEnter(); else telemetry(false);
  if (S.st) onState(S.st);
  setTimeout(resize, 260);
}
$$('.tab').forEach(t => t.onclick = () => { haptic(8); setTab(t.dataset.tab); });

// ------------------------------------------------------------------ map
const cv = $('#map'), cx = cv.getContext('2d');
let DPR = 1, mapDirty = true;
const view = { x: 0, y: 0, scale: 45, follow: true };
function resize() {
  DPR = Math.min(window.devicePixelRatio || 1, 2.5);
  const r = cv.getBoundingClientRect();
  cv.width = Math.max(1, Math.round(r.width * DPR)); cv.height = Math.max(1, Math.round(r.height * DPR));
  ['w-chart', 't-chart'].forEach(id => { const c = $('#' + id); const rr = c.getBoundingClientRect(); if (rr.width) { c.width = rr.width * DPR; c.height = rr.height * DPR; } });
  mapDirty = true; drawCharts();
}
window.addEventListener('resize', resize);
const W2S = (x, y) => [cv.width / 2 + (x - view.x) * view.scale * DPR, cv.height / 2 - (y - view.y) * view.scale * DPR];
const S2W = (sx, sy) => [view.x + (sx * DPR - cv.width / 2) / (view.scale * DPR), view.y - (sy * DPR - cv.height / 2) / (view.scale * DPR)];

function buildMap(m) {
  const raw = atob(m.data), w = m.width, h = m.height;
  const off = document.createElement('canvas'); off.width = w; off.height = h;
  const g = off.getContext('2d'), img = g.createImageData(w, h), d = img.data;
  const FREE = [0x34, 0x24, 0x3e], WALL = [0xf3, 0xe9, 0xd8], MID = [0x6f, 0x5a, 0x78];
  for (let y = 0; y < h; y++) {
    const row = (h - 1 - y) * w;            // map row 0 is the bottom (smallest y)
    for (let x = 0; x < w; x++) {
      const v = raw.charCodeAt(y * w + x);
      const o = (row + x) * 4;
      if (v === 0) { d[o + 3] = 0; continue; }
      const occ = v - 1;
      const c = occ >= 65 ? WALL : occ <= 25 ? FREE : MID;
      d[o] = c[0]; d[o + 1] = c[1]; d[o + 2] = c[2]; d[o + 3] = 255;
    }
  }
  g.putImageData(img, 0, 0);
  S.map = m; S.mapVer = m.version; S.mapImg = off;
  if (!S.st || !S.st.pose) { view.x = m.origin[0] + w * m.resolution / 2; view.y = m.origin[1] + h * m.resolution / 2; }
  mapDirty = true;
}

function drawMap() {
  const st = S.st, W = cv.width, H = cv.height, k = view.scale * DPR;
  const pose = st && st.pose;
  if (view.follow && pose) { view.x = pose.x; view.y = pose.y; }
  cx.setTransform(1, 0, 0, 1, 0, 0);
  cx.fillStyle = '#221628'; cx.fillRect(0, 0, W, H);
  if (S.mapImg) {
    const m = S.map, [sx, sy] = W2S(m.origin[0], m.origin[1] + m.height * m.resolution);
    cx.imageSmoothingEnabled = false;
    cx.drawImage(S.mapImg, sx, sy, m.width * m.resolution * k, m.height * m.resolution * k);
  }
  // 1 m grid (scale reference)
  if (k > 18 * DPR) {
    cx.strokeStyle = 'rgba(243,233,216,0.05)'; cx.lineWidth = 1;
    const [x0, y1] = S2W(0, 0), [x1, y0] = S2W(W / DPR, H / DPR);
    cx.beginPath();
    for (let gx = Math.floor(x0); gx <= x1; gx++) { const [px] = W2S(gx, 0); cx.moveTo(px, 0); cx.lineTo(px, H); }
    for (let gy = Math.floor(y0); gy <= y1; gy++) { const [, py] = W2S(0, gy); cx.moveTo(0, py); cx.lineTo(W, py); }
    cx.stroke();
  }
  if (!st) return;
  const navs = st.nav || {};
  // preview route (dashed) and active route
  const line = (pts, color, dash, width) => {
    if (!pts || pts.length < 2) return;
    cx.save(); cx.strokeStyle = color; cx.lineWidth = width * DPR; cx.lineJoin = cx.lineCap = 'round';
    cx.setLineDash(dash.map(v => v * DPR)); cx.beginPath();
    pts.forEach((p, i) => { const [x, y] = W2S(p[0], p[1]); i ? cx.lineTo(x, y) : cx.moveTo(x, y); });
    cx.stroke(); cx.restore();
  };
  if (navs.preview && navs.preview.path && navs.state !== 'DRIVING') line(navs.preview.path, 'rgba(47,183,166,.85)', [7, 6], 3);
  if (navs.path && navs.path.length) line(pose ? [[pose.x, pose.y], ...navs.path] : navs.path, '#2fb7a6', [], 4);
  // exploration: open map edges (frontiers) and the area it is heading to
  const ex = navs.explore;
  if (ex && ex.active) {
    for (const f of ex.frontiers || []) {
      const [x, y] = W2S(f[0], f[1]), r = Math.min(14, 3 + Math.sqrt(f[2]) * 0.9) * DPR;
      cx.fillStyle = 'rgba(229,72,127,.22)'; cx.beginPath(); cx.arc(x, y, r, 0, 7); cx.fill();
      cx.fillStyle = '#e5487f'; cx.beginPath(); cx.arc(x, y, 2.4 * DPR, 0, 7); cx.fill();
    }
    if (ex.target) {
      const [x, y] = W2S(ex.target[0], ex.target[1]);
      cx.save(); cx.strokeStyle = '#e5487f'; cx.lineWidth = 2.5 * DPR; cx.setLineDash([5 * DPR, 4 * DPR]);
      cx.beginPath(); cx.arc(x, y, 13 * DPR, 0, 7); cx.stroke(); cx.restore();
    }
  }
  // places
  const pl = (st.places && st.places.places) || [];
  cx.font = `600 ${12 * DPR}px Instrument, sans-serif`; cx.textAlign = 'center';
  const diamond = (x, y, r, fill) => { cx.beginPath(); cx.moveTo(x, y - r); cx.lineTo(x + r, y); cx.lineTo(x, y + r); cx.lineTo(x - r, y); cx.closePath(); cx.fillStyle = fill; cx.fill(); };
  for (const p of pl) {
    const [x, y] = W2S(p.x, p.y);
    diamond(x, y, 7 * DPR, '#2fb7a6'); diamond(x, y, 3 * DPR, '#221628');
    if (k > 22 * DPR) { cx.fillStyle = 'rgba(243,233,216,.85)'; cx.fillText(p.name, x, y - 12 * DPR); }
  }
  const home = st.places && st.places.home;
  if (home) { const [x, y] = W2S(home.x, home.y); diamond(x, y, 8 * DPR, '#f4b33c'); diamond(x, y, 3.5 * DPR, '#221628'); }
  if (navs.goal) {
    const [x, y] = W2S(navs.goal.x, navs.goal.y);
    cx.strokeStyle = '#2fb7a6'; cx.lineWidth = 2.5 * DPR; cx.beginPath(); cx.arc(x, y, 11 * DPR, 0, 7); cx.stroke();
  }
  if (S.pick) {
    const [x, y] = W2S(S.pick[0], S.pick[1]);
    cx.strokeStyle = '#f4b33c'; cx.lineWidth = 2.5 * DPR; cx.beginPath(); cx.arc(x, y, 10 * DPR, 0, 7); cx.stroke();
  }
  // goal heading arrow (where the robot will face)
  if (navs.goal && fin(navs.goal.yaw)) drawArrow(navs.goal.x, navs.goal.y, navs.goal.yaw, Math.max(0.35, 24 / view.scale), '#2fb7a6', 3);
  drawTools();
  if (!pose) return;
  const c = Math.cos(pose.yaw), s = Math.sin(pose.yaw);
  const R = (bx, by) => W2S(pose.x + c * bx - s * by, pose.y + s * bx + c * by);
  // LiDAR returns
  const pts = (st.scan && st.scan.points) || [];
  cx.fillStyle = '#f4b33c';
  const ps = Math.max(2, 2.2 * DPR);
  for (const p of pts) { const [x, y] = R(p[0], p[1]); cx.fillRect(x - ps / 2, y - ps / 2, ps, ps); }
  // ultrasonic cones
  const us = st.sensors && st.sensors.us, mounts = st.sensors && st.sensors.us_mounts;
  if (us && mounts) {
    us.forEach((r, i) => {
      const m = mounts[i]; if (!m) return;
      const range = fin(r) ? r : 1.2, a = m[2], half = 0.13;
      const [ox, oy] = R(m[0], m[1]);
      const [ax, ay] = R(m[0] + range * Math.cos(a - half), m[1] + range * Math.sin(a - half));
      const [bx, by] = R(m[0] + range * Math.cos(a + half), m[1] + range * Math.sin(a + half));
      cx.fillStyle = !fin(r) ? 'rgba(47,183,166,.08)' : r < 0.4 ? 'rgba(215,57,44,.45)' : r < 0.8 ? 'rgba(244,179,60,.3)' : 'rgba(47,183,166,.15)';
      cx.beginPath(); cx.moveTo(ox, oy); cx.lineTo(ax, ay); cx.lineTo(bx, by); cx.closePath(); cx.fill();
    });
  }
  // robot: drive axle at base_link, chassis behind it
  const body = [[0.07, 0.21], [0.07, -0.21], [-0.37, -0.21], [-0.37, 0.21]].map(p => R(p[0], p[1]));
  cx.beginPath(); body.forEach((p, i) => i ? cx.lineTo(p[0], p[1]) : cx.moveTo(p[0], p[1])); cx.closePath();
  cx.fillStyle = 'rgba(229,72,127,.88)'; cx.fill();
  cx.lineWidth = 2 * DPR; cx.strokeStyle = '#f3e9d8'; cx.stroke();
  const arrow = [[0.17, 0], [0.0, 0.1], [0.03, 0], [0.0, -0.1]].map(p => R(p[0], p[1]));
  cx.beginPath(); arrow.forEach((p, i) => i ? cx.lineTo(p[0], p[1]) : cx.moveTo(p[0], p[1])); cx.closePath();
  cx.fillStyle = '#f3e9d8'; cx.fill();
}
(function frame() { if (mapDirty && !$('#main').hidden) { mapDirty = false; drawMap(); } requestAnimationFrame(frame); })();

// gestures: drag = pan, pinch = zoom, long-press = action sheet
const ptrs = new Map();
let pinch = null, press = null;
function setFollow(on) { view.follow = on; const b = $('#m-follow'); b.classList.toggle('on', on); b.setAttribute('aria-pressed', on); mapDirty = true; }
$('#m-follow').onclick = () => { setFollow(!view.follow); haptic(8); };
cv.addEventListener('pointerdown', e => {
  cv.setPointerCapture(e.pointerId);
  ptrs.set(e.pointerId, { x: e.offsetX, y: e.offsetY });
  if (ptrs.size === 1 && T.tool) {
    T.drag = { sx: e.offsetX, sy: e.offsetY, a: S2W(e.offsetX, e.offsetY), b: S2W(e.offsetX, e.offsetY), px: 0 };
    mapDirty = true;
  } else if (ptrs.size === 1) {
    const sx = e.offsetX, sy = e.offsetY;
    press = { sx, sy, t: setTimeout(() => { press = null; openAction(S2W(sx, sy)); }, 520) };
  } else { if (press) { clearTimeout(press.t); press = null; } T.drag = null; }
  if (ptrs.size === 2) {
    const [a, b] = [...ptrs.values()];
    pinch = { d: Math.hypot(a.x - b.x, a.y - b.y), scale: view.scale };
  }
  $('#m-hint').classList.add('gone');
});
cv.addEventListener('pointermove', e => {
  const p = ptrs.get(e.pointerId); if (!p) return;
  const dx = e.offsetX - p.x, dy = e.offsetY - p.y;
  p.x = e.offsetX; p.y = e.offsetY;
  if (press && Math.hypot(e.offsetX - press.sx, e.offsetY - press.sy) > 9) { clearTimeout(press.t); press = null; }
  if (ptrs.size === 1 && T.drag) {
    T.drag.b = S2W(e.offsetX, e.offsetY);
    T.drag.px = Math.hypot(e.offsetX - T.drag.sx, e.offsetY - T.drag.sy);
    if (T.tool === 'measure') renderMode();
    mapDirty = true;
  } else if (ptrs.size === 1 && !press) {
    if (view.follow) setFollow(false);
    view.x -= dx / view.scale; view.y += dy / view.scale; mapDirty = true;
  } else if (ptrs.size === 2 && pinch) {
    const [a, b] = [...ptrs.values()];
    view.scale = clamp(pinch.scale * Math.hypot(a.x - b.x, a.y - b.y) / Math.max(1, pinch.d), 6, 260); mapDirty = true;
  }
});
const endPtr = e => {
  const wasTool = T.drag && ptrs.size === 1 && e.type === 'pointerup';
  ptrs.delete(e.pointerId); if (ptrs.size < 2) pinch = null; if (press) { clearTimeout(press.t); press = null; }
  if (wasTool) toolRelease(T.drag);
  if (T.tool !== 'measure') T.drag = null;
  mapDirty = true;
};
cv.addEventListener('pointerup', endPtr); cv.addEventListener('pointercancel', endPtr);
cv.addEventListener('wheel', e => { e.preventDefault(); view.scale = clamp(view.scale * (e.deltaY < 0 ? 1.15 : 1 / 1.15), 6, 260); mapDirty = true; }, { passive: false });
cv.addEventListener('contextmenu', e => e.preventDefault());

// ------------------------------------------------------------------ map tools: goal (like RViz 2D Goal Pose), route, measure
const T = { tool: null, drag: null, route: [], loop: false, sent: null };
const DRAG_PX = 14;                                   // shorter drags = tap (no heading)
const headingOf = d => d.px > DRAG_PX ? Math.atan2(d.b[1] - d.a[1], d.b[0] - d.a[0]) : null;
const deg = r => `${Math.round(((r * 57.2958) % 360 + 360) % 360)}°`;
function setTool(tool) {
  T.tool = T.tool === tool ? null : tool;
  $('#m-hint').classList.add('gone');
  T.drag = null;
  if (T.tool !== 'route') { T.route = []; }
  $$('.round.tool').forEach(b => { const on = b.dataset.tool === T.tool; b.classList.toggle('on', on); b.setAttribute('aria-pressed', on); });
  renderMode(); mapDirty = true;
}
$$('.round.tool').forEach(b => b.onclick = () => { haptic(10); setTool(b.dataset.tool); });
function renderMode() {
  const bar = $('#m-mode'), wrap = cv.parentElement;
  let html = '';
  if (T.tool === 'goal') {
    html = T.sent ? `<span class="grow"><b>Goal sent</b> ${T.sent}</span><button class="btn" data-m="cancel">Cancel</button><button class="btn ghost" data-m="close">Done</button>`
      : `<span class="grow"><b>Goal.</b> Tap a spot to send the robot. Drag to choose which way it faces.</span><button class="btn ghost" data-m="close">Done</button>`;
  } else if (T.tool === 'route') {
    const n = T.route.length;
    html = `<span class="grow"><b>${n ? `${n} point${n > 1 ? 's' : ''}` : 'Route.'}</b> ${n ? '' : 'Tap points in order (drag = facing).'}</span>
      ${n ? '<button class="btn ghost" data-m="undo">Undo</button>' : ''}
      <label><input type="checkbox" data-m="loop" ${T.loop ? 'checked' : ''}>Loop</label>
      <button class="btn primary" data-m="go" ${n ? '' : 'disabled'}>Go</button>
      <button class="btn ghost" data-m="close" aria-label="Close">✕</button>`;
  } else if (T.tool === 'measure') {
    const d = T.drag && T.drag.px > 3 ? Math.hypot(T.drag.b[0] - T.drag.a[0], T.drag.b[1] - T.drag.a[1]) : null;
    html = `<span class="grow"><b>${d == null ? 'Measure.' : `${fmt(d, 2)} m`}</b> ${d == null ? 'Drag between two points.' : `heading ${deg(Math.atan2(T.drag.b[1] - T.drag.a[1], T.drag.b[0] - T.drag.a[0]))}`}</span><button class="btn ghost" data-m="close">Done</button>`;
  }
  bar.innerHTML = html; bar.hidden = !html; wrap.classList.toggle('mode-on', !!html);
  $$('[data-m]', bar).forEach(el => {
    const a = el.dataset.m;
    if (a === 'loop') el.onchange = () => { T.loop = el.checked; };
    else el.onclick = () => modeAction(a);
  });
}
async function modeAction(a) {
  haptic(10);
  if (a === 'close') { T.sent = null; setTool(T.tool); return; }
  if (a === 'undo') { T.route.pop(); renderMode(); mapDirty = true; return; }
  if (a === 'cancel') {
    try { await nav({ type: 'cancel' }); toast('Goal canceled'); } catch (e) { toast(e.message, true); }
    T.sent = null; renderMode(); return;
  }
  if (a === 'go') {
    const waypoints = T.route.map((p, i) => ({ x: +p.x.toFixed(3), y: +p.y.toFixed(3), yaw: p.yaw == null ? null : +p.yaw.toFixed(3), label: `point ${i + 1}` }));
    try {
      await nav({ type: 'goto_poses', waypoints, repeat: T.loop ? -1 : 0, replace: true });
      toast(T.loop ? `Patrolling ${waypoints.length} points until you cancel` : `Route of ${waypoints.length} points started`);
      haptic(25); setFollow(true); T.route = []; setTool('route');
    } catch (e) { toast(e.message, true); }
  }
}
async function toolRelease(d) {
  if (T.tool === 'goal') {
    const yaw = headingOf(d), [x, y] = d.a;
    try {
      await nav({ type: 'goto_pose', x, y, yaw, label: 'map goal', replace: true });
      T.sent = yaw == null ? `to ${fmt(x, 1)}, ${fmt(y, 1)}` : `facing ${deg(yaw)}`;
      haptic(25); renderMode();
    } catch (e) { toast(e.message, true); }
  } else if (T.tool === 'route') {
    if (T.route.length >= 30) { toast('30 points at most', true); return; }
    T.route.push({ x: d.a[0], y: d.a[1], yaw: headingOf(d) }); haptic(12); renderMode();
  }
}
$('#m-home').onclick = async () => {
  haptic(15);
  try { await nav({ type: 'go_home', replace: true }); toast('Going home'); setFollow(true); } catch (e) { toast(e.message, true); }
};
function drawArrow(x0, y0, yaw, len, color, width) {
  const k = view.scale * DPR, [ax, ay] = W2S(x0, y0), [bx, by] = W2S(x0 + len * Math.cos(yaw), y0 + len * Math.sin(yaw));
  const ang = Math.atan2(by - ay, bx - ax), h = Math.min(14 * DPR, 0.4 * len * k);
  cx.save(); cx.strokeStyle = cx.fillStyle = color; cx.lineWidth = width * DPR; cx.lineCap = 'round';
  cx.beginPath(); cx.moveTo(ax, ay); cx.lineTo(bx, by); cx.stroke();
  cx.beginPath(); cx.moveTo(bx, by); cx.lineTo(bx - h * Math.cos(ang - 0.45), by - h * Math.sin(ang - 0.45));
  cx.lineTo(bx - h * Math.cos(ang + 0.45), by - h * Math.sin(ang + 0.45)); cx.closePath(); cx.fill(); cx.restore();
}
function drawTools() {
  const k = view.scale * DPR;
  // route points
  if (T.route.length) {
    cx.save(); cx.strokeStyle = 'rgba(244,179,60,.8)'; cx.lineWidth = 2.5 * DPR; cx.setLineDash([6 * DPR, 5 * DPR]);
    cx.beginPath(); T.route.forEach((p, i) => { const [x, y] = W2S(p.x, p.y); i ? cx.lineTo(x, y) : cx.moveTo(x, y); });
    if (T.loop && T.route.length > 2) { const [x, y] = W2S(T.route[0].x, T.route[0].y); cx.lineTo(x, y); }
    cx.stroke(); cx.restore();
    cx.font = `700 ${12 * DPR}px Instrument, sans-serif`; cx.textAlign = 'center'; cx.textBaseline = 'middle';
    T.route.forEach((p, i) => {
      if (p.yaw != null) drawArrow(p.x, p.y, p.yaw, Math.max(0.35, 26 / view.scale), '#f4b33c', 2.5);
      const [x, y] = W2S(p.x, p.y);
      cx.fillStyle = '#f4b33c'; cx.beginPath(); cx.arc(x, y, 11 * DPR, 0, 7); cx.fill();
      cx.fillStyle = '#2a1a10'; cx.fillText(String(i + 1), x, y + 0.5 * DPR);
    });
    cx.textBaseline = 'alphabetic';
  }
  // the drag in progress
  const d = T.drag;
  if (!d) return;
  const [ax, ay] = W2S(d.a[0], d.a[1]);
  if (T.tool === 'measure') {
    if (d.px < 3) return;
    const [bx, by] = W2S(d.b[0], d.b[1]);
    cx.save(); cx.strokeStyle = '#f3e9d8'; cx.lineWidth = 2 * DPR; cx.setLineDash([5 * DPR, 4 * DPR]);
    cx.beginPath(); cx.moveTo(ax, ay); cx.lineTo(bx, by); cx.stroke(); cx.restore();
    [[ax, ay], [bx, by]].forEach(([x, y]) => { cx.fillStyle = '#f3e9d8'; cx.beginPath(); cx.arc(x, y, 4 * DPR, 0, 7); cx.fill(); });
    const m = Math.hypot(d.b[0] - d.a[0], d.b[1] - d.a[1]);
    cx.font = `700 ${13 * DPR}px Instrument, sans-serif`; cx.textAlign = 'center';
    const tx = (ax + bx) / 2, ty = (ay + by) / 2 - 10 * DPR, label = `${m.toFixed(2)} m`, w = cx.measureText(label).width + 12 * DPR;
    cx.fillStyle = 'rgba(34,22,40,.9)'; cx.fillRect(tx - w / 2, ty - 14 * DPR, w, 20 * DPR);
    cx.fillStyle = '#f3e9d8'; cx.fillText(label, tx, ty);
    return;
  }
  const color = T.tool === 'goal' ? '#2fb7a6' : '#f4b33c';
  cx.strokeStyle = color; cx.lineWidth = 2.5 * DPR; cx.beginPath(); cx.arc(ax, ay, 12 * DPR, 0, 7); cx.stroke();
  if (d.px > DRAG_PX) drawArrow(d.a[0], d.a[1], headingOf(d), Math.max(Math.hypot(d.b[0] - d.a[0], d.b[1] - d.a[1]), 0.3), color, 3.5);
}

// ------------------------------------------------------------------ action sheet (map long-press)
function openAction([x, y]) {
  haptic(25);
  S.pick = [x, y]; mapDirty = true;
  $('#a-title').textContent = 'This spot';
  $('#a-coords').textContent = `${fmt(x, 2)} m, ${fmt(y, 2)} m on the map`;
  $('#a-route').textContent = 'Planning a route…';
  $('#action').hidden = false;
  $('#a-go').disabled = false;
  nav({ type: 'preview', x, y, label: 'map point' })
    .then(r => { $('#a-route').textContent = `Route ${fmt(r.length, 1)} m, about ${secs(r.eta_s)}`; })
    .catch(e => { $('#a-route').textContent = e.message; $('#a-go').disabled = true; });
}
function closeAction() { $('#action').hidden = true; S.pick = null; mapDirty = true; }
$('#a-cancel').onclick = closeAction;
$('#action').addEventListener('click', e => { if (e.target.id === 'action') closeAction(); });
$('#a-go').onclick = async () => {
  const [x, y] = S.pick; closeAction();
  try { await nav({ type: 'goto_pose', x, y, label: 'map point' }); toast('On the way'); haptic(20); setFollow(true); }
  catch (e) { toast(e.message, true); }
};
$('#a-save').onclick = async () => {
  const [x, y] = S.pick; closeAction();
  const name = await ask('Name this place', 'e.g. Lab 3 door');
  if (!name) return;
  try { await nav({ type: 'add_place', name, x, y, yaw: 0 }); toast(`Saved ${name}`); }
  catch (e) { toast(e.message, true); }
};
$('#a-home').onclick = async () => {
  const [x, y] = S.pick; closeAction();
  if (!await confirmBox('Set home here?', 'The robot returns to this spot for “go home” and after exploring.', 'Set home')) return;
  try { await nav({ type: 'set_home', x, y, yaw: 0 }); toast('Home set'); } catch (e) { toast(e.message, true); }
};
function ask(title, placeholder) {
  return new Promise(resolve => {
    const wrap = document.createElement('div');
    wrap.className = 'action';
    wrap.innerHTML = `<form class="action-card" role="dialog" aria-modal="true">
      <h3>${esc(title)}</h3><input required maxlength="40" placeholder="${esc(placeholder)}">
      <button class="btn primary wide" type="submit">Save</button>
      <button class="btn ghost wide" type="button">Cancel</button></form>`;
    document.body.appendChild(wrap);
    const input = $('input', wrap), done = v => { wrap.remove(); resolve(v); };
    setTimeout(() => input.focus(), 50);
    $('form', wrap).onsubmit = e => { e.preventDefault(); done(input.value.trim()); };
    $('button[type=button]', wrap).onclick = () => done(null);
  });
}
function confirmBox(title, text, yes) {
  return new Promise(resolve => {
    const wrap = document.createElement('div');
    wrap.className = 'action';
    wrap.innerHTML = `<div class="action-card" role="dialog" aria-modal="true"><h3>${esc(title)}</h3>
      <p class="muted">${esc(text)}</p><button class="btn primary wide" data-v="1">${esc(yes)}</button>
      <button class="btn ghost wide" data-v="0">Cancel</button></div>`;
    document.body.appendChild(wrap);
    $$('button', wrap).forEach(b => b.onclick = () => { wrap.remove(); resolve(b.dataset.v === '1'); });
  });
}

// ------------------------------------------------------------------ drive: joystick
const jc = $('#joy'), jx = jc.getContext('2d');
const joy = { x: 0, y: 0, active: false, id: null };
function drawJoy() {
  const W = jc.width, R = W / 2, r = R * 0.34;
  jx.clearRect(0, 0, W, W);
  jx.beginPath(); jx.arc(R, R, R - 3, 0, 7); jx.fillStyle = '#221628'; jx.fill();
  jx.lineWidth = 2; jx.strokeStyle = '#4a3657'; jx.stroke();
  jx.beginPath(); jx.arc(R, R, (R - 3) * 0.55, 0, 7); jx.strokeStyle = '#3b2946'; jx.stroke();
  jx.strokeStyle = '#3b2946'; jx.beginPath(); jx.moveTo(R, 14); jx.lineTo(R, W - 14); jx.moveTo(14, R); jx.lineTo(W - 14, R); jx.stroke();
  const kx = R + joy.x * (R - r - 4), ky = R - joy.y * (R - r - 4);
  jx.beginPath(); jx.arc(kx, ky, r, 0, 7);
  jx.fillStyle = joy.active ? '#f4b33c' : '#4a3657'; jx.fill();
  jx.lineWidth = 3; jx.strokeStyle = '#f3e9d8'; jx.stroke();
}
function joyFrom(e) {
  const r = jc.getBoundingClientRect(), R = r.width / 2;
  let x = (e.clientX - r.left - R) / (R * 0.66), y = -(e.clientY - r.top - R) / (R * 0.66);
  const m = Math.hypot(x, y); if (m > 1) { x /= m; y /= m; }
  joy.x = Math.abs(x) < 0.06 ? 0 : x; joy.y = Math.abs(y) < 0.06 ? 0 : y; drawJoy();
}
jc.addEventListener('pointerdown', e => { jc.setPointerCapture(e.pointerId); joy.active = true; joy.id = e.pointerId; haptic(10); joyFrom(e); });
jc.addEventListener('pointermove', e => { if (joy.active && e.pointerId === joy.id) joyFrom(e); });
const joyEnd = e => { if (e.pointerId !== joy.id) return; joy.active = false; joy.x = joy.y = 0; drawJoy(); sendCmd(0, 0); sendCmd(0, 0); };
jc.addEventListener('pointerup', joyEnd); jc.addEventListener('pointercancel', joyEnd);
function sendCmd(v, w) { post('/api/cmd', { v, w }, { timeout: 800 }).catch(() => {}); }
setInterval(() => {
  if (!joy.active || !S.st || S.st.estop) return;
  const lim = S.st.limits || { max_v: 0.3, max_w: 1.2 }, k = +$('#d-speed').value / 100;
  sendCmd(joy.y * lim.max_v * k, -joy.x * lim.max_w * k);
}, 100);
$('#d-speed').value = store.get('speed', 60);
$('#d-speed-out').textContent = $('#d-speed').value + '%';
$('#d-speed').oninput = e => { $('#d-speed-out').textContent = e.target.value + '%'; store.set('speed', +e.target.value); };
$('#d-savemap').onclick = async () => {
  const name = await ask('Save the map', 'e.g. cse_floor');
  if (!name) return;
  try { const r = await post('/api/save_map', { name }, { timeout: 15000 }); toast(r.saved && r.saved.includes('WARNING') ? 'Map saved, but not for re-use: see Status' : `Map saved as ${name}`); }
  catch (e) { toast(e.message, true); }
};
$('#d-locok').onclick = async () => { try { await nav({ type: 'confirm_localization' }); toast('Position confirmed'); } catch (e) { toast(e.message, true); } };

// ------------------------------------------------------------------ e-stop
$('#estop').onclick = () => { haptic(60); post('/api/estop', { on: true }).catch(e => toast(e.message, true)); $('#estopped').hidden = false; $('#estop').hidden = true; };
(() => {
  const b = $('#es-release'); let t = null;
  const start = e => { e.preventDefault(); b.classList.add('go'); t = setTimeout(async () => {
    b.classList.remove('go'); haptic(30);
    try { await post('/api/estop', { on: false }); toast('Released. Missions stay paused until you resume them.'); } catch (err) { toast(err.message, true); }
  }, 1000); };
  const stop = () => { clearTimeout(t); b.classList.remove('go'); };
  b.addEventListener('pointerdown', start); b.addEventListener('pointerup', stop); b.addEventListener('pointerleave', stop); b.addEventListener('pointercancel', stop);
})();

// ------------------------------------------------------------------ Go: places, commands, missions
let placeFilter = '';
$('#g-search').oninput = e => { placeFilter = e.target.value.trim().toLowerCase(); if (S.st) renderGo(S.st, true); };
let lastPlacesKey = '';
// ------------------------------------------------------------------ drive: map by itself (exploration)
let xSpeed = +store.get('xspeed', 0.18), xSeenSave = null;
function xSetSpeed(v) {
  xSpeed = v; store.set('xspeed', v);
  $$('[data-xs]').forEach(b => { const on = +b.dataset.xs === v; b.classList.toggle('on', on); b.setAttribute('aria-checked', on); });
}
$$('[data-xs]').forEach(b => b.onclick = () => { haptic(8); xSetSpeed(+b.dataset.xs); });
xSetSpeed(xSpeed);
$('#x-spin').checked = store.get('xspin', true);
$('#x-spin').onchange = e => store.set('xspin', e.target.checked);
$('#x-go').onclick = async () => {
  const ok = await confirmBox('Map this area by itself?',
    'Clear the floor of people’s feet and bags, open the doors you want mapped, and stay close to the STOP button. ' +
    'The robot drives slowly to every unmapped spot, then returns and saves the map.', 'Start mapping');
  if (!ok) return;
  try { await nav({ type: 'explore_start', speed: xSpeed, turn_speed: xSpeed < 0.15 ? 0.35 : 0.45, scan_spin: $('#x-spin').checked }); toast('Mapping started'); haptic(30); setFollow(true); }
  catch (e) { toast(e.message, true); }
};
$('#x-stop').onclick = async () => {
  try { await nav({ type: 'explore_stop' }); toast('Mapping stopped. Save the map if you want to keep it.'); haptic(20); }
  catch (e) { toast(e.message, true); }
};
function renderExplore(st) {
  const ex = (st.nav && st.nav.explore) || {}, on = !!ex.active;
  $('#x-card').classList.toggle('running', on);
  $('#x-idle').hidden = on; $('#x-run').hidden = !on;
  if (on) {
    $('#x-msg').textContent = st.nav.state === 'SCANNING' ? 'Turning once to see every wall…' : (ex.message || 'Exploring');
    $('#x-area').textContent = fin(ex.area) ? fmt(ex.area, 1) : '–';
    $('#x-done').textContent = (ex.goals_done || 0) + (ex.skipped ? ` +${ex.skipped} skipped` : '');
    $('#x-left').textContent = (ex.frontiers || []).length;
    $('#x-time').textContent = secs(ex.elapsed_s);
  }
  const a = st.autosave || {}, last = $('#x-last');
  if (!on && (ex.state === 'done' || ex.state === 'stopped')) {
    last.hidden = false;
    last.innerHTML = ex.state === 'done'
      ? `Last run: <b>${fmt(ex.area, 1)} m²</b> mapped, ${ex.goals_done || 0} areas in ${secs(ex.elapsed_s)}. ` +
        (a.state === 'saved' ? `Saved as <b>${esc(a.name)}</b>.` : a.state === 'saving' ? 'Saving the map…' : a.state === 'failed' ? `Map not saved: ${esc(a.result || '')}` : '')
      : `Stopped: ${esc(ex.message || '')}`;
  } else last.hidden = true;
  // one toast when a save finishes (not for saves that happened before we connected)
  if (xSeenSave === null) xSeenSave = a.name || '';
  else if (a.state === 'saved' && a.name !== xSeenSave) { xSeenSave = a.name; toast(`Mapping done. Map saved as ${a.name}`); haptic(40); }
}

function renderGo(st, force) {
  const navs = st.nav || {}, pose = st.pose;
  // mission card
  const m = navs.missions && navs.missions.active, card = $('#g-mission');
  if (m || navs.hold || (navs.last_failure && navs.state === 'IDLE')) {
    const q = navs.missions ? navs.missions.queue.length : 0;
    const total = (navs.distance || 0);
    card.className = 'mission' + (navs.hold ? ' hold' : '') + (!m && navs.last_failure ? ' fail' : '');
    card.innerHTML = m ? `
      <div class="m-top"><b>${esc(navs.goal ? navs.goal.label : m.stops[m.next_stop] || 'Mission')}</b>
        <span class="muted">${fin(navs.distance) ? `${fmt(total, 1)} m, ${secs(navs.eta_s)}` : esc(navs.state || '')}</span></div>
      <span class="muted">${esc(navs.message || '')}${q ? `. ${q} more queued` : ''}</span>
      <div class="bar"><i style="width:${m.stops.length ? Math.round(100 * m.next_stop / m.stops.length) : 0}%"></i></div>
      <div class="row gap">${navs.hold ? '<button class="btn primary grow" data-a="resume">Resume</button>' : ''}
        <button class="btn grow" data-a="cancel">Cancel mission</button></div>` : `
      <div class="m-top"><b>${navs.hold ? 'Missions paused' : 'Last mission failed'}</b></div>
      <span class="muted">${esc(navs.hold ? 'Press Resume when the area is safe.' : (navs.last_failure.text || navs.last_failure.code || ''))}</span>
      ${navs.hold ? '<div class="row gap"><button class="btn primary grow" data-a="resume">Resume</button></div>' : ''}`;
    card.hidden = false;
    $$('button', card).forEach(b => b.onclick = async () => {
      try { await nav({ type: b.dataset.a }); toast(b.dataset.a === 'resume' ? 'Resumed' : 'Mission canceled'); } catch (e) { toast(e.message, true); }
    });
  } else card.hidden = true;

  // places list (re-render only when something changed)
  const pl = (st.places && st.places.places) || [], home = st.places && st.places.home;
  const dist = p => pose ? Math.hypot(p.x - pose.x, p.y - pose.y) : null;
  const key = JSON.stringify([pl.map(p => p.name), !!home, placeFilter, pose ? [Math.round(pose.x * 2), Math.round(pose.y * 2)] : 0]);
  if (!force && key === lastPlacesKey) return;
  lastPlacesKey = key;
  const match = p => !placeFilter || p.name.toLowerCase().includes(placeFilter) || (p.aliases || []).some(a => a.toLowerCase().includes(placeFilter));
  const rows = [];
  if (home && (!placeFilter || 'home'.includes(placeFilter))) rows.push({ name: 'Home', home: true, d: dist(home) });
  pl.filter(match).map(p => ({ ...p, d: dist(p) })).sort((a, b) => (a.d ?? 1e9) - (b.d ?? 1e9)).forEach(p => rows.push(p));
  $('#g-places').innerHTML = rows.length ? rows.map((p, i) => `
    <li class="${p.home ? 'home' : ''}"><span class="p-name">${esc(p.name)}<span class="p-sub">${fin(p.d) ? `${fmt(p.d, 1)} m away` : ''}${p.aliases && p.aliases.length ? `${fin(p.d) ? ', ' : ''}also “${esc(p.aliases[0])}”` : ''}</span></span>
      <button class="btn small" data-i="${i}" data-a="route">Route</button>
      <button class="btn small primary" data-i="${i}" data-a="go">Go</button></li>`).join('')
    : `<li class="empty">${pl.length ? 'No place matches that search.' : 'No places yet. Drive somewhere and tap “Save place here”, or long‑press the map.'}</li>`;
  $$('#g-places button').forEach(b => b.onclick = async () => {
    const p = rows[+b.dataset.i];
    try {
      if (b.dataset.a === 'go') {
        await nav(p.home ? { type: 'go_home' } : { type: 'goto_place', name: p.name });
        toast(`Going to ${p.name}`); haptic(20); setFollow(true);
      } else {
        const r = await nav(p.home ? { type: 'preview', x: home.x, y: home.y, label: 'home' } : { type: 'preview', name: p.name });
        toast(`${p.name}: ${fmt(r.length, 1)} m, about ${secs(r.eta_s)}`);
      }
    } catch (e) { toast(e.message, true); }
  });
}
$('#g-cmd').onsubmit = async e => {
  e.preventDefault();
  const input = $('#g-cmd-in'), text = input.value.trim(); if (!text) return;
  const out = $('#g-reply'); out.textContent = 'Thinking…';
  try {
    const r = await nav({ type: 'command', text });
    const it = r.interpretation || {};
    const src = it.source === 'llm' ? 'Understood by the language model' : 'Understood offline';
    out.innerHTML = `<span class="src">${src}.</span> ${esc(it.reply || r.status || 'Done.')}`;
    if (it.notes && it.notes.length) out.innerHTML += ` <span class="muted">${esc(it.notes.join(' '))}</span>`;
    input.value = ''; haptic(15);
  } catch (err) { out.textContent = err.message; }
};
$('#g-here').onclick = async () => {
  const name = await ask('Name this place', 'e.g. HOD office');
  if (!name) return;
  try { await nav({ type: 'add_place', name, here: true }); toast(`Saved ${name}`); } catch (e) { toast(e.message, true); }
};
$('#g-home').onclick = async () => {
  if (!await confirmBox('Set home here?', 'The robot returns to this spot for “go home” and “come back”.', 'Set home')) return;
  try { await nav({ type: 'set_home', here: true }); toast('Home set'); } catch (e) { toast(e.message, true); }
};

// ------------------------------------------------------------------ Status
function renderStatus(st) {
  const L = st.link || {}, p = st.pose, n = st.nav || {}, se = st.sensors || {};
  const cls = (good, warn) => good ? 'ok' : warn ? 'warn' : 'bad';
  const items = [
    ['Robot address', BASE.replace('http://', ''), ''],
    ['Link delay', fin(S.rtt) ? `${Math.round(S.rtt)} ms round trip` : '–', cls(S.rtt < 250, S.rtt < 700)],
    ['Motor board', st.bridge_alive && L.connected ? `${fmt(L.rx_hz, 0)} Hz on ${L.port || '?'}` : 'not connected', cls(st.bridge_alive && L.rx_hz > 30, st.bridge_alive)],
    ['Firmware', st.fw ? `v${st.fw}` : 'unknown', cls(st.fw && st.fw >= '2.3', st.fw)],
    ['LiDAR', st.scan ? `${fmt(st.scan.hz, 1)} scans/s` : '–', cls(st.scan && st.scan.hz > 5, st.scan && st.scan.hz > 0)],
    ['Position', p ? `${fmt(p.x)}, ${fmt(p.y)} m, ${fmt(p.yaw * 57.2958, 0)}°` : 'unknown', cls(!!p)],
    ['Heading from', se.yaw_source === 'gyro' ? 'gyro + wheels' : 'wheels only', ''],
    ['Driving mode', { IDLE: 'Idle', RC: 'Remote control', AUTO: 'Pi driving', ESTOP: 'Stopped' }[L.esp_mode] || L.esp_mode || '–', ''],
    ['Commands from', { teleop: 'this app', nav: 'navigator', estop: 'stopped', none: 'nobody' }[L.cmd_source] || '–', ''],
    ['Navigator', n.state ? `${n.state.toLowerCase()}${n.message ? `: ${n.message}` : ''}` : 'not running', cls(!!n.state)],
    ['Map', st.map ? `${fmt(st.map.width * st.map.resolution, 1)} × ${fmt(st.map.height * st.map.resolution, 1)} m` : 'none yet', cls(!!st.map)],
    ['Ultrasonics', se.us ? se.us.map(v => fin(v) ? `${Math.round(v * 100)} cm` : 'clear').join(', ') : 'not fitted', ''],
    ['IMU', se.imu_ok ? 'working' : 'not fitted', ''],
  ];
  $('#s-list').innerHTML = items.map(([k, v, c]) => `<div><dt>${esc(k)}</dt><dd class="${c}">${esc(v)}</dd></div>`).join('');
  const ev = n.events || [];
  $('#s-events').innerHTML = ev.length ? ev.slice(0, 12).map(e => `<li><time>${clock(e.t)}</time><span>${esc(e.text)}</span></li>`).join('')
    : '<li><time></time><span class="muted">Nothing yet.</span></li>';
}
$('#s-disconnect').onclick = disconnect;
$('#t-robot').onclick = () => setTab('status');

// ------------------------------------------------------------------ Tuning Lab
const WHEEL_LABELS = { kp: 'Proportional gain, Kp', ki: 'Integral gain, Ki', pwm_min: 'Start‑up PWM', max_mms: 'Top wheel speed', accel: 'Acceleration ramp' };
const FOLLOW_LABELS = {
  max_linear: 'Cruise speed', max_angular: 'Fastest turn', lookahead: 'Look‑ahead distance', k_angular: 'Steering gain',
  linear_accel: 'Speed‑up rate', angular_accel: 'Turn‑start rate', rotate_in_place_above: 'Turn on the spot above', rotate_exit_below: 'Stop turning on the spot below',
};
const STEP = { kp: 0.01, ki: 0.01, pwm_min: 1, max_mms: 5, accel: 25, max_linear: 0.01, max_angular: 0.05, lookahead: 0.05,
  k_angular: 0.05, linear_accel: 0.05, angular_accel: 0.1, rotate_in_place_above: 0.05, rotate_exit_below: 0.05 };
const lab = { wheelRanges: null, wheel: {}, wheelEdit: 0, wheelDirty: false, follow: {}, followRanges: null, followDirty: false,
  live: false, seq: 0, samples: [], frozen: null, test: 'straight', testResult: null, telemT: null };

$$('.seg-b').forEach(b => b.onclick = () => {
  lab_set(b.dataset.lab);
});
function lab_set(which) {
  S.lab = which;
  $$('.seg-b').forEach(x => { const on = x.dataset.lab === which; x.classList.toggle('on', on); x.setAttribute('aria-selected', on); });
  ['wheels', 'path', 'tests'].forEach(n => { $('#lab-' + n).hidden = n !== which; });
  if (which === 'path') loadFollower();
  if (which !== 'wheels') telemetry(false); else if ($('#w-live').checked) telemetry(true);
  setTimeout(resize, 30);
}
async function labEnter() {
  if (!lab.wheelRanges) {
    try { const p = await get('/api/ping'); lab.wheelRanges = p.wheel_ranges; } catch (e) { /* retry next time */ }
  }
  lab_set(S.lab);
}
function paramRows(container, ranges, labels, values, onChange) {
  container.innerHTML = Object.entries(ranges).map(([k, r]) => `
    <div class="param" data-k="${k}">
      <label class="p-l" for="pn-${k}">${esc(labels[k] || k)}${r.unit ? ` <span class="muted">(${esc(r.unit)})</span>` : ''}</label>
      <input id="pn-${k}" type="number" inputmode="decimal" step="${STEP[k] || 0.01}" min="${r.min}" max="${r.max}" value="${values[k] ?? r.default}">
      <span class="p-h">${esc(r.help)}</span>
      <input type="range" aria-label="${esc(labels[k] || k)}" step="${STEP[k] || 0.01}" min="${r.min}" max="${r.max}" value="${values[k] ?? r.default}">
    </div>`).join('');
  $$('.param', container).forEach(row => {
    const k = row.dataset.k, num = $('input[type=number]', row), rng = $('input[type=range]', row);
    const set = v => { v = clamp(+v, ranges[k].min, ranges[k].max); if (!isFinite(v)) return; num.value = +v.toFixed(4); rng.value = v; row.classList.add('changed'); onChange(k, v); };
    rng.oninput = () => set(rng.value);
    num.onchange = () => set(num.value);
  });
}
function syncRows(container, values) {
  $$('.param', container).forEach(row => {
    const k = row.dataset.k; if (values[k] == null || document.activeElement && row.contains(document.activeElement)) return;
    $('input[type=number]', row).value = +(+values[k]).toFixed(4); $('input[type=range]', row).value = values[k];
  });
}
function markSaved(container) { $$('.param', container).forEach(r => r.classList.remove('changed')); }

// wheels
const pushWheel = debounce(vals => post('/api/esp', { op: 'tune', values: vals }).catch(e => toast(e.message, true)), 200);
let wheelPending = {};
function labOnState(st) {
  if (S.lab !== 'wheels') return;
  const box = $('#w-params');
  if (lab.wheelRanges && !box.children.length) {
    paramRows(box, lab.wheelRanges, WHEEL_LABELS, st.tune || {}, (k, v) => {
      lab.wheelEdit = performance.now(); wheelPending[k] = v; pushWheel(wheelPending); wheelPending = { ...wheelPending };
    });
  }
  if (st.tune && performance.now() - lab.wheelEdit > 1500) { syncRows(box, st.tune); wheelPending = {}; }
  if (!st.tune && !$('#w-fw-note')) {
    box.insertAdjacentHTML('beforebegin', '<p id="w-fw-note" class="lede">The robot has older firmware, so live tuning is off. Flash robot_esp32_classic v2.3 to use this page.</p>');
  } else if (st.tune && $('#w-fw-note')) $('#w-fw-note').remove();
}
$('#w-save').onclick = async () => {
  try { await post('/api/esp', { op: 'save' }); markSaved($('#w-params')); toast('Saved on the robot'); haptic(20); } catch (e) { toast(e.message, true); }
};
$('#w-reset').onclick = async () => {
  if (!await confirmBox('Back to factory values?', 'The speed loop goes back to the values compiled into the firmware.', 'Reset')) return;
  try { await post('/api/esp', { op: 'defaults' }); markSaved($('#w-params')); lab.wheelEdit = 0; toast('Factory values loaded'); } catch (e) { toast(e.message, true); }
};
$('#w-live').onchange = e => telemetry(e.target.checked);
function telemetry(on) {
  if (on === lab.live) return;
  lab.live = on;
  $('#w-live').checked = on;
  post('/api/esp', { op: 'telemetry', on }).catch(() => {});
  clearInterval(lab.telemT);
  if (on) {
    lab.frozen = null;
    get('/api/telemetry?since=999999999').then(r => { lab.seq = r.seq; }).catch(() => {});
    lab.telemT = setInterval(pullTelemetry, 200);
  }
}
async function pullTelemetry() {
  try {
    const r = await get(`/api/telemetry?since=${lab.seq}`, { timeout: 1500 });
    lab.seq = r.seq;
    lab.samples.push(...r.samples);
    const cut = (lab.samples.length ? lab.samples[lab.samples.length - 1][0] : 0) - 8000;
    while (lab.samples.length && lab.samples[0][0] < cut) lab.samples.shift();
    drawWheelChart();
  } catch (e) { /* next tick */ }
}
function chartFrame(c, xr, yr, yUnit) {
  const g = c.getContext('2d'), W = c.width, H = c.height, pad = { l: 40 * DPR, r: 8 * DPR, t: 8 * DPR, b: 20 * DPR };
  g.setTransform(1, 0, 0, 1, 0, 0); g.clearRect(0, 0, W, H);
  const X = x => pad.l + (x - xr[0]) / (xr[1] - xr[0] || 1) * (W - pad.l - pad.r);
  const Y = y => H - pad.b - (y - yr[0]) / (yr[1] - yr[0] || 1) * (H - pad.t - pad.b);
  g.font = `${11 * DPR}px Instrument, sans-serif`; g.fillStyle = '#8d7b96'; g.strokeStyle = 'rgba(243,233,216,.07)'; g.lineWidth = 1;
  const ticks = (a, b, n) => { const s = (b - a) / n, p = Math.pow(10, Math.floor(Math.log10(s))), m = s / p, st = (m < 1.5 ? 1 : m < 3.5 ? 2 : m < 7.5 ? 5 : 10) * p; const out = []; for (let v = Math.ceil(a / st) * st; v <= b + 1e-9; v += st) out.push(+v.toFixed(6)); return out; };
  g.textAlign = 'right';
  for (const v of ticks(yr[0], yr[1], 4)) { const y = Y(v); g.beginPath(); g.moveTo(pad.l, y); g.lineTo(W - pad.r, y); g.stroke(); g.fillText(v + (yUnit || ''), pad.l - 5 * DPR, y + 4 * DPR); }
  g.textAlign = 'center';
  for (const v of ticks(xr[0], xr[1], 5)) g.fillText(v, X(v), H - 5 * DPR);
  const series = (pts, color, dash = [], w = 2) => {
    if (pts.length < 2) return;
    g.save(); g.strokeStyle = color; g.lineWidth = w * DPR; g.setLineDash(dash.map(d => d * DPR)); g.lineJoin = 'round';
    g.beginPath(); pts.forEach(([x, y], i) => i ? g.lineTo(X(x), Y(y)) : g.moveTo(X(x), Y(y))); g.stroke(); g.restore();
  };
  return { g, X, Y, series };
}
function drawWheelChart() {
  const c = $('#w-chart'); if (!c.width || $('#lab-wheels').hidden) return;
  const data = lab.frozen || lab.samples;
  if (!data.length) {
    const g = c.getContext('2d'); g.setTransform(1, 0, 0, 1, 0, 0); g.clearRect(0, 0, c.width, c.height);
    g.fillStyle = '#8d7b96'; g.font = `${13 * DPR}px Instrument, sans-serif`; g.textAlign = 'center';
    g.fillText(lab.live ? 'Waiting for wheel data…' : 'Turn on Live, or run a step test', c.width / 2, c.height / 2);
    return;
  }
  const t0 = data[0][0], t1 = data[data.length - 1][0];
  let lo = 0, hi = 50;
  for (const s of data) { lo = Math.min(lo, s[1], s[2], s[4], s[5]); hi = Math.max(hi, s[1], s[2], s[4], s[5]); }
  const pad = (hi - lo) * 0.1;
  const f = chartFrame(c, [0, Math.max(1, (t1 - t0) / 1000)], [lo - pad, hi + pad], '');
  const pt = i => data.map(s => [(s[0] - t0) / 1000, s[i]]);
  f.series(pt(1), 'rgba(243,233,216,.6)', [5, 4], 1.5);
  f.series(pt(4), 'rgba(243,233,216,.35)', [2, 4], 1.5);
  f.series(pt(2), '#f4b33c'); f.series(pt(5), '#2fb7a6');
}
$('#w-step').onclick = async () => {
  const btn = $('#w-step'), v = +$('#w-step-v').value;
  if (!await confirmBox('Run a step test?', `The robot drives forward at ${v} m/s for 2 seconds (about ${fmt(v * 2, 1)} m). Make room in front.`, 'Drive')) return;
  btn.disabled = true; btn.textContent = 'Driving…';
  try {
    await post('/api/esp', { op: 'telemetry', on: true });
    await sleep(300);
    const before = await get('/api/telemetry?since=999999999');
    const start = before.seq;
    await post('/api/test', { kind: 'straight', v, secs: 2 });
    let st;
    do { await sleep(300); st = await get('/api/test?since=99999'); } while (st.running);
    await sleep(300);
    const r = await get(`/api/telemetry?since=${start}`);
    if (!lab.live) post('/api/esp', { op: 'telemetry', on: false }).catch(() => {});
    if (st.aborted) toast(`Test ended early: ${st.aborted}`, true);
    lab.frozen = r.samples;
    $('#w-live').checked = false; lab.live = false; clearInterval(lab.telemT);
    drawWheelChart();
    showStepMetrics(r.samples, v * 1000);
  } catch (e) { toast(e.message, true); }
  btn.disabled = false; btn.textContent = 'Run step test';
};
function stepMetrics(samples, target, iT, iM) {
  const moving = samples.filter(s => s[iT] > 0.5);
  if (moving.length < 10) return null;
  const tStart = moving[0][0];
  const plateau = moving.filter(s => Math.abs(s[iT] - target) < 1);
  const rise = moving.find(s => s[iM] >= 0.9 * target);
  const peak = Math.max(...moving.map(s => s[iM]));
  const tail = plateau.slice(Math.floor(plateau.length * 0.4));
  const mean = tail.reduce((a, s) => a + s[iM], 0) / Math.max(1, tail.length);
  const sd = Math.sqrt(tail.reduce((a, s) => a + (s[iM] - mean) ** 2, 0) / Math.max(1, tail.length));
  return { rise: rise ? (rise[0] - tStart) / 1000 : null, overshoot: Math.max(0, (peak - target) / target * 100),
    err: mean - target, ripple: sd, mean, rampT: plateau.length ? (plateau[0][0] - tStart) / 1000 : null };
}
function showStepMetrics(samples, target) {
  const L = stepMetrics(samples, target, 1, 2), R = stepMetrics(samples, target, 4, 5), box = $('#w-metrics');
  if (!L || !R) { box.hidden = false; box.innerHTML = '<div class="verdict">Not enough wheel data. Is the firmware v2.3 and did the wheels turn?</div>'; return; }
  const rate = (v, good, warn) => v <= good ? 'good' : v <= warn ? 'warn' : 'bad';
  const ov = Math.max(L.overshoot, R.overshoot), err = Math.max(Math.abs(L.err), Math.abs(R.err)), rip = Math.max(L.ripple, R.ripple);
  const mism = Math.abs(L.mean - R.mean), rise = Math.max(L.rise ?? 9, R.rise ?? 9);
  const tips = [];
  if (ov > 8) tips.push(ov > 15 ? 'Overshoot is high: lower Kp, or soften the acceleration ramp.' : 'Slight overshoot: lower Kp a touch if moves feel jumpy.');
  if (rip > 8) tips.push('Speed wobbles at a steady target: lower Kp, then Ki.');
  if (err > 6) tips.push('Settles off target: raise Ki a little, or adjust Top wheel speed.');
  if (L.rampT != null && rise > L.rampT + 0.5) tips.push('Slow to reach speed after the ramp: raise Kp or the start‑up PWM.');
  if (mism > 8) tips.push('Wheels disagree: check for a dragging wheel, then the start‑up PWM.');
  if (!tips.length) tips.push('This looks well tuned. Save it to the robot.');
  box.innerHTML = `
    <div class="metric ${rate(rise, 0.9, 1.6)}"><b>${fmt(rise, 2)} s</b><span>to reach 90 % speed (ramp alone ${fmt(L.rampT, 2)} s)</span></div>
    <div class="metric ${rate(ov, 8, 15)}"><b>${fmt(ov, 0)} %</b><span>overshoot</span></div>
    <div class="metric ${rate(err, 6, 12)}"><b>${fmt(err, 0)} mm/s</b><span>steady error</span></div>
    <div class="metric ${rate(rip, 8, 15)}"><b>±${fmt(rip, 0)} mm/s</b><span>wobble at speed</span></div>
    <div class="metric wide ${rate(mism, 8, 15)}"><b>${fmt(mism, 0)} mm/s</b><span>left/right difference (left ${fmt(L.mean, 0)}, right ${fmt(R.mean, 0)})</span></div>
    <div class="verdict">${tips.map(esc).join(' ')}</div>`;
  box.hidden = false;
}

// path follower
async function loadFollower() {
  try {
    const r = await nav({ type: 'get_tune' });
    lab.followRanges = r.ranges; lab.follow = r.follower;
    paramRows($('#f-params'), r.ranges, FOLLOW_LABELS, r.follower, (k, v) => { lab.follow[k] = v; pushFollow({ [k]: v }); });
  } catch (e) { $('#f-params').innerHTML = `<p class="lede">${esc(e.message)}</p>`; }
}
const pushFollow = debounce(vals => nav({ type: 'tune', follower: vals }).catch(e => toast(e.message, true)), 200);
$('#f-save').onclick = async () => {
  try { await nav({ type: 'tune', follower: lab.follow, save: true }); markSaved($('#f-params')); toast('Saved. Used from now on, also after restarts.'); haptic(20); }
  catch (e) { toast(e.message, true); }
};
$('#f-reset').onclick = async () => {
  if (!lab.followRanges) return;
  const d = Object.fromEntries(Object.entries(lab.followRanges).map(([k, r]) => [k, r.default]));
  try { const r = await nav({ type: 'tune', follower: d }); lab.follow = r.follower; syncRows($('#f-params'), r.follower); $$('#f-params .param').forEach(x => x.classList.add('changed')); toast('Defaults applied. Save to keep them.'); }
  catch (e) { toast(e.message, true); }
};

// drive tests
const TEST_SPEEDS = { straight: [[0.1, '0.10 m/s'], [0.15, '0.15 m/s'], [0.2, '0.20 m/s'], [0.3, '0.30 m/s']],
  spin: [[0.4, '0.4 rad/s'], [0.6, '0.6 rad/s'], [0.8, '0.8 rad/s'], [1.2, '1.2 rad/s']] };
function setTest(kind) {
  lab.test = kind;
  $$('.tp').forEach(b => b.classList.toggle('on', b.dataset.test === kind));
  $('#t-speed').innerHTML = TEST_SPEEDS[kind].map(([v, l], i) => `<option value="${v}" ${i === 2 ? 'selected' : ''}>${l}</option>`).join('');
  if (kind === 'spin') $('#t-secs').value = '8';
}
$$('.tp').forEach(b => b.onclick = () => setTest(b.dataset.test));
setTest('straight');
$('#t-run').onclick = async () => {
  const kind = lab.test, sp = +$('#t-speed').value, dur = +$('#t-secs').value, btn = $('#t-run');
  const what = kind === 'straight' ? `drives forward about ${fmt(sp * dur, 1)} m` : `turns on the spot about ${fmt(sp * dur / 6.283, 1)} times`;
  if (!await confirmBox('Run the test?', `The robot ${what}. Keep the remote in hand.`, 'Start')) return;
  btn.disabled = true; btn.textContent = 'Running…';
  try {
    await post('/api/test', kind === 'straight' ? { kind, v: sp, secs: dur } : { kind, w: sp, secs: dur });
    let st;
    do { await sleep(250); st = await get('/api/test?since=0'); lab.testResult = st; drawTestChart(); } while (st.running);
    if (st.aborted) toast(`Test ended early: ${st.aborted}`, true);
    showTestMetrics(st);
  } catch (e) { toast(e.message, true); }
  btn.disabled = false; btn.textContent = 'Run test';
};
function unwrap(yaws) { const out = []; let acc = 0; yaws.forEach((y, i) => { if (i) { let d = y - yaws[i - 1]; d -= Math.round(d / 6.283185) * 6.283185; acc += d; } out.push(acc); }); return out; }
function drawTestChart() {
  const c = $('#t-chart'), r = lab.testResult; if (!c.width || !r || !r.samples || r.samples.length < 2) return;
  const s = r.samples.filter(x => fin(x[1]));
  if (s.length < 2) return;
  if (r.kind === 'straight') {
    const [, x0, y0, th0] = s[0], c0 = Math.cos(-th0), s0 = Math.sin(-th0);
    const pts = s.map(q => { const dx = q[1] - x0, dy = q[2] - y0; return [dx * c0 - dy * s0, (dx * s0 + dy * c0) * 100]; });
    const xmax = Math.max(0.5, ...pts.map(p => p[0])), ymax = Math.max(5, ...pts.map(p => Math.abs(p[1]))) * 1.2;
    const f = chartFrame(c, [0, xmax], [-ymax, ymax], ' cm');
    f.series([[0, 0], [xmax, 0]], 'rgba(243,233,216,.4)', [5, 5], 1.5);
    f.series(pts, '#e5487f', [], 3);
  } else {
    const yaw = unwrap(s.map(q => q[3])).map(v => v * 57.2958);
    const odo = s.map(q => fin(q[8]) ? (q[8] - s[0][8]) * 360 : null);
    const tmax = s[s.length - 1][0], all = [...yaw, ...odo.filter(fin)];
    const f = chartFrame(c, [0, tmax], [Math.min(0, ...all), Math.max(10, ...all) * 1.05], '°');
    f.series(s.map((q, i) => [q[0], yaw[i]]), '#e5487f', [], 3);
    if (odo.some(fin)) f.series(s.map((q, i) => [q[0], odo[i]]).filter(p => fin(p[1])), '#f4b33c', [6, 4], 2);
  }
}
function showTestMetrics(r) {
  const box = $('#t-metrics'), s = (r.samples || []).filter(x => fin(x[1]));
  if (s.length < 5) { box.hidden = false; box.innerHTML = '<div class="verdict">No position data came back. Is the map running?</div>'; return; }
  const a = s[0], z = s[s.length - 1];
  if (r.kind === 'straight') {
    const c0 = Math.cos(-a[3]), s0 = Math.sin(-a[3]), dx = z[1] - a[1], dy = z[2] - a[2];
    const fwd = dx * c0 - dy * s0, lat = dx * s0 + dy * c0, head = unwrap([a[3], z[3]])[1] * 57.2958;
    const asked = r.v * r.secs, per = Math.abs(lat) / Math.max(0.2, fwd) * 100;
    const tip = per < 3 ? 'Drives straight. Nothing to fix here.' :
      `Curves to the ${lat > 0 ? 'left' : 'right'} by ${fmt(per, 0)} cm per metre. With the speed loop working, this usually means the two wheels’ ticks per revolution differ: repeat the 1 m push test and set each wheel separately.`;
    box.innerHTML = `
      <div class="metric"><b>${fmt(fwd, 2)} m</b><span>travelled (asked ${fmt(asked, 2)} m)</span></div>
      <div class="metric ${per < 3 ? 'good' : per < 6 ? 'warn' : 'bad'}"><b>${fmt(lat * 100, 1)} cm</b><span>sideways drift</span></div>
      <div class="metric"><b>${fmt(head, 1)}°</b><span>heading change</span></div>
      <div class="metric"><b>${fmt(fwd / Math.max(0.1, z[0] - 1), 2)} m/s</b><span>average speed</span></div>
      <div class="verdict">${esc(tip)}</div>`;
  } else {
    const turned = unwrap(s.map(q => q[3])), map = turned[turned.length - 1] / 6.283185;
    const odo = fin(z[8]) && fin(a[8]) ? z[8] - a[8] : null, sep = (S.st && S.st.sep) || 0.30;
    let tip = 'Firmware or bridge too old to report odometry turns.', sug = null;
    if (fin(odo) && Math.abs(map) > 0.2) {
      sug = sep * odo / map;
      const errp = (odo / map - 1) * 100;
      tip = Math.abs(errp) < 2 ? 'Wheel separation is right.' :
        `Wheel odometry is ${fmt(Math.abs(errp), 1)} % ${errp > 0 ? 'over' : 'under'}-counting turns. Set wheel_separation to ${fmt(sug, 3)} in robot_params.yaml and restart.`;
    }
    box.innerHTML = `
      <div class="metric"><b>${fmt(map, 2)}</b><span>turns, from the LiDAR map</span></div>
      <div class="metric"><b>${fmt(odo, 2)}</b><span>turns, from the wheels</span></div>
      <div class="metric wide ${sug == null ? '' : Math.abs(sug / sep - 1) < 0.02 ? 'good' : 'warn'}"><b>${sug == null ? '–' : fmt(sug, 3) + ' m'}</b><span>suggested wheel_separation (now ${fmt(sep, 3)} m)</span></div>
      <div class="verdict">${esc(tip)}</div>`;
  }
  box.hidden = false;
}
function drawCharts() { drawWheelChart(); drawTestChart(); }

// ------------------------------------------------------------------ boot
$('#c-scan').onclick = scan;
$('#c-manual').onsubmit = async e => {
  e.preventDefault();
  const base = normalizeAddr($('#c-addr').value);
  if (!base) return;
  $('#c-progress').textContent = `Trying ${base.replace('http://', '')}…`;
  const r = await probe(base, 2500);
  if (r) connect(r); else $('#c-progress').textContent = `No robot answered at ${base.replace('http://', '')}. Check the address and the Wi‑Fi.`;
};
drawJoy();
renderFound(store.get('robots', []).map(r => ({ ...r })));
(async () => {
  // served by the robot itself -> connect straight away
  if (/^https?:$/.test(location.protocol)) {
    const r = await probe(location.origin, 2000);
    if (r) return connect(r);
  }
  const last = store.get('last', null);
  if (last) { const r = await probe(last, 1500); if (r) return connect(r); }
  scan();
})();
})();
