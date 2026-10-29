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

// ------------------------------------------------------------------ theme (light / dark / follow the phone)
const PAL = {};
function readPalette() {
  const cs = getComputedStyle(document.documentElement), v = n => cs.getPropertyValue(n).trim();
  Object.assign(PAL, {
    bg: v('--map-bg'), free: v('--map-free'), wall: v('--map-wall'), mid: v('--map-mid'), grid: v('--map-grid'),
    labelBg: v('--map-label-bg'), label: v('--map-label'), sub: v('--map-sub'),
    teal: v('--teal'), orange: v('--orange'), magenta: v('--magenta'), good: v('--good'), dim2: v('--dim-2'),
    joyBg: v('--joy-bg'), knob: v('--joy-knob'), chartGrid: v('--chart-grid'),
  });
}
const rgbOf = hex => { const h = hex.replace('#', ''); return [0, 2, 4].map(i => parseInt(h.slice(i, i + 2), 16)); };
const A = (hex, a) => { const [r, g, b] = rgbOf(hex); return `rgba(${r},${g},${b},${a})`; };
const themeMQ = window.matchMedia ? matchMedia('(prefers-color-scheme: light)') : null;
function applyTheme(pref) {
  pref = pref || store.get('theme', 'system');
  store.set('theme', pref);
  const dark = pref === 'dark' || (pref === 'system' && !(themeMQ && themeMQ.matches));
  document.documentElement.setAttribute('data-theme', dark ? 'dark' : 'light');
  const meta = document.querySelector('meta[name=theme-color]');
  if (meta) meta.setAttribute('content', dark ? '#0d0d10' : '#f4f1eb');
  try { if (native && native.setDarkBars) native.setDarkBars(dark); } catch (e) { /* older APK */ }
  readPalette();
  $$('[data-theme-pick]').forEach(b => { const on = b.dataset.themePick === pref; b.classList.toggle('on', on); b.setAttribute('aria-checked', on); });
  if (S.lastMap) buildMap(S.lastMap);
  if (typeof drawJoy === 'function') drawJoy();
  mapDirty = true;
  if (typeof drawCharts === 'function') drawCharts();
}
if (themeMQ && themeMQ.addEventListener) themeMQ.addEventListener('change', () => { if (store.get('theme', 'system') === 'system') applyTheme('system'); });

// ------------------------------------------------------------------ API
let BASE = '';
const DEMO = 'demo:';
// login session per robot: {token, user}. 'session' mirrors the current one (the classic page reads it).
const sessions = {
  get(base) { return (store.get('sessions', {}) || {})[base] || null; },
  set(base, v) {
    const all = store.get('sessions', {}) || {};
    if (v) all[base] = v; else delete all[base];
    store.set('sessions', all);
    store.set('session', v ? { base, ...v } : null);
  },
};
let TOKEN = '';
function httpError(msg, code) { const e = new Error(msg); e.code = code; return e; }
async function req(path, { method = 'GET', body, timeout = 4000 } = {}) {
  if (BASE === DEMO) {                       // built-in simulated robot (demo.js): no network at all
    await sleep(15);
    try {
      return window.PathikDemo.handle(path, method, body === undefined ? undefined : JSON.parse(JSON.stringify(body)), TOKEN);
    } catch (e) { if (e.code === 401 && TOKEN) onLoggedOut('Please log in again.'); throw e; }
  }
  const ctl = new AbortController();
  const t = setTimeout(() => ctl.abort(), timeout);
  try {
    // text/plain avoids a CORS preflight for the body; the token header needs one (cached 2 h by the robot)
    const headers = {};
    if (body !== undefined) headers['Content-Type'] = 'text/plain';
    if (TOKEN) headers.Authorization = 'Bearer ' + TOKEN;
    const r = await fetch(BASE + path, {
      method, signal: ctl.signal, cache: 'no-store',
      body: body === undefined ? undefined : JSON.stringify(body), headers,
    });
    const j = await r.json().catch(() => ({}));
    if (r.status === 401 && j.login && TOKEN) onLoggedOut('Your session ended. Please log in again.');
    if (!r.ok) throw httpError(j.error || `robot answered ${r.status}`, r.status);
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
    const t0 = performance.now();
    const r = await fetch(base + '/api/ping', { signal: ctl.signal, cache: 'no-store' });
    const j = await r.json();
    if (j && j.api) return { base, ...j, ms: Math.round(performance.now() - t0) };
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
    renderFound([...found.values()], true);
    haptic(20);
  };
  $('#c-card-name').textContent = 'Looking…';
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
  btn.disabled = false;
  scanning = false;
  renderFound(found.size ? [...found.values()] : store.get('robots', []), found.size > 0);
  if (!found.size) btn.textContent = 'Find again';
}
// How this phone reaches the robot. The robot says it best (st.net.via: what IT sees); before the first
// answer we guess from the address.
const VIA_TXT = { public: 'Public link (Funnel)', tailscale: 'Internet (Tailscale)', lan: 'Same Wi‑Fi', local: 'This device' };
function netType(base, net) {
  if (base === DEMO) return 'Demo';
  if (net && VIA_TXT[net.via]) {
    if (net.via !== 'lan') return VIA_TXT[net.via];
  }
  if (/^https:\/\//.test(base || '') && /\.ts\.net/.test(base)) return VIA_TXT.public;
  const h = (base || '').replace(/^https?:\/\//, '').replace(/:\d+$/, '');
  if (h === '10.42.0.1') return 'Robot Wi‑Fi';
  if (h.startsWith('169.254.')) return 'Cable';
  if (/^100\.(6[4-9]|[7-9]\d|1[01]\d|12[0-7])\./.test(h) || h.endsWith('.ts.net') || (!/\./.test(h) && h !== 'localhost')) return 'Internet (Tailscale)';
  if (h === 'localhost' || h.startsWith('127.')) return 'This device';
  return 'Same Wi‑Fi';
}
let foundRobots = [];
function renderFound(robots, live = false) {
  foundRobots = robots;
  $('#c-list').innerHTML = robots.length ? robots.map((r, i) => `
    <li><button data-i="${i}">
      <svg viewBox="0 0 64 64"><use href="#mark"/></svg>
      <span><b>${esc(r.robot || 'robot')}</b><small>${esc(shortAddr(r.base))} · ${esc(netType(r.base))}</small></span>
      <span class="go">${live ? 'Live' : 'Saved'}</span>
    </button></li>`).join('') : '<li class="empty">No saved robots yet.</li>';
  $$('#c-list button').forEach(b => b.onclick = () => connect(robots[+b.dataset.i]));
  const top = live && robots[0];
  $('#c-card-name').textContent = top ? (top.robot || 'robot') : (scanning ? 'Looking…' : 'No robot found yet');
  $('#c-card-net').textContent = top ? netType(top.base) : '–';
  $('#c-card-lat').textContent = top && fin(top.ms) ? `${top.ms} ms` : '–';
  $('#c-card-fw').textContent = top && top.fw ? `Firmware ${top.fw}` : '–';
  const btn = $('#c-scan');
  if (top && !scanning) { btn.textContent = `Connect to ${top.robot || 'robot'}`; btn.onclick = () => connect(top); }
  else if (!scanning) { btn.textContent = 'Find robot'; btn.onclick = scan; }
}
// What people type -> a base URL:
//   10.42.0.1 / slam-robot / 100.101.1.2 / slam-robot.tail1234.ts.net  -> http://<that>:8080 (Tailscale or LAN)
//   https://slam-robot.tail1234.ts.net                                -> as typed (Funnel: https on port 443)
//   http://host:9000                                                   -> as typed
function normalizeAddr(s) {
  s = (s || '').trim().replace(/\s+/g, '');
  if (!s) return '';
  s = s.replace(/\/(app\/?)?$/, '').replace(/\/+$/, '');     // a pasted ".../app/" link works too
  if (/^https:\/\//i.test(s)) return s.replace(/^https/i, 'https');
  if (!/^http:\/\//i.test(s)) s = 'http://' + s;
  if (!/:\d+$/.test(s.replace(/^http:\/\//i, ''))) s += ':8080';
  return s.replace(/^http/i, 'http');
}
const shortAddr = base => (base || '').replace(/^https?:\/\//, '');

// ------------------------------------------------------------------ app state
const S = {
  st: null, gen: 0, fails: 0, lastOk: 0, robot: null,
  map: null, mapVer: -1, mapImg: null,
  tab: 'home', lab: null, user: null, lastMap: null,
};
const ROLE_RANK = { staff: 0, operator: 1, admin: 2 };
const can = need => !!S.user && (ROLE_RANK[S.user.role] ?? -1) >= ROLE_RANK[need];
const initials = n => String(n || '?').trim().split(/\s+/).map(w => w[0]).slice(0, 2).join('').toUpperCase() || '?';

async function connect(robot) {
  BASE = robot.base;
  S.robot = robot;
  if (robot.base !== DEMO) {                 // the demo is never saved as a robot
    const saved = store.get('robots', []).filter(r => r.base !== robot.base);
    store.set('robots', [{ base: robot.base, robot: robot.robot }, ...saved].slice(0, 5));
    store.set('last', robot.base);
  }
  $('#t-host').textContent = robot.base === DEMO ? 'Pathik demo' : robot.host && robot.host !== 'simulator' ? robot.host : (robot.robot || 'robot');
  if (!robot.auth) {                         // older robot software: no accounts, everyone can do everything
    TOKEN = ''; return enterMain({ username: '', name: 'Operator', role: 'admin', legacy: true });
  }
  const saved = sessions.get(robot.base);
  if (saved && saved.token) {
    TOKEN = saved.token;
    try { const me = await get('/api/auth/me', { timeout: 3000 }); sessions.set(robot.base, { token: TOKEN, user: me.user }); return enterMain(me.user, me.pending); }
    catch (e) { if (e.code !== 401) { /* robot busy/unreachable: still try with the saved session */ return enterMain(saved.user); } }
  }
  TOKEN = '';
  showAuth(robot.setup_needed ? 'setup' : 'login');
}
function enterMain(user, pending = 0) {
  S.user = user;
  document.body.classList.remove('role-staff', 'role-operator', 'role-admin');
  document.body.classList.add('role-' + user.role);
  const staff = user.role === 'staff';
  // staff: Home · Send · Map · Me; operator/admin: Home · Send · Map · Drive · Operate (the robot-team area)
  $('#tab-more-t').textContent = staff ? 'Me' : 'Operate';
  $('#main').style.setProperty('--ntabs', staff ? 4 : 5);      // + the STOP slot in the middle of the tab bar
  S.tab = 'home'; S.lab = null;
  renderMe(pending);
  $('#connect').hidden = true; $('#auth').hidden = true; $('#main').hidden = false;
  if (BASE === DEMO) toast(`Demo robot as ${user.role}. Try Send → “take me to the 3D printer”.`);
  else if (!user.legacy) toast(`Welcome, ${user.name || user.username}`);
  S.gen++; S.fails = 0; S.mapVer = -1; S.mapImg = null; S.st = null;
  const g = S.gen;
  pollState(g); pollMap(g); pollCam(g);
  setCam(CAM.open);
  resize();
  $('#m-hint').classList.remove('gone');
  setTimeout(() => $('#m-hint').classList.add('gone'), 7000);
  if (native && native.keepAwake) native.keepAwake(true);
  setTab(S.tab);
}
function disconnect() {
  S.gen++;
  if (window.PathikDemo) window.PathikDemo.running = false;
  telemetry(false);
  renderFound(store.get('robots', []));
  $('#c-progress').textContent = '';
  $('#main').hidden = true; $('#auth').hidden = true; $('#connect').hidden = false;
  if (native && native.keepAwake) native.keepAwake(false);
}
function onLoggedOut(msg) {
  if (!S.user && !TOKEN) return;
  sessions.set(BASE, null); TOKEN = ''; S.user = null; S.gen++;
  telemetry(false);
  showAuth('login', msg);
}

// ------------------------------------------------------------------ login / request access / first admin
const auth = { mode: 'login' };
function showAuth(mode, msg) {
  auth.mode = mode;
  const r = S.robot || {};
  $('#au-robot').textContent = BASE === DEMO ? 'Pathik demo' : (r.robot || r.host || 'robot');
  $('#au-net').textContent = netType(BASE);
  const T = {
    login: ['Welcome back', 'Log in to use Pathik.', 'Log in', 'New here?', 'Request access'],
    signup: ['Request access', 'Pick a username and password. An admin approves you, then you can log in.', 'Send request', 'Have an account?', 'Log in'],
    setup: ['Set up this robot', 'Create the first account. It becomes the admin.', 'Create admin account', '', ''],
  }[mode === 'pending' ? 'login' : mode];
  $('#au-title').textContent = T[0]; $('#au-sub').textContent = T[1]; $('#au-go').textContent = T[2];
  $('#au-switch-t').textContent = T[3]; $('#au-switch-b').textContent = T[4];
  $('#au-switch').hidden = mode === 'setup' || mode === 'pending';
  $('#au-setup').hidden = mode !== 'setup';
  $('#au-name-l').hidden = mode === 'login';
  $('#au-pass').autocomplete = mode === 'login' ? 'current-password' : 'new-password';
  $('#au-form').hidden = mode === 'pending'; $('#au-pending').hidden = mode !== 'pending';
  $('#au-msg').textContent = msg || ''; $('#au-msg').className = 'a-msg' + (msg ? ' err' : '');
  const demo = BASE === DEMO;
  $('#au-demo').hidden = !demo || mode !== 'login';
  if (demo) {
    $('#au-demo-row').innerHTML = window.PathikDemo.accounts.map(([u, p, label]) =>
      `<button type="button" data-u="${u}" data-p="${p}"><b>${label}</b><span>${u} / ${p}</span></button>`).join('');
    $$('#au-demo-row button').forEach(b => b.onclick = () => { $('#au-user').value = b.dataset.u; $('#au-pass').value = b.dataset.p; doAuth(); });
  }
  $('#connect').hidden = true; $('#main').hidden = true; $('#auth').hidden = false;
  setTimeout(() => { const f = mode === 'login' ? $('#au-user') : $('#au-name'); if (f && !f.value) f.focus(); }, 80);
}
async function doAuth() {
  const msg = $('#au-msg'), btn = $('#au-go');
  const username = $('#au-user').value.trim().toLowerCase(), password = $('#au-pass').value, name = $('#au-name').value.trim();
  if (!username || !password) { msg.textContent = 'Enter a username and password.'; msg.className = 'a-msg err'; return; }
  btn.disabled = true; msg.className = 'a-msg'; msg.textContent = auth.mode === 'login' ? 'Logging in…' : 'Creating the account…';
  try {
    const r = await post(auth.mode === 'login' ? '/api/auth/login' : '/api/auth/signup', { username, password, name }, { timeout: 8000 });
    if (r.token) {
      TOKEN = r.token; sessions.set(BASE, { token: r.token, user: r.user });
      $('#au-pass').value = ''; haptic(25);
      enterMain(r.user);
    } else { showAuth('pending'); }
  } catch (e) { msg.textContent = e.message; msg.className = 'a-msg err'; haptic(40); }
  btn.disabled = false;
}
$('#au-form').onsubmit = e => { e.preventDefault(); doAuth(); };
$('#au-switch-b').onclick = () => showAuth(auth.mode === 'login' ? 'signup' : 'login');
$('#au-pending-ok').onclick = () => showAuth('login');
$('#au-back').onclick = disconnect;
$('#au-show').onclick = () => { const i = $('#au-pass'), sh = i.type === 'password'; i.type = sh ? 'text' : 'password'; $('#au-show').textContent = sh ? 'Hide' : 'Show'; };

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
    const busy = joy.active || (S.tab === 'more' && !!S.lab);
    const every = document.hidden ? 2000 : busy ? 140 : (S.rtt > 150 ? 400 : 140);
    await sleep(S.fails ? Math.min(2000, 250 * S.fails) : every);
  }
}
// ------------------------------------------------------------------ camera (vision node on the robot)
const CAM = { open: !!store.get('cam', false), big: false, url: null };
const VIS_TXT = { STOP: 'Stopped: person', SLOW_DOWN: 'Slowing down', CONTINUE: 'Path clear', UNKNOWN: 'Camera only' };
const VIS_CLS = { STOP: 'stop', SLOW_DOWN: 'slow', CONTINUE: 'go' };
function setCam(open) {
  CAM.open = open; store.set('cam', open);
  $('#cam').hidden = !open; $('#t-cam').setAttribute('aria-pressed', open);
  if (S.st) renderVision(S.st);
}
$('#t-cam').onclick = () => { haptic(8); setCam(!CAM.open); };
$('#cam-x').onclick = () => setCam(false);
$('#cam-size').onclick = () => { CAM.big = !CAM.big; $('#cam').classList.toggle('big', CAM.big); };
async function camFrame() {
  if (BASE === DEMO) { drawDemoCam(S.st && S.st.vision); return true; }
  const ctl = new AbortController();
  const t = setTimeout(() => ctl.abort(), 3000);
  try {
    const r = await fetch(BASE + '/api/camera.jpg', { signal: ctl.signal, cache: 'no-store',
      headers: TOKEN ? { Authorization: 'Bearer ' + TOKEN } : {} });
    if (!r.ok) return false;
    const url = URL.createObjectURL(await r.blob());
    const img = $('#cam-img'), old = CAM.url;
    CAM.url = url;
    img.onload = () => { if (old) URL.revokeObjectURL(old); };
    img.src = url;
    return true;
  } catch (e) { return false; } finally { clearTimeout(t); }
}
async function pollCam(g) {
  while (g === S.gen) {
    if (CAM.open && !document.hidden) {
      const ok = await camFrame();
      $('#cam-img').hidden = !ok || BASE === DEMO;
      $('#cam-demo').hidden = BASE !== DEMO;
      $('#cam-empty').hidden = ok;
    }
    // ~5 pictures a second on Wi-Fi, ~2 over mobile data; nothing while closed
    await sleep(!CAM.open || document.hidden ? 500 : S.rtt > 150 ? 450 : 200);
  }
}
function renderVision(st) {
  const v = st.vision, navs = st.nav || {}, note = navs.vision || '';
  $('#t-cam').hidden = !v && !st.camera && BASE !== DEMO;
  const act = v ? v.action : 'UNKNOWN', cls = VIS_CLS[act] || 'off';
  $('#t-cam-dot').className = cls;
  // on the map: say it when the camera is holding the robot back (the card says it when open)
  const mv = $('#m-vis');
  mv.hidden = !note || CAM.open;
  if (note) mv.innerHTML = `<b>${/waiting/.test(note) ? 'Waiting for a person.' : 'Slowing down.'}</b> ${esc(note.replace(/^camera: /, ''))}`;
  if (!CAM.open) return;
  const a = $('#cam-act');
  a.textContent = v ? (VIS_TXT[act] || act) : 'Camera off';
  a.className = 'cam-act ' + cls;
  $('#cam-fps').textContent = v && v.fps ? `${v.fps}/s${v.lidar ? ' · LiDAR' : ''}` : '';
  let why;
  if (!v) why = 'The camera program is not running on the robot (vision:=false or not installed).';
  else if (!v.camera_ok) why = v.camera_error || 'No picture from the camera.';
  else if (!v.detector) why = v.detector_error || 'People detection is off.';
  else why = (v.reason || '') + (act === 'SLOW_DOWN' ? ` (speed ×${v.speed_factor})` : '') +
             (note ? '. The robot is obeying this.' : '');
  $('#cam-why').textContent = why;
  if (!v || !v.camera_ok) $('#cam-empty').textContent = why;
  $('#cam-people').innerHTML = ((v && v.people) || []).map(p => {
    const b = Math.round(p.bearing_deg), side = Math.abs(b) <= 15 ? 'ahead' : `${Math.abs(b)}° ${b < 0 ? 'left' : 'right'}`;
    return `<li><b>Person ${p.id}</b> ${side}${p.distance_m != null ? ` · ${(+p.distance_m).toFixed(1)} m` : ''}` +
           `${p.approaching ? ' · <em>coming closer</em>' : p.moving_away ? ' · moving away' : ''}</li>`;
  }).join('');
}
function drawDemoCam(v) {                       // demo: a drawn corridor with the simulated people
  const c = $('#cam-demo'), g = c.getContext('2d'), w = c.width, h = c.height;
  g.fillStyle = '#c9c2b6'; g.fillRect(0, 0, w, h);
  g.fillStyle = '#9b948a'; g.beginPath(); g.moveTo(0, h); g.lineTo(w * 0.42, h * 0.45); g.lineTo(w * 0.58, h * 0.45); g.lineTo(w, h); g.fill();
  g.fillStyle = '#e7e1d6'; g.fillRect(w * 0.42, h * 0.18, w * 0.16, h * 0.27);
  g.strokeStyle = 'rgba(255,255,255,.7)'; g.setLineDash([4, 4]);
  [-15, 15].forEach(a => { g.beginPath(); g.moveTo(w / 2 + a / 102 * w, h); g.lineTo(w / 2 + a / 102 * w, h * 0.45); g.stroke(); });
  g.setLineDash([]);
  ((v && v.people) || []).forEach(p => {
    const d = p.distance_m || 3, bh = Math.min(h * 0.95, h * 0.55 / d), bw = bh * 0.38;
    const x = w / 2 + p.bearing_deg / 102 * w - bw / 2, y = h * 0.45 + h * 0.25 / d - bh * 0.55;
    g.fillStyle = '#3d4a5c'; g.fillRect(x + bw * 0.2, y + bh * 0.2, bw * 0.6, bh * 0.8);
    g.beginPath(); g.arc(x + bw / 2, y + bh * 0.11, bw * 0.22, 0, 7); g.fill();
    g.strokeStyle = Math.abs(p.bearing_deg) <= 15 ? (v.action === 'STOP' ? '#e8282a' : '#ffaa00') : '#00c8ff';
    g.lineWidth = 2; g.strokeRect(x, y, bw, bh);
    g.fillStyle = g.strokeStyle; g.font = '11px sans-serif';
    g.fillText(`${p.bearing_deg > 0 ? '+' : ''}${Math.round(p.bearing_deg)}° ${d.toFixed(1)}m`, x, Math.max(11, y - 3));
  });
  g.fillStyle = 'rgba(0,0,0,.75)'; g.fillRect(0, 0, w, 18);
  g.fillStyle = { STOP: '#ff5a4f', SLOW_DOWN: '#ffb547', CONTINUE: '#47e3a4' }[v && v.action] || '#bbb';
  g.font = 'bold 11px sans-serif'; g.fillText(`DEMO  ${v ? v.action : ''}  ${v ? v.reason : ''}`.slice(0, 60), 6, 13);
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
  else if (navs.state === 'CONFIRM') { mode = 'Waiting for OK'; cls = 'wait'; }
  else if (navs.state === 'RECOVERING') { mode = navs.recovery && navs.recovery.status === 'asking' ? 'Needs help · see Home' : 'Recovering'; cls = 'wait'; }
  else if (navs.state === 'DOCKING') { mode = 'Docking'; cls = 'auto'; }
  else if (navs.state === 'UNDOCKING') { mode = 'Leaving dock'; cls = 'auto'; }
  else if (navs.state === 'CHARGING') { mode = navs.battery && navs.battery.charging ? 'Charging' : 'On the dock'; cls = ''; }
  else if (navs.state === 'SCANNING') { mode = 'Scanning'; cls = 'auto'; }
  else if (navs.explore && navs.explore.active) { mode = 'Mapping'; cls = 'auto'; }
  else if (navs.missions && navs.missions.active && navs.missions.active.repeat === -1) { mode = `Patrol, round ${navs.missions.active.round}`; cls = 'auto'; }
  else if (navs.state === 'DRIVING' && navs.blocked) { mode = 'Blocked · why?'; cls = 'wait'; }
  else if (navs.state === 'DRIVING') { mode = navs.goal ? `To ${navs.goal.label}` : 'Driving'; cls = 'auto'; }
  else if (navs.state === 'PLANNING' || (navs.state === 'WAITING' && navs.missions && navs.missions.active)) { mode = 'Planning'; cls = 'auto'; }
  else if (navs.state === 'DWELL') { mode = 'At a stop'; cls = 'auto'; }
  else if (link.cmd_source === 'teleop') { mode = 'Driving'; cls = 'auto'; }
  else if (!st.bridge_alive) { mode = 'No motor link'; }
  if (mode === 'Idle' && navs.last_failure && navs.state === 'IDLE' && /failed/.test(navs.message || '')) { mode = 'Failed · why?'; cls = 'wait'; }
  chip.textContent = mode; chip.className = 'chip glass ' + cls;
  renderBattery(navs.battery);
  renderConfirm(navs.confirm);
  $('#t-link').textContent = `${netType(BASE, st.net)}${fin(S.rtt) ? ` · ${Math.round(S.rtt)} ms` : ''}`;
  renderPublic(st.net);

  $('#estopped').hidden = !st.estop;
  $('#estop').hidden = !!st.estop;

  // banner on the map for things that block driving
  const banner = $('#m-banner');
  let b = '';
  if (!st.bridge_alive) b = '<b>Motor board not talking.</b> Check the ESP32 power and its 3 wires to the Pi (pins 6, 8, 10).';
  else if (!st.pose) b = '<b>Waiting for the robot’s position.</b> Is the LiDAR spinning?';
  else if (navs && navs.localized === false) b = '<b>Confirm the position.</b> Drive a little until the orange dots sit on the walls, then Map → Position OK. Wrong place? Use the Locate tool.';
  banner.innerHTML = b; banner.hidden = !b; banner.classList.remove('link');
  $('#d-locok').hidden = !(navs && navs.localized === false);

  // drive readout
  const od = st.odom || {};
  $('#d-v').textContent = fmt(od.v); $('#d-w').textContent = fmt(od.w);

  renderWhere(st);
  renderVision(st);
  renderExplore(st);
  renderMapStats(st);
  if (S.tab === 'home' || S.tab === 'send') renderGo(st);
  renderHome(st);
  if (S.tab === 'more' && S.lab) { if (S.lab === 'health') renderStatus(st); labOnState(st); }
  mapDirty = true;
}
function renderBattery(b) {
  const el = $('#t-bat');
  if (!b || b.level === 'unknown' || !fin(b.pct)) { el.hidden = true; return; }
  el.hidden = false;
  el.className = 'chip glass bat ' + (b.charging ? 'charging' : b.level);
  el.innerHTML = `<i><b style="width:${clamp(b.pct, 4, 100) * 0.16}px"></b></i>${b.charging ? '⚡' : ''}${b.pct}%`;
  el.title = `${fmt(b.v, 2)} V${fin(b.a) ? `, ${fmt(b.a, 2)} A` : ''}`;
}
let confirmShown = null;
function renderConfirm(c) {
  const box = $('#m-confirm');
  if (!c) { box.hidden = true; confirmShown = null; return; }
  if (confirmShown !== c.label) {
    confirmShown = c.label; haptic(60);
    box.innerHTML = `<b>Pathik is at ${esc(c.label)}</b><span class="muted" id="m-confirm-t"></span>
      <button class="btn grad wide" id="m-confirm-ok">OK, carry on</button>`;
    $('#m-confirm-ok').onclick = async () => {
      haptic(25);
      try { await nav({ type: 'confirm' }); box.hidden = true; toast('Thanks! Carrying on.'); } catch (e) { toast(e.message, true); }
    };
  }
  $('#m-confirm-t').textContent = `Waiting for someone to press OK (continues by itself in ${secs(c.left_s)}).`;
  box.hidden = false;
}
let whyT = 0;
async function explainNow(speak) {
  try {
    const r = await nav({ type: 'explain' });
    const box = $('#m-why');
    box.innerHTML = `<p>${esc(r.answer)}</p>${r.details && r.details.length ? `<ol>${r.details.slice(0, 4).map(d => `<li>${esc(d)}</li>`).join('')}</ol>` : ''}
      <div class="row"><button class="btn ghost small" id="m-why-x">Close</button></div>`;
    box.hidden = false;
    $('#m-why-x').onclick = () => { box.hidden = true; };
    clearTimeout(whyT); whyT = setTimeout(() => { box.hidden = true; }, 15000);
    if (speak) say(r.answer);
    return r.answer;
  } catch (e) { toast(e.message, true); return ''; }
}
$('#t-mode').onclick = () => { haptic(10); explainNow(true); };
function onLinkLost() {
  if (S.fails === 3) toast('Lost contact with the robot. Retrying…', true);
  if (S.fails >= 3) {
    const banner = $('#m-banner'), busy = S.st && S.st.nav && ((S.st.nav.missions && S.st.nav.missions.active) || (S.st.nav.explore && S.st.nav.explore.active));
    banner.innerHTML = `<b>Link to the robot lost.</b> ${busy ? 'It carries on with its job by itself; ' : ''}reconnecting… The STOP button works again once the link is back.`;
    banner.classList.add('link'); banner.hidden = false;
  }
  $('#t-dot').className = 'dot bad';
  if (S.fails >= 3) { $('#t-mode').textContent = 'Offline'; $('#t-mode').className = 'chip glass'; $('#t-link').textContent = 'reconnecting…'; if (S.st) renderHome(S.st); }
}

// ------------------------------------------------------------------ tabs, pages & sheet
// Home / Send / Operate (Me for staff) are pages over the map; Map shows the map with a sheet; Drive the joystick.
const TABS = ['home', 'send', 'map', 'drive', 'more'];
function setTab(tab) {
  if (!TABS.includes(tab)) tab = 'home';
  if (!can('operator') && tab === 'drive') tab = 'map';
  const changed = S.tab !== tab;
  S.tab = tab;
  $$('.tab').forEach(t => { const on = t.dataset.tab === tab; t.classList.toggle('on', on); t.setAttribute('aria-selected', on); });
  ['home', 'send', 'more'].forEach(n => { $('#p-' + n).hidden = n !== tab; });
  TABS.forEach(n => document.body.classList.toggle('tab-' + n, n === tab));
  if (tab === 'more') { renderMe(); if (can('admin')) loadPeople(); if (S.lab) labEnter(); else lab_set(null); }
  else telemetry(false);
  if (changed) { const p = $('#p-' + tab); if (p) p.scrollTop = 0; $('#sheet').scrollTop = 0; }
  if (tab !== 'drive' && tab !== 'map' && T.tool) setTool(T.tool);      // tools live on the map
  if (S.st) onState(S.st);
  setTimeout(resize, 260);
}
$$('.tab').forEach(t => t.onclick = () => {
  haptic(8);
  if (t.dataset.tab === 'more' && S.tab === 'more' && S.lab) lab_set(null);   // tap Operate again: back to its list
  setTab(t.dataset.tab);
});
// Map tab: tap the handle to shrink the sheet to a strip (more map) or grow it back
$('#sheet-grip').onclick = () => {
  const min = !document.body.classList.contains('sheet-min');
  document.body.classList.toggle('sheet-min', min);
  $('#sheet-grip').setAttribute('aria-expanded', !min);
  haptic(8); setTimeout(resize, 240);
};
// the alert stack (link lost, OK?, why, camera) pushes pages and map overlays down instead of covering them
if (window.ResizeObserver) {
  new ResizeObserver(() => {
    document.documentElement.style.setProperty('--alerts-h', `${Math.ceil($('#alerts').getBoundingClientRect().height) || 0}px`);
  }).observe($('#alerts'));
}
// Android back button (the APK asks this first): close a dialog, a sub-page, then go Home; false = leave the app
window.__pathikBack = () => {
  const dlg = $$('.action').filter(d => !d.hidden).pop();
  if (dlg) { if (dlg.id === 'action') closeAction(); else { const c = $('[data-cancel], button[type=button], [data-v="0"]', dlg); if (c) c.click(); else dlg.remove(); } return true; }
  if ($('#main').hidden) { if (!$('#auth').hidden) { disconnect(); return true; } return false; }
  if (CAM.big) { CAM.big = false; $('#cam').classList.remove('big'); return true; }
  if (T.tool) { setTool(T.tool); return true; }
  if (S.tab === 'more' && S.lab) { lab_set(null); return true; }
  if (S.tab !== 'home') { setTab('home'); return true; }
  return false;
};

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
  S.lastMap = m;
  const FREE = rgbOf(PAL.free), WALL = rgbOf(PAL.wall), MID = rgbOf(PAL.mid);
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
  if (view.follow && pose) {
    // keep the robot in the middle of the part of the map that isn't covered by the sheet
    const covered = (S.tab === 'map' && window.innerWidth < 820 ? ($('#sheet').offsetHeight || 0) : 0) * 0.5;
    view.x = pose.x; view.y = pose.y - covered / view.scale;
  }
  cx.setTransform(1, 0, 0, 1, 0, 0);
  cx.fillStyle = PAL.bg; cx.fillRect(0, 0, W, H);
  if (S.mapImg) {
    const m = S.map, [sx, sy] = W2S(m.origin[0], m.origin[1] + m.height * m.resolution);
    cx.imageSmoothingEnabled = false;
    cx.drawImage(S.mapImg, sx, sy, m.width * m.resolution * k, m.height * m.resolution * k);
  }
  // 1 m grid (scale reference)
  if (k > 18 * DPR) {
    cx.strokeStyle = PAL.grid; cx.lineWidth = 1;
    const [x0, y1] = S2W(0, 0), [x1, y0] = S2W(W / DPR, H / DPR);
    cx.beginPath();
    for (let gx = Math.floor(x0); gx <= x1; gx++) { const [px] = W2S(gx, 0); cx.moveTo(px, 0); cx.lineTo(px, H); }
    for (let gy = Math.floor(y0); gy <= y1; gy++) { const [, py] = W2S(0, gy); cx.moveTo(0, py); cx.lineTo(W, py); }
    cx.stroke();
  }
  if (!st) return;
  const navs = st.nav || {};
  // preview route (dashed) and active route
  const line = (pts, color, dash, width, glow) => {
    if (!pts || pts.length < 2) return;
    cx.save(); cx.strokeStyle = color; cx.lineWidth = width * DPR; cx.lineJoin = cx.lineCap = 'round';
    if (glow) { cx.shadowColor = color; cx.shadowBlur = 14 * DPR; }
    cx.setLineDash(dash.map(v => v * DPR)); cx.beginPath();
    pts.forEach((p, i) => { const [x, y] = W2S(p[0], p[1]); i ? cx.lineTo(x, y) : cx.moveTo(x, y); });
    cx.stroke(); cx.restore();
  };
  if (navs.preview && navs.preview.path && navs.state !== 'DRIVING') line(navs.preview.path, A(PAL.teal, 0.8), [7, 6], 3);
  if (navs.path && navs.path.length) line(pose ? [[pose.x, pose.y], ...navs.path] : navs.path, PAL.teal, [], 4, true);
  // exploration: open map edges (frontiers) and the area it is heading to
  const ex = navs.explore;
  if (ex && ex.active) {
    for (const f of ex.frontiers || []) {
      const [x, y] = W2S(f[0], f[1]), r = Math.min(14, 3 + Math.sqrt(f[2]) * 0.9) * DPR;
      cx.fillStyle = A(PAL.magenta, 0.22); cx.beginPath(); cx.arc(x, y, r, 0, 7); cx.fill();
      cx.fillStyle = PAL.magenta; cx.beginPath(); cx.arc(x, y, 2.4 * DPR, 0, 7); cx.fill();
    }
    if (ex.target) {
      const [x, y] = W2S(ex.target[0], ex.target[1]);
      cx.save(); cx.strokeStyle = PAL.magenta; cx.lineWidth = 2.5 * DPR; cx.setLineDash([5 * DPR, 4 * DPR]);
      cx.beginPath(); cx.arc(x, y, 13 * DPR, 0, 7); cx.stroke(); cx.restore();
    }
  }
  // rooms (room-level semantics): soft tinted areas with their name
  const rooms = (st.places && st.places.rooms) || [];
  cx.textAlign = 'center';
  for (const r of rooms) {
    const P = r.polygon || []; if (P.length < 3) continue;
    const here = navs.room === r.name;
    cx.beginPath(); P.forEach((p, i) => { const [x, y] = W2S(p[0], p[1]); i ? cx.lineTo(x, y) : cx.moveTo(x, y); }); cx.closePath();
    cx.fillStyle = here ? A(PAL.teal, 0.08) : A(PAL.orange, 0.05); cx.fill();
    cx.save(); cx.setLineDash([4 * DPR, 5 * DPR]); cx.lineWidth = 1.2 * DPR;
    cx.strokeStyle = here ? A(PAL.teal, 0.5) : A(PAL.orange, 0.35); cx.stroke(); cx.restore();
    if (k > 14 * DPR) {
      // label in the room's top-left corner, so it never sits on a place pin
      const minx = Math.min(...P.map(p => p[0])), maxy = Math.max(...P.map(p => p[1]));
      const [x, y] = W2S(minx, maxy);
      cx.textAlign = 'left';
      cx.font = `700 ${11 * DPR}px Bricolage, sans-serif`; cx.fillStyle = here ? A(PAL.teal, 0.85) : A(PAL.orange, 0.75);
      cx.fillText(r.name.toUpperCase(), x + 7 * DPR, y + 16 * DPR);
      if (k > 30 * DPR && r.contents && r.contents.length) {
        cx.font = `500 ${10.5 * DPR}px Instrument, sans-serif`; cx.fillStyle = PAL.sub;
        const t = r.contents.slice(0, 3).join(' · ') + (r.contents.length > 3 ? ' …' : '');
        cx.fillText(t, x + 7 * DPR, y + 30 * DPR);
      }
      cx.textAlign = 'center';
    }
  }
  // charging dock: a bracket in front of the docked robot (front docking), or behind it (old reverse docking)
  const dock = st.places && st.places.dock;
  if (dock) {
    const front = navs.dock_direction !== 'reverse', sgn = front ? 1 : -1;
    const dc = Math.cos(dock.yaw), ds = Math.sin(dock.yaw), bx = front ? 0.12 : -0.40;
    const D = (u, v) => W2S(dock.x + dc * u - ds * v, dock.y + ds * u + dc * v);
    cx.save(); cx.strokeStyle = PAL.good; cx.lineWidth = 3 * DPR; cx.lineCap = 'round'; cx.shadowColor = PAL.good; cx.shadowBlur = 10 * DPR;
    cx.beginPath(); [[bx - sgn * 0.12, 0.28], [bx, 0.28], [bx, -0.28], [bx - sgn * 0.12, -0.28]].forEach((p, i) => { const [x, y] = D(p[0], p[1]); i ? cx.lineTo(x, y) : cx.moveTo(x, y); });
    cx.stroke(); cx.restore();
    const [lx, ly] = D(bx + sgn * 0.12, 0); cx.font = `700 ${11 * DPR}px Instrument, sans-serif`; cx.fillStyle = PAL.good; cx.fillText('⚡', lx, ly + 4 * DPR);
  }
  // spots where the route barely fits the body (it slows down there)
  for (const t of navs.tight || []) {
    const [x, y] = W2S(t[0], t[1]);
    cx.strokeStyle = A(PAL.orange, 0.85); cx.lineWidth = 2 * DPR; cx.beginPath(); cx.arc(x, y, 6 * DPR, 0, 7); cx.stroke();
  }
  // places
  const pl = (st.places && st.places.places) || [];
  cx.font = `600 ${12 * DPR}px Instrument, sans-serif`; cx.textAlign = 'center';
  const diamond = (x, y, r, fill) => { cx.beginPath(); cx.moveTo(x, y - r); cx.lineTo(x + r, y); cx.lineTo(x, y + r); cx.lineTo(x - r, y); cx.closePath(); cx.fillStyle = fill; cx.fill(); };
  for (const p of pl) {
    const [x, y] = W2S(p.x, p.y);
    diamond(x, y, 7 * DPR, PAL.teal); diamond(x, y, 3 * DPR, PAL.bg);
    if (k > 22 * DPR) {
      const w = cx.measureText(p.name).width + 12 * DPR;
      cx.fillStyle = PAL.labelBg; cx.fillRect(x - w / 2, y - 26 * DPR, w, 17 * DPR);
      cx.fillStyle = PAL.label; cx.fillText(p.name, x, y - 13 * DPR);
    }
  }
  const home = st.places && st.places.home;
  if (home) { const [x, y] = W2S(home.x, home.y); diamond(x, y, 8 * DPR, PAL.orange); diamond(x, y, 3.5 * DPR, PAL.bg); }
  if (navs.goal) {
    const [x, y] = W2S(navs.goal.x, navs.goal.y);
    cx.save(); cx.strokeStyle = PAL.teal; cx.shadowColor = PAL.teal; cx.shadowBlur = 12 * DPR;
    cx.lineWidth = 2.5 * DPR; cx.beginPath(); cx.arc(x, y, 11 * DPR, 0, 7); cx.stroke(); cx.restore();
  }
  if (S.pick) {
    const [x, y] = W2S(S.pick[0], S.pick[1]);
    cx.strokeStyle = PAL.orange; cx.lineWidth = 2.5 * DPR; cx.beginPath(); cx.arc(x, y, 10 * DPR, 0, 7); cx.stroke();
  }
  // goal heading arrow (where the robot will face)
  if (navs.goal && fin(navs.goal.yaw)) drawArrow(navs.goal.x, navs.goal.y, navs.goal.yaw, Math.max(0.35, 24 / view.scale), PAL.teal, 3);
  drawTools();
  if (!pose) return;
  const c = Math.cos(pose.yaw), s = Math.sin(pose.yaw);
  const R = (bx, by) => W2S(pose.x + c * bx - s * by, pose.y + s * bx + c * by);
  // LiDAR returns
  const pts = (st.scan && st.scan.points) || [];
  cx.fillStyle = PAL.orange;
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
  const [fF, fR, fW] = navs.footprint || [0.07, 0.37, 0.21];
  const body = [[fF, fW], [fF, -fW], [-fR, -fW], [-fR, fW]].map(p => R(p[0], p[1]));
  cx.beginPath(); body.forEach((p, i) => i ? cx.lineTo(p[0], p[1]) : cx.moveTo(p[0], p[1])); cx.closePath();
  cx.save(); cx.shadowColor = PAL.magenta; cx.shadowBlur = 16 * DPR;
  cx.fillStyle = A(PAL.magenta, 0.92); cx.fill(); cx.restore();
  cx.lineWidth = 2 * DPR; cx.strokeStyle = '#ffffff'; cx.stroke();
  const arrow = [[0.17, 0], [0.0, 0.1], [0.03, 0], [0.0, -0.1]].map(p => R(p[0], p[1]));
  cx.beginPath(); arrow.forEach((p, i) => i ? cx.lineTo(p[0], p[1]) : cx.moveTo(p[0], p[1])); cx.closePath();
  cx.fillStyle = '#ffffff'; cx.fill();
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
    if (T.tool === 'draw' && T.draw) { ptrs.delete(e.pointerId); return; }         // finish or discard first
    T.drag = { sx: e.offsetX, sy: e.offsetY, a: S2W(e.offsetX, e.offsetY), b: S2W(e.offsetX, e.offsetY), px: 0,
      pts: [S2W(e.offsetX, e.offsetY)], lx: e.offsetX, ly: e.offsetY };
    mapDirty = true;
  } else if (ptrs.size === 1 && can('operator')) {
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
    if (T.tool === 'draw' && Math.hypot(e.offsetX - T.drag.lx, e.offsetY - T.drag.ly) > 4) {
      T.drag.pts.push(T.drag.b); T.drag.lx = e.offsetX; T.drag.ly = e.offsetY;
    }
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
const T = { tool: null, drag: null, route: [], loop: false, sent: null, draw: null };
const DRAG_PX = 14;                                   // shorter drags = tap (no heading)
const headingOf = d => d.px > DRAG_PX ? Math.atan2(d.b[1] - d.a[1], d.b[0] - d.a[0]) : null;
const deg = r => `${Math.round(((r * 57.2958) % 360 + 360) % 360)}°`;
function setTool(tool) {
  T.tool = T.tool === tool ? null : tool;
  $('#m-hint').classList.add('gone');
  T.drag = null;
  if (T.tool !== 'route') { T.route = []; }
  if (T.tool !== 'draw') closePath();
  if (T.tool && S.tab !== 'drive' && S.tab !== 'map') setTab('map');
  $$('.tool[data-tool]').forEach(b => { const on = b.dataset.tool === T.tool; b.classList.toggle('on', on); b.setAttribute('aria-pressed', on); });
  renderMode(); mapDirty = true;
}
$$('.tool[data-tool]').forEach(b => b.onclick = () => { haptic(10); setTool(b.dataset.tool); });
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
      <button class="btn grad" data-m="go" ${n ? '' : 'disabled'}>Go</button>
      <button class="btn ghost" data-m="close" aria-label="Close">✕</button>`;
  } else if (T.tool === 'draw' && !T.draw) {
    html = `<span class="grow"><b>Draw a path.</b> Drag your finger the way the robot should drive.</span><button class="btn ghost" data-m="close">Done</button>`;
  } else if (T.tool === 'room') {
    html = `<span class="grow"><b>Room.</b> Drag a box over a room, then say what is in it (“3D printer, oscilloscope”).</span><button class="btn ghost" data-m="close">Done</button>`;
  } else if (T.tool === 'locate') {
    html = `<span class="grow"><b>Set position.</b> Press where the robot really is, drag the way it faces.</span><button class="btn ghost" data-m="close">Done</button>`;
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
// ---- draw a path
function closePath() {
  T.draw = null; $('#path-sheet').hidden = true; document.body.classList.remove('path-open'); mapDirty = true;
}
async function finishDrawing(raw) {
  if (raw.length < 3) return;
  const pts = raw.map(([x, y]) => [+x.toFixed(3), +y.toFixed(3)]);
  try {
    const r = await nav({ type: 'check_path', points: pts });
    T.draw = { raw: pts, path: r.path, blocked: r.blocked || [], length: r.length, eta: r.eta_s };
    $('#ps-len').textContent = `${fmt(r.length, 1)} m`;
    $('#ps-eta').textContent = secs(r.eta_s);
    const bad = T.draw.blocked;
    $('#ps-warn').hidden = !bad.length;
    $('#ps-warn').textContent = bad.length ? `Too close to a wall at ${fmt(bad[0][0], 1)} m (red). Redraw that part.` : '';
    $('#ps-go').disabled = !!bad.length;
    $('#path-sheet').hidden = false; document.body.classList.add('path-open');
    renderMode(); mapDirty = true; haptic(15);
  } catch (e) { toast(e.message, true); }
}
$('#ps-close').onclick = () => { closePath(); renderMode(); };
$('#ps-redraw').onclick = () => { closePath(); renderMode(); };
$('#ps-go').onclick = async () => {
  if (!T.draw) return;
  try {
    await nav({ type: 'follow_path', points: T.draw.raw, repeat: $('#ps-loop').checked ? -1 : 0, replace: true });
    toast($('#ps-loop').checked ? 'Following your path on a loop' : 'Following your path'); haptic(25); setFollow(true);
    T.lastDrawn = T.draw.path;
    closePath(); setTool('draw');
  } catch (e) { toast(e.message, true); }
};

function roomDialog(title, room) {
  return formBox(title, `
    <label>Name<input name="name" required maxlength="60" placeholder="e.g. Lab 2" value="${esc(room ? room.name : '')}"></label>
    <label>What is in it? (comma separated)<textarea name="contents" rows="3" placeholder="3D printer, oscilloscope, soldering station">${esc(room ? (room.contents || []).join(', ') : '')}</textarea></label>
    <label>Other names (optional)<input name="aliases" maxlength="120" placeholder="electronics lab" value="${esc(room ? (room.aliases || []).join(', ') : '')}"></label>`,
    'Save room', room ? 'Delete room' : null);
}
async function toolRelease(d) {
  if (T.tool === 'draw') { await finishDrawing(d.pts || []); return; }
  if (T.tool === 'room') {
    const [x0, y0] = d.a, [x1, y1] = d.b;
    if (Math.abs(x1 - x0) < 0.7 || Math.abs(y1 - y0) < 0.7) { toast('Drag a bigger box over the room', true); return; }
    const f = await roomDialog('New room');
    if (!f || f.action !== 'ok') { mapDirty = true; return; }
    const polygon = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]].map(p => [+p[0].toFixed(2), +p[1].toFixed(2)]);
    try { await nav({ type: 'add_room', name: f.name, polygon, contents: f.contents, aliases: f.aliases }); toast(`Room ${f.name} saved`); haptic(20); }
    catch (e) { toast(e.message, true); }
    return;
  }
  if (T.tool === 'locate') {
    const pose = S.st && S.st.pose, yaw = headingOf(d) ?? (pose ? pose.yaw : 0), [x, y] = d.a;
    if (!await confirmBox('Robot is here?', `Set the robot's position to ${fmt(x, 1)}, ${fmt(y, 1)} m facing ${deg(yaw)}. The LiDAR then re-matches the map from there.`, 'Set position')) return;
    try {
      await post('/api/relocalize', { x, y, yaw });
      toast('Position set. When the orange dots sit on the walls, tap Position OK (Map tab).'); haptic(25); setTool('locate');
    } catch (e) { toast(e.message, true); }
    return;
  }
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
      if (p.yaw != null) drawArrow(p.x, p.y, p.yaw, Math.max(0.35, 26 / view.scale), PAL.orange, 2.5);
      const [x, y] = W2S(p.x, p.y);
      cx.fillStyle = PAL.orange; cx.beginPath(); cx.arc(x, y, 11 * DPR, 0, 7); cx.fill();
      cx.fillStyle = '#2a1a10'; cx.fillText(String(i + 1), x, y + 0.5 * DPR);
    });
    cx.textBaseline = 'alphabetic';
  }
  // drawn path: live stroke, then the checked path with blocked stretches in red
  const glowLine = (pts, color, w, dash) => {
    if (!pts || pts.length < 2) return;
    cx.save(); cx.strokeStyle = color; cx.shadowColor = color; cx.shadowBlur = 16 * DPR; cx.lineWidth = w * DPR;
    cx.lineCap = cx.lineJoin = 'round'; if (dash) cx.setLineDash(dash.map(v => v * DPR));
    cx.beginPath(); pts.forEach((p, i) => { const [x, y] = W2S(p[0], p[1]); i ? cx.lineTo(x, y) : cx.moveTo(x, y); }); cx.stroke(); cx.restore();
  };
  if (T.tool === 'draw' && T.drag && T.drag.pts) glowLine(T.drag.pts, PAL.teal, 4);
  if (T.draw && T.draw.path) {
    glowLine(T.draw.path, PAL.teal, 5);
    if (T.draw.blocked.length) {
      let acc = 0; const P = T.draw.path, red = [];
      for (let i = 1; i < P.length; i++) {
        acc += Math.hypot(P[i][0] - P[i - 1][0], P[i][1] - P[i - 1][1]);
        if (T.draw.blocked.some(([a, b]) => acc >= a - 0.1 && acc <= b + 0.1)) red.push([P[i - 1], P[i]]);
      }
      red.forEach(seg => glowLine(seg, '#ef3b2d', 6));
    }
    const P = T.draw.path, [ex, ey] = W2S(P[P.length - 1][0], P[P.length - 1][1]);
    cx.fillStyle = PAL.magenta; cx.beginPath(); cx.arc(ex, ey, 7 * DPR, 0, 7); cx.fill();
  }
  // the drag in progress
  const d = T.drag;
  if (!d || T.tool === 'draw') return;
  const [ax, ay] = W2S(d.a[0], d.a[1]);
  if (T.tool === 'measure') {
    if (d.px < 3) return;
    const [bx, by] = W2S(d.b[0], d.b[1]);
    cx.save(); cx.strokeStyle = PAL.label; cx.lineWidth = 2 * DPR; cx.setLineDash([5 * DPR, 4 * DPR]);
    cx.beginPath(); cx.moveTo(ax, ay); cx.lineTo(bx, by); cx.stroke(); cx.restore();
    [[ax, ay], [bx, by]].forEach(([x, y]) => { cx.fillStyle = PAL.label; cx.beginPath(); cx.arc(x, y, 4 * DPR, 0, 7); cx.fill(); });
    const m = Math.hypot(d.b[0] - d.a[0], d.b[1] - d.a[1]);
    cx.font = `700 ${13 * DPR}px Instrument, sans-serif`; cx.textAlign = 'center';
    const tx = (ax + bx) / 2, ty = (ay + by) / 2 - 10 * DPR, label = `${m.toFixed(2)} m`, w = cx.measureText(label).width + 12 * DPR;
    cx.fillStyle = PAL.labelBg; cx.fillRect(tx - w / 2, ty - 14 * DPR, w, 20 * DPR);
    cx.fillStyle = PAL.label; cx.fillText(label, tx, ty);
    return;
  }
  if (T.tool === 'room') {
    if (d.px < 3) return;
    const [bx, by] = W2S(d.b[0], d.b[1]);
    cx.save(); cx.strokeStyle = PAL.orange; cx.fillStyle = A(PAL.orange, 0.1); cx.lineWidth = 2 * DPR; cx.setLineDash([6 * DPR, 4 * DPR]);
    cx.beginPath(); cx.rect(Math.min(ax, bx), Math.min(ay, by), Math.abs(bx - ax), Math.abs(by - ay)); cx.fill(); cx.stroke(); cx.restore();
    cx.font = `700 ${12 * DPR}px Instrument, sans-serif`; cx.textAlign = 'center'; cx.fillStyle = PAL.orange;
    cx.fillText(`${fmt(Math.abs(d.b[0] - d.a[0]), 1)} × ${fmt(Math.abs(d.b[1] - d.a[1]), 1)} m`, (ax + bx) / 2, (ay + by) / 2);
    return;
  }
  const color = T.tool === 'goal' ? PAL.teal : T.tool === 'locate' ? PAL.magenta : PAL.orange;
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

// a small form in a sheet: resolves to {action: 'ok' | 'alt', ...fields} or null
function formBox(title, inner, okText, altText) {
  return new Promise(resolve => {
    const wrap = document.createElement('div');
    wrap.className = 'action';
    wrap.innerHTML = `<form class="action-card glass" role="dialog" aria-modal="true"><h3>${esc(title)}</h3>${inner}
      <button class="btn grad wide" type="submit">${esc(okText)}</button>
      ${altText ? `<button class="btn wide" type="button" data-alt>${esc(altText)}</button>` : ''}
      <button class="btn ghost wide" type="button" data-cancel>Cancel</button></form>`;
    document.body.appendChild(wrap);
    const form = $('form', wrap);
    const read = action => {
      const o = { action }; new FormData(form).forEach((v, k) => { o[k] = typeof v === 'string' ? v.trim() : v; });
      o.days = $$('.days button.on', form).map(b => +b.dataset.d); return o;
    };
    const done = v => { wrap.remove(); resolve(v); };
    form.onsubmit = e => { e.preventDefault(); done(read('ok')); };
    $('[data-cancel]', wrap).onclick = () => done(null);
    if (altText) $('[data-alt]', wrap).onclick = () => done(read('alt'));
    $$('.days button', wrap).forEach(b => b.onclick = () => { b.classList.toggle('on'); });
    setTimeout(() => { const i = $('input,textarea', form); if (i) i.focus(); }, 60);
    wrap.__form = form;
  });
}

// ------------------------------------------------------------------ drive: joystick
const jc = $('#joy'), jx = jc.getContext('2d');
const joy = { x: 0, y: 0, active: false, id: null };
function drawJoy() {
  const W = jc.width, R = W / 2, r = R * 0.34;
  jx.clearRect(0, 0, W, W);
  jx.beginPath(); jx.arc(R, R, R - 6, 0, 7); jx.fillStyle = PAL.joyBg; jx.fill();
  jx.save(); jx.shadowColor = PAL.teal; jx.shadowBlur = joy.active ? 22 : 12;
  jx.lineWidth = 4; jx.strokeStyle = joy.active ? PAL.teal : A(PAL.teal, 0.75); jx.stroke(); jx.restore();
  jx.fillStyle = A(PAL.teal, 0.8);
  [[0, -1], [0, 1], [-1, 0], [1, 0]].forEach(([dx, dy]) => {        // direction carets
    const cxp = R + dx * (R - 20), cyp = R + dy * (R - 20), a = Math.atan2(dy, dx);
    jx.beginPath(); jx.moveTo(cxp + Math.cos(a) * 7, cyp + Math.sin(a) * 7);
    jx.lineTo(cxp + Math.cos(a + 2.4) * 7, cyp + Math.sin(a + 2.4) * 7); jx.lineTo(cxp + Math.cos(a - 2.4) * 7, cyp + Math.sin(a - 2.4) * 7); jx.fill();
  });
  const kx = R + joy.x * (R - r - 8), ky = R - joy.y * (R - r - 8);
  jx.beginPath(); jx.arc(kx, ky, r, 0, 7);
  jx.fillStyle = joy.active ? A(PAL.teal, 0.28) : PAL.knob; jx.fill();
  jx.lineWidth = 3; jx.strokeStyle = joy.active ? PAL.teal : A(PAL.teal, 0.6); jx.stroke();
}
function joyFrom(e) {
  const r = jc.getBoundingClientRect(), R = r.width / 2;
  let x = (e.clientX - r.left - R) / (R * 0.66), y = -(e.clientY - r.top - R) / (R * 0.66);
  const m = Math.hypot(x, y); if (m > 1) { x /= m; y /= m; }
  joy.x = Math.abs(x) < 0.06 ? 0 : x; joy.y = Math.abs(y) < 0.06 ? 0 : y; drawJoy();
}
jc.addEventListener('pointerdown', e => {
  const why = S.st ? linkTooSlow() : '';
  if (why) { toast(`Driving by hand is paused: ${why}`, true); return; }
  jc.setPointerCapture(e.pointerId); joy.active = true; joy.id = e.pointerId; haptic(10); joyFrom(e);
});
jc.addEventListener('pointermove', e => { if (joy.active && e.pointerId === joy.id) joyFrom(e); });
const joyEnd = e => { if (e.pointerId !== joy.id) return; joy.active = false; joy.x = joy.y = 0; drawJoy(); sendCmd(0, 0, true); sendCmd(0, 0, true); };
jc.addEventListener('pointerup', joyEnd); jc.addEventListener('pointercancel', joyEnd);
// Driving by hand needs a fresh, fast link. The robot stops by itself 0.5 s after the last command
// (bridge watchdog); the app also never queues commands (a late one is worse than none) and pauses the
// joystick when the link is slow or silent, so a command can't arrive long after you let go.
const JOY_MAX_DELAY = 600, JOY_MAX_AGE = 1500;       // ms
const cmdLink = { inflight: false, ms: 0, stopped: false };
function linkTooSlow() {
  if (BASE === DEMO) return '';
  const age = performance.now() - (S.lastOk || 0);
  if (!S.lastOk || age > JOY_MAX_AGE) return `no answer from the robot for ${(age / 1000).toFixed(1)} s`;
  if (fin(S.rtt) && S.rtt > JOY_MAX_DELAY) return `link delay ${Math.round(S.rtt)} ms (max ${JOY_MAX_DELAY})`;
  return '';
}
function sendCmd(v, w, force = false) {
  if (cmdLink.inflight && !force) return;
  cmdLink.inflight = true;
  const t0 = performance.now();
  post('/api/cmd', { v, w }, { timeout: 800 })
    .then(() => { const d = performance.now() - t0; cmdLink.ms = cmdLink.ms ? cmdLink.ms * 0.7 + d * 0.3 : d; })
    .catch(() => {}).finally(() => { cmdLink.inflight = false; });
}
function setJoyLock(why) {
  const lock = $('#joy-lock');
  if (!lock) return;
  const was = !lock.hidden;
  lock.hidden = !why;
  if (why) lock.querySelector('span').textContent = why;
  $('.joy-wrap').classList.toggle('locked', !!why);
  if (why && !was && joy.active) { joy.active = false; joy.x = joy.y = 0; drawJoy(); toast('Link too slow: driving by hand paused', true); }
}
setInterval(() => {
  const why = S.st ? linkTooSlow() : '';
  setJoyLock(why);
  if (why) { if (!cmdLink.stopped && S.st) { sendCmd(0, 0, true); cmdLink.stopped = true; } return; }
  cmdLink.stopped = false;
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

function renderMapStats(st) {
  const box = $('#m-stats'), ex = (st.nav && st.nav.explore) || {};
  if (S.tab !== 'map' || T.tool) { box.hidden = true; return; }
  const m = st.map;
  if (ex.active) box.innerHTML = `<b>${fmt(ex.area, 1)} m²</b><small>${(ex.frontiers || []).length} open edges · ${secs(ex.elapsed_s)}</small>`;
  else if (m) box.innerHTML = `<b>${fmt(m.width * m.resolution, 1)} × ${fmt(m.height * m.resolution, 1)} m</b><small>${st.slam_mode === 'localization' ? 'saved map' : 'mapping live'}</small>`;
  box.hidden = !m && !ex.active;
  $('#map-mode-note').textContent = st.slam_mode === 'localization'
    ? 'Running on a saved map. If the robot is in the wrong place, use the Locate tool, then Position OK.'
    : st.slam_mode === 'mapping' ? 'Building a new map. Save it when it looks right, to drive on it later.' : '';
}

function renderGo(st, force) {
  renderRooms(st, force); renderSched(st, force); renderCharging(st);
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
  const key = JSON.stringify([pl.map(p => p.name), !!home, placeFilter, pose ? [Math.round(pose.x * 2), Math.round(pose.y * 2)] : 0, placeRoute]);
  if (!force && key === lastPlacesKey) return;
  lastPlacesKey = key;
  const match = p => !placeFilter || p.name.toLowerCase().includes(placeFilter) || (p.aliases || []).some(a => a.toLowerCase().includes(placeFilter));
  const rows = [];
  if (home && (!placeFilter || 'home'.includes(placeFilter))) rows.push({ name: 'Home', home: true, d: dist(home) });
  pl.filter(match).map(p => ({ ...p, d: dist(p) })).sort((a, b) => (a.d ?? 1e9) - (b.d ?? 1e9)).forEach(p => rows.push(p));
  const op = can('operator');
  $('#g-places').innerHTML = rows.length ? rows.map((p, i) => `
    <li class="${p.home ? 'home' : ''}"><span class="p-name">${esc(p.name)}<span class="p-sub">${p.home ? 'home · ' : ''}${fin(p.d) ? `${fmt(p.d, 1)} m away` : 'distance unknown'}</span></span>
      <span class="p-act"><button class="btn small" data-i="${i}" data-a="route" aria-label="Preview the route to ${esc(p.name)}">Route</button>
      <button class="btn small grad" data-i="${i}" data-a="go" aria-label="Send Pathik to ${esc(p.name)}">Go</button></span>
      ${placeRoute && placeRoute.name === p.name ? `<span class="p-route">${placeRoute.error ? `<span class="muted">${esc(placeRoute.error)}</span>`
        : `<b>${fmt(placeRoute.length, 1)} m</b> about <b>${secs(placeRoute.eta_s)}</b>`}<button class="btn small ghost" data-i="${i}" data-a="show">Show on map</button>
        ${op && !p.home ? `<button class="btn small ghost danger" data-i="${i}" data-a="delete">Delete place</button>` : ''}</span>` : ''}</li>`).join('')
    : `<li class="empty">${pl.length ? 'No place matches that search. Try a room or a thing in it.' : `No places yet. ${op ? 'Drive somewhere and tap “Save here”, or long‑press the map.' : 'Ask the robot team (an operator) to save some.'}`}</li>`;
  $$('#g-places button').forEach(b => b.onclick = async () => {
    const p = rows[+b.dataset.i];
    try {
      if (b.dataset.a === 'go') {
        await nav(p.home ? { type: 'go_home' } : { type: 'goto_place', name: p.name });
        toast(`Going to ${p.name}`); haptic(20); setFollow(true);
      } else if (b.dataset.a === 'route') {
        try {
          const r = await nav(p.home ? { type: 'preview', x: home.x, y: home.y, label: 'home' } : { type: 'preview', name: p.name });
          placeRoute = { name: p.name, length: r.length, eta_s: r.eta_s };
        } catch (e) { placeRoute = { name: p.name, error: e.message }; }
        renderGo(S.st, true);
      } else if (b.dataset.a === 'show') {
        setFollow(false);
        const t = p.home ? home : p; view.x = (t.x + (pose ? pose.x : t.x)) / 2; view.y = (t.y + (pose ? pose.y : t.y)) / 2;
        setTab('map');
      } else if (b.dataset.a === 'delete') {
        if (!await confirmBox(`Delete ${p.name}?`, 'The saved place goes. Rooms, schedules and the map stay.', 'Delete')) return;
        await nav({ type: 'delete_place', name: p.name }); placeRoute = null; toast(`${p.name} deleted`);
      }
    } catch (e) { toast(e.message, true); }
  });
}
let placeRoute = null;
// ---- rooms: "this room contains that"
let lastRoomsKey = '';
function renderRooms(st, force) {
  const rooms = (st.places && st.places.rooms) || [], navs = st.nav || {};
  const key = JSON.stringify([rooms, placeFilter, navs.room]);
  if (!force && key === lastRoomsKey) return;
  lastRoomsKey = key;
  const q = placeFilter;
  const hit = t => q && t.toLowerCase().includes(q);
  const shown = rooms.filter(r => !q || r.name.toLowerCase().includes(q) || (r.contents || []).some(hit) || (r.aliases || []).some(hit));
  $('#g-rooms').innerHTML = shown.length ? shown.map((r, i) => `
    <li><div class="r-top"><span><span class="r-name">${esc(r.name)}</span>
      <span class="r-sub">${navs.room === r.name ? ' · robot is here' : ''}</span></span>
      <span class="row gap">${can('operator') ? `<button class="btn small" data-r="${i}" data-a="edit">Edit</button>` : ''}<button class="btn small grad" data-r="${i}" data-a="go">Go</button></span></div>
      ${(r.contents || []).length ? `<div class="things">${r.contents.map(t => `<span class="${hit(t) ? 'hit' : ''}">${esc(t)}</span>`).join('')}</div>`
        : '<span class="r-sub">Nothing listed yet. Edit to add what is in this room.</span>'}</li>`).join('')
    : `<li class="empty">${rooms.length ? 'No room has that.' : (can('operator') ? 'No rooms yet. Map tab → Room tool, drag a box over a room, then list what is in it. Then say “take me to the 3D printer”.' : 'No rooms yet. The robot team (an operator) adds them on the map.')}</li>`;
  $$('#g-rooms button').forEach(b => b.onclick = async () => {
    const r = shown[+b.dataset.r];
    if (b.dataset.a === 'go') {
      try { await nav({ type: 'goto_room', name: r.name }); toast(`Going to ${r.name}`); haptic(20); setFollow(true); } catch (e) { toast(e.message, true); }
      return;
    }
    const f = await roomDialog('Edit room', r);
    if (!f) return;
    try {
      if (f.action === 'alt') {
        if (!await confirmBox(`Delete ${r.name}?`, 'The room outline and its list go. Places stay.', 'Delete')) return;
        await nav({ type: 'delete_room', name: r.name }); toast('Room deleted');
      } else { await nav({ type: 'update_room', name: r.name, new_name: f.name !== r.name ? f.name : undefined, contents: f.contents, aliases: f.aliases }); toast('Room updated'); }
    } catch (e) { toast(e.message, true); }
  });
}
$('#g-charge').onclick = async () => {
  haptic(15);
  try { await nav({ type: 'go_dock', replace: true }); toast('Going to charge'); setFollow(true); } catch (e) { toast(e.message, true); }
};
$('#g-dock').onclick = async () => {
  const front = !(S.st && S.st.nav && S.st.nav.dock_direction === 'reverse');
  if (!await confirmBox('Is the robot on the dock now?', front
    ? 'Push it nose-first onto the dock so the contacts touch (the charger LED changes), facing the marker. This spot becomes the dock: the robot drives in front of it and creeps on by itself when the battery is low.'
    : 'Push it onto the dock so the contacts touch (the charger LED changes). This spot becomes the dock: the robot backs onto it by itself when the battery is low.', 'Save dock here')) return;
  try { await nav({ type: 'set_dock', here: true }); toast('Dock saved'); } catch (e) { toast(e.message, true); }
};
function renderCharging(st) {
  const b = (st.nav && st.nav.battery) || {}, dock = st.places && st.places.dock;
  let t = '';
  if (b.level === 'unknown' || !fin(b.pct)) t = 'No battery reading. Fit the INA226 (v2.4 firmware) to see the charge and let the robot go and charge by itself.';
  else t = `Battery ${b.pct}% (${fmt(b.v, 2)} V${fin(b.a) ? `, ${b.a < 0 ? 'charging at' : 'using'} ${fmt(Math.abs(b.a), 2)} A` : ''}). `
    + (b.charging ? 'Charging.' : b.level === 'low' ? 'Low: it will go and charge after this task.' : b.level === 'critical' ? 'Critical: charging now, new tasks refused.' : 'Goes to charge by itself below 25%.');
  if (!dock) t += ' Dock not set yet.';
  $('#g-bat').textContent = t;
}
// ---- schedule: "patrol at 10:00 on weekdays"
const DAYS = ['Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat', 'Sun'];
let lastSchedKey = '';
function schedAction(a) {
  if (!a) return '';
  if (a.type === 'goto_place') return `go to ${a.name}`;
  if (a.type === 'go_home') return 'go home';
  if (a.type === 'go_dock') return 'charge';
  if (a.type === 'command') return `“${a.text}”`;
  if (a.type === 'explore_start') return 'map by itself';
  return a.type;
}
function renderSched(st, force) {
  const items = (st.nav && st.nav.schedules) || [];
  const key = JSON.stringify(items);
  if (!force && key === lastSchedKey) return;
  lastSchedKey = key;
  $('#g-sched').innerHTML = items.length ? items.map(s => `
    <li class="${s.enabled ? '' : 'off'}"><span><span class="s-time">${esc(s.at)}</span>
      <span class="s-sub"> ${esc(s.days_text)} · ${esc(schedAction(s.action))}</span><br><span class="s-sub">${esc(s.name)}${s.last ? ` · last ${esc(s.last.slice(5))}` : ''}</span></span>
      <span class="s-act"><label class="switch"><input type="checkbox" data-s="${s.id}" ${s.enabled ? 'checked' : ''} aria-label="On"></label>
      <button class="btn small ghost" data-del="${s.id}" aria-label="Delete">✕</button></span></li>`).join('')
    : '<li class="empty">Nothing scheduled. “Add” runs a trip or patrol at a set time every day or on chosen days (robot clock).</li>';
  $$('#g-sched input[data-s]').forEach(i => i.onchange = () => nav({ type: 'schedule_enable', id: +i.dataset.s, on: i.checked }).catch(e => toast(e.message, true)));
  $$('#g-sched [data-del]').forEach(b => b.onclick = async () => {
    if (!await confirmBox('Delete this schedule?', '', 'Delete')) return;
    nav({ type: 'schedule_delete', id: +b.dataset.del }).catch(e => toast(e.message, true));
  });
}
$('#g-sched-add').onclick = async () => {
  const pl = ((S.st && S.st.places && S.st.places.places) || []).map(p => p.name);
  const f = await formBox('New schedule', `
    <label>Name<input name="name" maxlength="60" placeholder="Morning patrol"></label>
    <label>Time (robot clock)<input name="at" type="time" required value="10:00"></label>
    <div class="days">${DAYS.map((d, i) => `<button type="button" data-d="${i}" class="${i < 5 ? 'on' : ''}">${d}</button>`).join('')}</div>
    <label>What to do<select name="what">
      ${pl.map(n => `<option value="place:${esc(n)}">Go to ${esc(n)}</option>`).join('')}
      <option value="home">Go home</option><option value="dock">Charge on the dock</option><option value="cmd">A command (type it below)</option>
    </select></label>
    <label>Command (if chosen)<input name="text" maxlength="200" placeholder="go to Lab 3, wait 2 minutes, then HOD office, then come back"></label>`, 'Save schedule');
  if (!f) return;
  let action;
  if (f.what.startsWith('place:')) action = { type: 'goto_place', name: f.what.slice(6) };
  else if (f.what === 'home') action = { type: 'go_home' };
  else if (f.what === 'dock') action = { type: 'go_dock' };
  else { if (!f.text) { toast('Type the command', true); return; } action = { type: 'command', text: f.text }; }
  try {
    await nav({ type: 'schedule_add', name: f.name, at: f.at, days: f.days.length === 7 ? [] : f.days, action });
    toast(`Scheduled for ${f.at}`); haptic(20);
  } catch (e) { toast(e.message, true); }
};

// ---- semantic commands: understand -> show the plan as a timeline -> send
function say(text) {
  if (!text) return;
  try { if (native && native.speak) native.speak(text); else if (window.speechSynthesis) speechSynthesis.speak(new SpeechSynthesisUtterance(text)); } catch (e) { /* silent */ }
}
const waitTxt = s => s >= 60 ? `wait ${fmt(s / 60, s % 60 ? 1 : 0)} min` : `wait ${Math.round(s)} s`;
async function planCommand(text, fromVoice) {
  const out = $('#g-reply'), plan = $('#g-plan');
  text = (text || '').trim(); if (!text) return;
  out.textContent = 'Understanding…'; plan.hidden = true;
  try {
    const r = await nav({ type: 'command', text, dry_run: true });
    const it = r.interpretation || {}, p = r.plan || { steps: [] };
    const src = it.source === 'llm' ? 'Understood by the language model' : 'Understood offline';
    out.innerHTML = `<span class="src">${src}.</span> ${esc(it.reply || '')} ${it.notes && it.notes.length ? `<span class="muted">${esc(it.notes.join(' '))}</span>` : ''}`;
    if (['stop', 'status', 'explain', 'find', 'confirm'].includes(it.intent)) {   // act at once: never delay a stop
      const done = await nav({ type: 'command', text });
      const ans = done.status || done.answer || (done.interpretation && done.interpretation.reply) || '';
      if (ans) out.innerHTML = `<span class="src">${src}.</span> <b>${esc(ans)}</b>`;
      say(it.intent === 'stop' ? 'Stopping.' : ans);
      return;
    }
    if (!p.steps.length) { if (fromVoice) say(it.reply || 'Sorry, I did not get a place.'); return; }
    const tags = [`<span class="step"><span class="tag start">Start</span><small>here</small></span>`];
    p.steps.forEach(st => {
      tags.push('<span class="tl-arrow">›</span>');
      tags.push(st.kind === 'home'
        ? `<span class="step"><span class="tag home">Return</span><small>home</small></span>`
        : st.kind === 'dock' ? `<span class="step"><span class="tag dock">Charge</span><small>dock</small></span>`
        : `<span class="step"><span class="tag go">Go</span><small>${esc(st.label)}</small></span>`);
      if (st.confirm) tags.push(`<span class="tl-arrow">›</span><span class="step"><span class="tag ok">OK?</span><small>wait for OK</small></span>`);
      if (st.wait_s > 0) tags.push(`<span class="tl-arrow">›</span><span class="step"><span class="tag wait">Wait</span><small>${esc(waitTxt(st.wait_s))}</small></span>`);
    });
    const missing = p.steps.find(st => st.missing);
    plan.innerHTML = `<h3>Plan ${p.priority >= 3 ? '· urgent' : ''}</h3><div class="timeline">${tags.join('')}</div>
      ${missing ? `<p class="note">${missing.kind === 'dock' ? 'The dock is not set yet (Go → Charging → Robot is on the dock).' : 'Home is not set, so it will not come back. Set home first.'}</p>` : ''}
      <div class="row gap"><button class="btn ghost" data-p="edit">Edit</button><button class="btn grad grow" data-p="send">Send to robot</button></div>`;
    plan.hidden = false;
    $('[data-p=edit]', plan).onclick = () => { plan.hidden = true; $('#g-cmd-in').focus(); };
    $('[data-p=send]', plan).onclick = async () => {
      try {
        const res = await nav({ type: 'command', text });
        plan.hidden = true; $('#g-cmd-in').value = '';
        toast(res.interpretation && res.interpretation.reply ? res.interpretation.reply : 'On the way'); haptic(25); setFollow(true);
        say(res.interpretation && res.interpretation.reply);
      } catch (e) { toast(e.message, true); }
    };
    if (fromVoice) say(it.reply ? `${it.reply} Shall I go?` : 'Shall I go?');
  } catch (err) { out.textContent = err.message; }
}
$('#g-cmd').onsubmit = e => { e.preventDefault(); planCommand($('#g-cmd-in').value, false); };
$('#g-cmd-in').addEventListener('keydown', e => { if (e.key === 'Enter' && !e.shiftKey) { e.preventDefault(); planCommand($('#g-cmd-in').value, false); } });

// ---- voice: Android speech recognizer through the APK, or the browser's when it has one
const voice = { on: false, rec: null };
function voiceUI(on, title, sub) {
  voice.on = on;
  $('#g-voice').classList.toggle('listening', on);
  $('#g-voice-t').textContent = title || (on ? 'Listening…' : 'Tap and speak');
  $('#g-voice-s').textContent = sub || (on ? 'Say where to go' : 'Pathik understands places, waits and “come back”');
  if (!on) $$('#g-wave i').forEach(i => { i.style.height = ''; });
}
window.__pathikVoice = (kind, text) => {
  if (kind === 'partial') { $('#g-cmd-in').value = text; voiceUI(true, 'Listening…', text); }
  else if (kind === 'level') {
    const lv = clamp((+text + 2) / 12, 0, 1);
    $$('#g-wave i').forEach((i, n) => { i.style.height = `${6 + lv * 24 * (0.5 + 0.5 * Math.abs(Math.sin(n * 1.7 + performance.now() / 120)))}px`; });
  }
  else if (kind === 'final') { $('#g-cmd-in').value = text; voiceUI(false); if (text) planCommand(text, true); }
  else if (kind === 'error') { voiceUI(false); toast(text, true); }
  else if (kind === 'end') voiceUI(true, 'Thinking…', $('#g-cmd-in').value);
};
$('#g-voice').onclick = () => {
  haptic(20);
  if (voice.on) { try { if (native && native.stopListening) native.stopListening(); else if (voice.rec) voice.rec.stop(); } catch (e) { /* */ } voiceUI(false); return; }
  if (native && native.listen) { voiceUI(true); native.listen('en-IN'); return; }
  const SR = window.SpeechRecognition || window.webkitSpeechRecognition;
  if (!SR) { toast('Voice works in the Pathik app (APK). In a browser, type the command.', true); return; }
  const rec = new SR(); voice.rec = rec;
  rec.lang = 'en-IN'; rec.interimResults = true;
  rec.onresult = ev => { const r = ev.results[ev.results.length - 1]; window.__pathikVoice(r.isFinal ? 'final' : 'partial', r[0].transcript); };
  rec.onerror = ev => window.__pathikVoice('error', ev.error === 'not-allowed' ? 'Microphone blocked (browsers need https for voice)' : `Voice error: ${ev.error}`);
  rec.onend = () => { if (voice.on) voiceUI(false); };
  voiceUI(true); rec.start();
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
// What kind of motor-board drops happened since the robot started (esp32_bridge counts them)
function linkDrops(d) {
  if (!d) return '–';
  const n = d.usb + d.resets + d.gaps;
  if (!n) return 'none';
  const parts = [d.usb && `${d.usb} USB`, d.resets && `${d.resets} ESP32 reboot${d.resets > 1 ? 's' : ''}${d.reset_reason ? ` (${d.reset_reason})` : ''}`,
    d.gaps && `${d.gaps} data gap${d.gaps > 1 ? 's' : ''}`].filter(Boolean).join(', ');
  return `${parts} · last ${d.last_age < 60 ? d.last_age + ' s' : Math.round(d.last_age / 60) + ' min'} ago`;
}
function renderStatus(st) {
  const L = st.link || {}, p = st.pose, n = st.nav || {}, se = st.sensors || {};
  const cls = (good, warn) => good ? 'ok' : warn ? 'warn' : 'bad';
  const items = [
    ['Network', `${netType(BASE)} · ${BASE.replace('http://', '')}`, ''],
    ['Link delay', fin(S.rtt) ? `${Math.round(S.rtt)} ms` : '–', cls(S.rtt < 250, S.rtt < 700)],
    ['Motor board', st.bridge_alive && L.connected ? `${fmt(L.rx_hz, 0)} Hz on ${L.port || '?'}` : 'not connected', cls(st.bridge_alive && L.rx_hz > 30, st.bridge_alive)],
    ['Link drops', linkDrops(L.drops), L.drops ? cls(!(L.drops.usb + L.drops.resets + L.drops.gaps), L.drops.last_age > 300) : ''],
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
    ['Battery', n.battery && fin(n.battery.pct) ? `${n.battery.pct}% · ${fmt(n.battery.v, 2)} V${fin(n.battery.a) ? ` · ${fmt(n.battery.a, 2)} A` : ''}` : 'no sensor',
      n.battery && fin(n.battery.pct) ? cls(n.battery.level === 'ok' || n.battery.charging, n.battery.level === 'low') : ''],
    ['Bumper', se.bumper === true ? 'PRESSED' : se.bumper === false ? 'clear' : 'not fitted', se.bumper ? 'bad' : ''],
    ['Body', n.footprint ? `${Math.round((n.footprint[0] + n.footprint[1]) * 100)} × ${Math.round(n.footprint[2] * 200)} cm, axle ${Math.round(n.footprint[0] * 100)} cm from front` : '–', ''],
    ['Room', n.room || '–', ''],
  ];
  $('#s-list').innerHTML = items.map(([k, v, c]) => `<div class="tile"><dt>${esc(k)}</dt><dd class="${c}">${esc(v)}</dd></div>`).join('');
  const ev = n.events || [];
  $('#s-events').innerHTML = ev.length ? ev.slice(0, 14).map(e => `<li class="${esc(e.level || '')}"><time>${clock(e.t)}</time><span>${esc(e.text)}</span></li>`).join('')
    : '<li><time></time><span class="muted">Nothing yet.</span></li>';
}
$('#s-disconnect').onclick = disconnect;
$('#t-robot').onclick = () => { S.lab = can('operator') ? 'health' : null; setTab('more'); };

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

// Operate area: a list (hub) of the robot-team tools; each opens as a sub-page with a back button
const SUBS = ['health', 'wheels', 'path', 'tests', 'log', 'people'];
$$('[data-sub]').forEach(b => b.onclick = () => { haptic(8); S.lab = b.dataset.sub; labEnter(); });
$$('[data-back]').forEach(b => b.onclick = () => { haptic(8); lab_set(null); });
function lab_set(which) {
  if (which === 'people' ? !can('admin') : which && !can('operator')) which = null;
  const was = S.lab;
  S.lab = which;
  $('#more-hub').hidden = !!which;
  SUBS.forEach(n => { $('#lab-' + n).hidden = n !== which; });
  if (was !== which) $('#p-more').scrollTop = 0;
  if (which === 'log') loadLog();
  if (which === 'health' && S.st) renderStatus(S.st);
  if (which === 'path') loadFollower();
  if (which === 'people') loadPeople();
  if (which !== 'wheels') telemetry(false); else if ($('#w-live').checked) telemetry(true);
  if (which && S.st) labOnState(S.st);
  setTimeout(resize, 30);
}
async function labEnter() {
  if (!lab.wheelRanges && S.lab === 'wheels') {
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
  g.font = `${11 * DPR}px Instrument, sans-serif`; g.fillStyle = PAL.dim2; g.strokeStyle = PAL.chartGrid; g.lineWidth = 1;
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
    g.fillStyle = PAL.dim2; g.font = `${13 * DPR}px Instrument, sans-serif`; g.textAlign = 'center';
    g.fillText(lab.live ? 'Waiting for wheel data…' : 'Turn on Live, or run a step test', c.width / 2, c.height / 2);
    return;
  }
  const t0 = data[0][0], t1 = data[data.length - 1][0];
  let lo = 0, hi = 50;
  for (const s of data) { lo = Math.min(lo, s[1], s[2], s[4], s[5]); hi = Math.max(hi, s[1], s[2], s[4], s[5]); }
  const pad = (hi - lo) * 0.1;
  const f = chartFrame(c, [0, Math.max(1, (t1 - t0) / 1000)], [lo - pad, hi + pad], '');
  const pt = i => data.map(s => [(s[0] - t0) / 1000, s[i]]);
  f.series(pt(1), '#ff9f2e', [5, 4], 1.8);
  f.series(pt(4), 'rgba(255,159,46,.5)', [2, 4], 1.5);
  f.series(pt(2), '#ffc15a', [], 2.2); f.series(pt(5), '#2de2c8', [], 2.2);
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
    f.series([[0, 0], [xmax, 0]], A(PAL.label.startsWith('#') ? PAL.label : '#888888', 0.3), [5, 5], 1.5);
    f.series(pts, '#e8407c', [], 3);
  } else {
    const yaw = unwrap(s.map(q => q[3])).map(v => v * 57.2958);
    const odo = s.map(q => fin(q[8]) ? (q[8] - s[0][8]) * 360 : null);
    const tmax = s[s.length - 1][0], all = [...yaw, ...odo.filter(fin)];
    const f = chartFrame(c, [0, tmax], [Math.min(0, ...all), Math.max(10, ...all) * 1.05], '°');
    f.series(s.map((q, i) => [q[0], yaw[i]]), '#e8407c', [], 3);
    if (odo.some(fin)) f.series(s.map((q, i) => [q[0], odo[i]]).filter(p => fin(p[1])), '#ff9f2e', [6, 4], 2);
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

// ------------------------------------------------------------------ mission log (evaluation)
let logRows = [];
async function loadLog() {
  try {
    const r = await nav({ type: 'mission_log', n: 100 });
    logRows = r.missions || [];
    const s = r.stats || {};
    const pc = v => fin(v) ? `${Math.round(v * 100)}%` : '–';
    $('#l-stats').innerHTML = [
      ['Missions', `${s.missions || 0}`], ['Success', pc(s.success_rate)],
      ['Avg time', secs(s.avg_duration_s)], ['Avg speed', fin(s.avg_speed_mps) ? `${fmt(s.avg_speed_mps, 2)} m/s` : '–'],
      ['Path efficiency', pc(s.path_efficiency)], ['Replans / mission', fin(s.avg_replans) ? fmt(s.avg_replans, 1) : '–'],
      ['Distance', `${fmt(s.total_distance_m, 0)} m`], ['Bumps · e-stops', `${s.bumps || 0} · ${s.estops || 0}`],
      ['Recoveries', `${s.recoveries || 0} · ${s.recovered || 0} missions saved`],
    ].map(([k, v]) => `<div class="tile"><dt>${esc(k)}</dt><dd>${esc(v)}</dd></div>`).join('');
    $('#l-list').innerHTML = logRows.length ? logRows.slice(0, 40).map(m => `
      <li class="${esc(m.result)}"><span class="l-top"><span>${esc(m.text || m.stops.join(' → '))}</span><span>${esc(m.result.toLowerCase())}</span></span>
      <span class="l-sub">${clock(m.started)} · ${secs(m.duration_s)} · ${fmt(m.distance_m, 1)} m driven / ${fmt(m.planned_m, 1)} m planned · ${m.replans} replans${m.bumps ? ` · ${m.bumps} bumps` : ''}${m.recoveries && m.recoveries.length ? ` · recovered by ${esc(m.recoveries.map(r => r.action.replace('_', ' ')).join(', '))}` : ''}${m.reason && m.result !== 'DONE' ? ` · ${esc(m.reason)}` : ''}</span></li>`).join('')
      : '<li class="empty muted">No finished missions yet.</li>';
  } catch (e) { toast(e.message, true); }
}
$('#l-refresh').onclick = loadLog;
$('#l-csv').onclick = async () => {
  if (!logRows.length) { toast('No missions logged yet', true); return; }
  const cols = ['id', 'started', 'text', 'source', 'result', 'reason', 'duration_s', 'distance_m', 'planned_m', 'straight_m', 'replans', 'blocked_s', 'bumps', 'estops', 'skipped', 'min_gap_m'];
  const q = v => { const t = String(v ?? ''); return /[",\n]/.test(t) ? `"${t.replace(/"/g, '""')}"` : t; };
  const csvText = [cols.join(',')].concat(logRows.map(r => cols.map(c => q(c === 'started' ? new Date(r.started * 1000).toISOString() : r[c])).join(','))).join('\n');
  try {
    if (native && native.share) { native.share('pathik_missions.csv', csvText); return; }
    const a = document.createElement('a');
    a.href = URL.createObjectURL(new Blob([csvText], { type: 'text/csv' })); a.download = 'pathik_missions.csv';
    document.body.appendChild(a); a.click(); a.remove();
    if (navigator.clipboard) navigator.clipboard.writeText(csvText).catch(() => {});
    toast('CSV saved (also copied)');
  } catch (e) { toast(e.message, true); }
};

// ------------------------------------------------------------------ greeting / where is the robot
function hello() { const h = new Date().getHours(); return h < 12 ? 'Good morning' : h < 17 ? 'Good afternoon' : 'Good evening'; }
function renderWhere(st) {
  const n = st.nav || {}, b = n.battery || {};
  const first = String((S.user && (S.user.name || S.user.username)) || '').split(/\s+/)[0];
  const parts = [];
  parts.push(n.room ? `Pathik is in ${n.room}` : st.pose ? 'Pathik is on the map' : 'Waiting for its position');
  parts.push($('#t-mode').textContent.toLowerCase());
  if (fin(b.pct)) parts.push(`battery ${b.pct}%`);
  const where = parts.join(' · ');
  $('#g-hello').textContent = `${hello()}${first ? ', ' + first : ''}`;
  $('#g-where').textContent = where;
  $('#sm-where').textContent = where + (n.goal ? `. Heading to ${n.goal.label}.` : '.');
}
// ------------------------------------------------------------------ Home: one calm status card + what it is doing
const LIGHT = { auto: 'auto', wait: 'wait', estop: 'bad', rc: 'wait', '': 'ok' };
function renderHome(st) {
  const n = st.nav || {}, b = n.battery || {}, mode = $('#t-mode');
  const offline = S.fails >= 3;
  $('#h-state').textContent = offline ? 'out of reach'
    : n.state === 'RECOVERING' ? (n.recovery && n.recovery.status === 'asking' ? 'asking for help' : 'recovering')
    : mode.textContent.replace(/ · why\?$/, '').toLowerCase();
  const cls = (mode.className.match(/\b(auto|wait|estop|rc)\b/) || [''])[0];
  $('#h-light').className = 'hs-light ' + (offline || !st.bridge_alive ? 'bad' : LIGHT[cls]);
  let sub = '';
  if (offline) sub = 'Reconnecting… The robot carries on with its job by itself.';
  else if (st.estop) sub = 'STOP is pressed. Motors are locked.';
  else if (n.goal && fin(n.distance)) sub = `${n.goal.label}: ${fmt(n.distance, 1)} m, about ${secs(n.eta_s)}`;
  else if (n.message) sub = n.message;
  $('#h-sub').textContent = sub;
  const pct = fin(b.pct) ? b.pct : null;
  $('#h-bat').textContent = pct == null ? 'no sensor' : `${pct}%${b.charging ? ' ⚡' : ''}`;
  const bar = $('#h-bat-bar');
  bar.style.width = `${pct == null ? 0 : clamp(pct, 2, 100)}%`; bar.className = b.level === 'low' || b.level === 'critical' ? b.level : '';
  $('#h-link').textContent = offline ? 'lost' : fin(S.rtt) ? `${Math.round(S.rtt)} ms` : '–';
  $('#h-link-sub').textContent = netType(BASE, st.net);
  $('#h-room').textContent = n.room || (st.pose ? 'on the map' : 'unknown');
  $('#h-room-sub').textContent = st.pose ? `${fmt(st.pose.x, 1)}, ${fmt(st.pose.y, 1)} m` : 'waiting for position';
  renderRecovery(n.recovery);
  renderDocking(n.docking, n.state);
  const busy = !$('#g-mission').hidden;
  $('#h-idle').hidden = busy || offline;
  // camera summary (the card itself opens from the chip or the Camera button)
  const v = st.vision, vc = $('#h-vis-card');
  vc.hidden = !v && !st.camera && BASE !== DEMO;
  if (!vc.hidden) {
    const act = v ? v.action : 'UNKNOWN', tag = $('#h-vis-tag');
    tag.textContent = v ? (VIS_TXT[act] || act) : 'Camera off'; tag.className = 'pill-tag ' + (VIS_CLS[act] || '');
    $('#h-vis').textContent = !v ? 'The camera program is not running on the robot.' : !v.camera_ok ? (v.camera_error || 'No picture from the camera.')
      : `${(v.people || []).length ? `${v.people.length} person${v.people.length > 1 ? 's' : ''} in view. ` : 'Nobody in view. '}${v.reason || ''}`;
  }
  // robot voice (Phase 5: push-to-talk on the robot); shown once the robot reports it
  const vo = st.voice, voc = $('#h-voice-card');
  voc.hidden = !vo;
  if (vo) {
    $('#h-voice-tag').textContent = vo.listening ? 'Listening' : vo.speaking ? 'Speaking' : vo.ok === false ? 'Off' : 'Ready';
    $('#h-voice-tag').className = 'pill-tag ' + (vo.listening ? 'slow' : vo.ok === false ? '' : 'go');
    $('#h-voice').textContent = vo.last_heard ? `Heard: “${vo.last_heard}”${vo.last_reply ? ` → “${vo.last_reply}”` : ''}` : (vo.message || 'Press the button on the robot and speak.');
  }
}
// LLM error recovery (Phase 3): what happened, what the robot decided, and a question when it needs a person
function renderRecovery(r) {
  const box = $('#h-recovery');
  if (!r || (r.status === 'done' && Date.now() / 1000 - (r.t || 0) > 120)) { box.hidden = true; box.dataset.key = ''; return; }
  const key = JSON.stringify([r.id, r.status, r.doing, r.left_s == null ? null : Math.round(r.left_s)]);
  if (box.dataset.key === key) return;
  box.dataset.key = key;
  const src = r.source === 'llm' ? 'Decided by the language model' : 'Decided by the safety rules';
  const ask = r.status === 'asking' && r.ask;
  box.innerHTML = `
    <div class="rc-h"><b class="title-s">${ask ? 'Pathik needs your help' : r.status === 'failed' ? 'Could not recover' : r.status === 'done' ? 'Recovered' : 'Recovering'}</b>
      <span class="pill-tag ${r.source === 'llm' ? 'go' : ''}">${esc(src)}</span></div>
    <dl><div><dt>What happened</dt><dd>${esc(r.what_happened || r.code || '')}</dd></div>
      <div><dt>What I'm doing</dt><dd>${esc(r.doing || '')}${fin(r.left_s) && r.left_s > 0 ? ` <span class="muted">(${secs(r.left_s)})</span>` : ''}</dd></div>
      ${r.why ? `<div><dt>Why</dt><dd class="muted small">${esc(r.why)}</dd></div>` : ''}</dl>
    ${ask ? `<p>${esc(r.ask.question || '')}</p><div class="rc-act">${(r.ask.options || []).map(o => `<button class="btn ${o.primary ? 'primary' : ''}" data-rc="${esc(o.id)}">${esc(o.label)}</button>`).join('')}</div>` : ''}`;
  box.hidden = false;
  if (ask && box.dataset.asked !== String(r.id)) {
    box.dataset.asked = String(r.id); haptic(60);
    if (S.tab !== 'home') toast('Pathik needs your help: see Home');
  }
  $$('[data-rc]', box).forEach(b => b.onclick = async () => {
    haptic(20);
    try { await nav({ type: 'recovery_answer', id: r.id, choice: b.dataset.rc }); toast('Thanks, carrying on'); box.dataset.key = ''; }
    catch (e) { toast(e.message, true); }
  });
}
// front docking with the camera marker (Phase 4): one step bar
const DOCK_STEPS = [['predock', 'In front'], ['align', 'Find marker'], ['creep', 'Creep in'], ['contact', 'Contact'], ['charging', 'Charging']];
function renderDocking(d, state) {
  const box = $('#h-docking');
  if (!d || !d.phase || d.phase === 'off') { box.hidden = true; return; }
  const idx = DOCK_STEPS.findIndex(s => s[0] === d.phase), failed = d.phase === 'failed', back = d.phase === 'backoff';
  const m = d.marker || {};
  box.innerHTML = `
    <div class="row between"><b class="title-s">${failed ? 'Docking failed' : back ? 'Backing off to try again' : d.phase === 'charging' ? 'Docked and charging' : 'Docking'}</b>
      <span class="pill-tag">${fin(d.try) ? `try ${d.try} of ${d.tries || '?'}` : esc(state || '')}</span></div>
    <ol class="dock-steps">${DOCK_STEPS.map(([k, l], i) => `<li class="${failed && i === Math.max(0, d.last_step ?? idx) ? 'fail' : i < idx ? 'done' : i === idx ? 'now' : ''}">${l}</li>`).join('')}</ol>
    <p class="muted small">${esc(d.message || '')}${m.found ? ` Marker ${fmt(m.z_m ?? m.x_m, 2)} m ahead, ${fmt((m.y_m || 0) * 100, 1)} cm to the side, ${fmt(m.yaw_deg, 1)}°.` : d.phase === 'align' ? ' Looking for the marker…' : ''}</p>`;
  box.hidden = false;
}
$('#h-send').onclick = () => { haptic(10); setTab('send'); setTimeout(() => $('#g-cmd-in').focus(), 280); };
$('#q-cam').onclick = () => { haptic(8); setCam(true); };
$('#q-home').onclick = async () => { haptic(15); try { await nav({ type: 'go_home', replace: true }); toast('Coming back'); setFollow(true); } catch (e) { toast(e.message, true); } };
$('#q-why').onclick = () => { haptic(10); explainNow(true); };
$('#q-charge').onclick = () => $('#g-charge').click();

// ------------------------------------------------------------------ Me: account, theme, robot, people
function renderMe(pending) {
  const u = S.user; if (!u) return;
  const ini = initials(u.name || u.username);
  $('#me-ava').textContent = ini; $('#g-ava').textContent = ini;
  $('#me-name').textContent = u.name || u.username;
  $('#me-user').textContent = u.legacy ? 'This robot has no accounts (old software)' : `@${u.username}`;
  const r = $('#me-role'); r.textContent = u.role; r.className = 'role ' + u.role;
  $('#me-robot').textContent = BASE === DEMO ? 'Pathik demo' : ((S.robot && (S.robot.robot || S.robot.host)) || 'robot');
  $('#me-net').textContent = `${netType(BASE, S.st && S.st.net)} · ${BASE === DEMO ? 'in the app' : shortAddr(BASE)}`;
  $('#me-dot').className = 'dot ' + (S.st && S.fails < 3 ? 'ok' : 'bad');
  $('#me-pass').hidden = $('#me-out').hidden = !!u.legacy;
  if (pending !== undefined) setPendingBadge(pending);
}
// Funnel on: the robot answers on a public https URL. Logins protect it, but it should be off after use.
function renderPublic(net) {
  const on = !!(net && net.public);
  $('#t-public').hidden = !on;
  const box = $('#me-public');
  box.hidden = !on;
  if (on) $('#me-public-url').textContent = net.public_url || 'a public https address';
}
function setPendingBadge(n) {
  ['#tab-me-badge', '#hub-people-badge'].forEach(id => { const b = $(id); b.textContent = n; b.hidden = !n || !can('admin'); });
  $('#hub-people').textContent = n && can('admin') ? `${n} waiting for approval` : 'Approve sign-ups, roles, passwords';
}
$$('[data-theme-pick]').forEach(b => b.onclick = () => { haptic(8); applyTheme(b.dataset.themePick); });
$$('.theme-toggle').forEach(b => b.onclick = () => {
  haptic(8); applyTheme(document.documentElement.getAttribute('data-theme') === 'dark' ? 'light' : 'dark');
});
$('#me-switch').onclick = disconnect;
$('#me-out').onclick = async () => {
  if (!await confirmBox('Log out?', 'You can log in again any time on this robot.', 'Log out')) return;
  try { await post('/api/auth/logout', {}); } catch (e) { /* offline: forget it locally anyway */ }
  sessions.set(BASE, null); TOKEN = ''; S.user = null; S.gen++; telemetry(false);
  showAuth('login');
};
$('#me-pass').onclick = async () => {
  const f = await formBox('Change password', `
    <label>Current password<input name="old" type="password" autocomplete="current-password" required></label>
    <label>New password (6+ characters)<input name="new" type="password" autocomplete="new-password" minlength="6" required></label>`, 'Change password');
  if (!f) return;
  try {
    const r = await post('/api/auth/password', { old: f.old, new: f.new });
    toast(r.message || 'Password changed');
    // the robot ends all sessions on a password change: log straight back in with the new one
    const l = await post('/api/auth/login', { username: S.user.username, password: f.new });
    TOKEN = l.token; sessions.set(BASE, { token: l.token, user: l.user });
  } catch (e) { toast(e.message, true); }
};
// admin: people (approve sign-ups, roles, disable, reset password, remove)
const ROLE_HELP = { staff: 'send & stop the robot', operator: 'also drive, map, tune', admin: 'also manage people' };
let peopleRows = [];
async function loadPeople() {
  if (!can('admin') || S.user.legacy) return;
  try {
    const r = await get('/api/users');
    peopleRows = r.users || [];
    setPendingBadge(peopleRows.filter(u => u.status === 'pending').length);
    renderPeople();
  } catch (e) { $('#people').innerHTML = `<li class="empty">${esc(e.message)}</li>`; }
}
function renderPeople() {
  const me = S.user.username;
  $('#people').innerHTML = peopleRows.length ? peopleRows.map((u, i) => {
    const pend = u.status === 'pending', off = u.status === 'disabled';
    const sub = pend ? 'wants access' : off ? 'disabled' : u.last_login ? `last seen ${new Date(u.last_login * 1000).toLocaleDateString()}` : 'never logged in';
    const roleSel = `<select data-i="${i}" data-a="role" aria-label="Role">${['staff', 'operator', 'admin'].map(r => `<option value="${r}" ${u.role === r ? 'selected' : ''}>${r} · ${ROLE_HELP[r]}</option>`).join('')}</select>`;
    return `<li class="${pend ? 'pending' : ''}">
      <div class="p-top"><span class="avatar sm">${esc(initials(u.name || u.username))}</span>
        <span class="grow"><b>${esc(u.name || u.username)}${u.username === me ? ' (you)' : ''}</b><small>@${esc(u.username)} · ${esc(sub)}</small></span>
        <span class="role ${pend ? 'pending' : off ? 'disabled' : u.role}">${pend ? 'pending' : off ? 'off' : u.role}</span></div>
      <div class="p-act">${pend
        ? `${roleSel}<button class="btn small grad" data-i="${i}" data-a="approve">Approve</button><button class="btn small" data-i="${i}" data-a="delete">Reject</button>`
        : u.username === me ? '' : `${roleSel}<button class="btn small" data-i="${i}" data-a="${off ? 'enable' : 'disable'}">${off ? 'Enable' : 'Disable'}</button>
          <button class="btn small" data-i="${i}" data-a="reset">New password</button><button class="btn small ghost" data-i="${i}" data-a="delete">Remove</button>`}</div></li>`;
  }).join('') : '<li class="empty">Nobody yet.</li>';
  $$('#people [data-a]').forEach(el => {
    const u = peopleRows[+el.dataset.i], a = el.dataset.a;
    const act = async () => {
      try {
        if (a === 'role' && u.status !== 'pending') { await post('/api/users/update', { username: u.username, role: el.value }); toast(`${u.name || u.username} is now ${el.value}`); }
        else if (a === 'approve') {
          const role = $(`select[data-i="${el.dataset.i}"]`).value;
          await post('/api/users/update', { username: u.username, role, status: 'active' }); toast(`Approved ${u.name || u.username} as ${role}`); haptic(20);
        } else if (a === 'disable' || a === 'enable') {
          await post('/api/users/update', { username: u.username, status: a === 'disable' ? 'disabled' : 'active' }); toast(a === 'disable' ? 'Account disabled' : 'Account enabled');
        } else if (a === 'reset') {
          const f = await formBox(`New password for ${u.name || u.username}`, '<label>New password (6+ characters)<input name="pw" type="text" minlength="6" required autocomplete="off"></label><p class="muted">Tell them in person. They can change it in Me → Change password.</p>', 'Set password');
          if (!f) return;
          await post('/api/users/reset', { username: u.username, password: f.pw }); toast('Password set');
        } else if (a === 'delete') {
          if (!await confirmBox(u.status === 'pending' ? `Reject ${u.name || u.username}?` : `Remove ${u.name || u.username}?`, 'Their account is deleted. They can ask for access again.', u.status === 'pending' ? 'Reject' : 'Remove')) return;
          await post('/api/users/delete', { username: u.username }); toast('Done');
        } else return;
      } catch (e) { toast(e.message, true); }
      loadPeople();
    };
    if (el.tagName === 'SELECT') el.onchange = act; else el.onclick = act;
  });
}
$('#pp-refresh').onclick = loadPeople;
// admins: check for new sign-up requests now and then (badge on the Me tab)
setInterval(async () => {
  if (!can('admin') || !S.user || S.user.legacy || $('#main').hidden) return;
  try { const me = await get('/api/auth/me', { timeout: 3000 }); setPendingBadge(me.pending); } catch (e) { /* link poll reports problems */ }
}, 30000);

// ------------------------------------------------------------------ boot
$('#c-scan').onclick = scan;
$('#c-demo').onclick = () => { haptic(15); connect(window.PathikDemo.info); };
$('#c-manual').onsubmit = async e => {
  e.preventDefault();
  const base = normalizeAddr($('#c-addr').value);
  if (!base) return;
  $('#c-progress').textContent = `Trying ${shortAddr(base)}…`;
  const r = await probe(base, 4000);                 // Tailscale / Funnel over mobile data can be slow
  if (r) connect(r); else $('#c-progress').textContent = `No robot answered at ${shortAddr(base)}. ` +
    (/ts\.net|^http:\/\/(slam-robot|100\.)/.test(base) ? 'Is Tailscale on (this phone and the robot)?' : 'Check the address and the Wi‑Fi.');
};
applyTheme();
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
