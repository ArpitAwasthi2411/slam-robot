/* Head mode: the phone mounted on the robot.
 *   face        mood from the robot's state (face.js), big line + subtitle, OK / answer buttons when it asks
 *   STOP        always, no PIN
 *   kiosk       tap the face -> PIN -> places, rooms, typed or spoken command, go home, cancel
 *   detection   people detection ON THE PHONE on the Pi camera's plain picture, boxes POSTed to the robot
 *               (/api/vision/phone): the Pi then skips its own YOLO; only while the robot is busy
 *   no robot    demo state (moods cycle) + optional detection test with the phone's own camera
 * Robot address + login arrive in the URL hash from the Pathik app: head.html#base=http://..:8080&token=...
 */
(function () {
  'use strict';
  const V = window.PathikVision, DT = window.PathikDetector, F = window.PathikFace;
  const $ = id => document.getElementById(id);
  const hash = new URLSearchParams(location.hash.slice(1));
  const R = { base: (hash.get('base') || '').replace(/\/$/, ''), token: hash.get('token') || '' };
  const DEMO = !R.base;
  const BUSY_KEEP_S = 30;                      // keep detecting this long after the robot stopped being busy
  const store = {
    get(k, d) { try { const v = localStorage.getItem('pathik.head.' + k); return v === null ? d : JSON.parse(v); } catch (e) { return d; } },
    set(k, v) { try { localStorage.setItem('pathik.head.' + k, JSON.stringify(v)); } catch (e) { /* private mode */ } },
  };
  const S = { st: null, fails: 0, busyT: -1e12, faceT: performance.now(), face: new F.Face($('face')), model: false,
              det: { on: store.get('detect', true), test: false, running: false, ms: 0, fps: 0, last: 0, err: '' } };

  // ------------------------------------------------------------------ robot API
  async function api(method, path, body) {
    if (DEMO) return demoApi(method, path, body);
    const r = await fetch(R.base + path, {
      method, cache: 'no-store',
      headers: Object.assign({ 'Content-Type': 'application/json' }, R.token ? { Authorization: 'Bearer ' + R.token } : {}),
      body: body ? JSON.stringify(body) : undefined });
    const j = await r.json().catch(() => ({}));
    if (!r.ok || j.ok === false) throw new Error(j.error || `HTTP ${r.status}`);
    return j;
  }
  const nav = req => api('POST', '/api/nav', req);

  // ------------------------------------------------------------------ demo robot (no connection)
  const demo = { t0: performance.now(), goal: null, estop: false, step: -1 };
  const DEMO_PLACES = { places: [{ name: 'HOD office' }, { name: 'Lab 2' }, { name: 'Seminar hall' }, { name: 'Library' }],
                        rooms: [{ name: 'Lab 3', contents: ['3D printer'] }, { name: 'Store', contents: ['tools'] }] };
  const DEMO_STEPS = [['PLANNING', 2], ['DRIVING', 5], ['DRIVING-STOP', 3], ['DRIVING', 4], ['CONFIRM', 6]];
  function demoState() {
    let state = 'IDLE', vision = null, n = {};
    if (demo.goal) {
      let t = (performance.now() - demo.t0) / 1000, i = 0;
      while (i < DEMO_STEPS.length && t > DEMO_STEPS[i][1]) { t -= DEMO_STEPS[i][1]; i++; }
      if (i >= DEMO_STEPS.length) { demo.goal = null; } else {
        state = DEMO_STEPS[i][0];
        if (state === 'DRIVING-STOP') { state = 'DRIVING'; vision = { action: 'STOP', reason: 'person 0.7 m ahead: too close, waiting' }; }
        n = { goal: { label: demo.goal }, distance: Math.max(0.5, 18 - i * 4 - t), eta_s: Math.max(3, 60 - i * 12) };
        if (state === 'CONFIRM') n.confirm = { label: demo.goal };
      }
    }
    return { estop: demo.estop, vision, places: DEMO_PLACES, nav: Object.assign({ state, localized: true, message: '',
      battery: { pct: 78, level: 'ok', charging: false } }, n), net: { kind: 'demo' } };
  }
  async function demoApi(method, path, body) {
    if (path === '/api/state') return demoState();
    if (path === '/api/estop') { demo.estop = !!body.on; return { ok: true }; }
    if (path === '/api/nav') {
      const t = body.type;
      if (['goto_place', 'goto_room'].includes(t)) { demo.goal = body.name; demo.t0 = performance.now(); }
      else if (t === 'command') { demo.goal = body.text.replace(/^(take me to|go to)\s+(the\s+)?/i, '') || 'there'; demo.t0 = performance.now(); }
      else if (t === 'go_home') { demo.goal = 'Home'; demo.t0 = performance.now(); }
      else if (t === 'cancel' || t === 'confirm') { demo.goal = null; }
      return { ok: true };
    }
    return { ok: true };
  }

  // ------------------------------------------------------------------ state polling -> face + chips
  async function poll() {
    for (;;) {
      try {
        const st = await api('GET', '/api/state');
        S.st = st; S.fails = 0;
        const s = (st.nav || {}).state;
        if (s && !['IDLE', 'CHARGING'].includes(s)) S.busyT = S.faceT = performance.now();
      } catch (e) {
        if (++S.fails >= 3) S.st = null;
      }
      render();
      await sleep(DEMO ? 300 : 400);
    }
  }

  function render() {
    const st = S.st, idleS = (performance.now() - S.faceT) / 1000;
    const f = F.faceFor(st, { idleS });
    S.face.set(f.mood);
    $('line').textContent = f.line;
    $('sub').textContent = f.sub || '';
    document.body.classList.toggle('estop', !!(st && st.estop));
    renderActs(f);
    const bat = st && st.nav && st.nav.battery;
    $('c-bat').textContent = bat && bat.pct != null ? `🔋 ${bat.pct} %${bat.charging ? ' ⚡' : ''}` : '🔋 –';
    $('c-bat').className = 'h-chip ' + (!bat || bat.pct == null ? '' : bat.level === 'ok' ? 'ok' : 'warn');
    $('c-net').textContent = DEMO ? 'demo (no robot)' : st ? 'robot connected' : 'robot not reachable';
    $('c-net').className = 'h-chip ' + (DEMO ? 'warn' : st ? 'ok' : 'bad');
    const d = S.det;
    const robotSide = st && st.vision;
    let txt, cls = '';
    if (d.running && d.err) { txt = `detection: ${d.err}`; cls = 'bad'; }
    else if (d.running && d.mode === 'test') {
      txt = `camera test · ${Math.round(d.ms)} ms · ${d.fps.toFixed(1)}/s · ${d.testDec || '…'}`; cls = 'ok';
    }
    else if (d.running) { txt = d.last ? `detecting on this phone · ${Math.round(d.ms)} ms · ${d.fps.toFixed(1)}/s` : 'detecting on this phone · starting…'; cls = 'ok'; }
    else if (robotSide && robotSide.paused) txt = 'detection resting (robot idle)';
    else if (robotSide && robotSide.detected_by === 'pi') { txt = 'detection on the robot (Pi)'; cls = 'warn'; }
    else txt = S.model ? 'detection ready' : d.err ? `detector not loaded: ${d.err}` : 'loading detector…';
    $('c-det').textContent = txt; $('c-det').className = 'h-chip ' + cls;
  }

  let actsKey = '';
  function renderActs(f) {
    const key = f.mood + (f.ok ? 'ok' : '') + (f.ask ? JSON.stringify(f.ask) : '');
    if (key === actsKey) return;
    actsKey = key;
    const box = $('acts');
    box.innerHTML = '';
    const btn = (label, cls, fn) => {
      const b = document.createElement('button'); b.className = 'h-b ' + cls; b.textContent = label;
      b.onclick = e => { e.stopPropagation(); fn(); }; box.appendChild(b);
    };
    if (f.ok) btn('OK', 'go', () => nav({ type: 'confirm' }).catch(toastErr));
    if (f.ask && f.ask.options) {
      f.ask.options.forEach(o => btn(o.label, o.primary ? 'go' : '', () =>
        nav({ type: 'recovery_answer', id: f.recId, choice: o.id }).catch(toastErr)));
    }
  }
  function toastErr(e) { $('sub').textContent = e.message || String(e); }

  // ------------------------------------------------------------------ STOP (no PIN), release (PIN)
  $('stop').onclick = () => { haptic(60); api('POST', '/api/estop', { on: true }).catch(toastErr); };

  // ------------------------------------------------------------------ PIN
  async function digest(s) {
    if (window.crypto && crypto.subtle) {
      const b = await crypto.subtle.digest('SHA-256', new TextEncoder().encode('pathik-head:' + s));
      return Array.from(new Uint8Array(b)).map(x => x.toString(16).padStart(2, '0')).join('');
    }
    let h = 7; for (const c of 'pathik-head:' + s) h = (h * 31 + c.charCodeAt(0)) >>> 0; return 'x' + h;
  }
  const pin = { val: '', resolve: null, mode: 'check', first: null };
  function pinPad(title, text, mode = 'check') {
    pin.val = ''; pin.mode = mode; pin.first = null;
    $('pin-t').textContent = title; $('pin-p').textContent = text; $('pin-e').textContent = '';
    drawDots(); $('pin').hidden = false;
    $('pin-x').hidden = mode === 'setup';
    return new Promise(res => { pin.resolve = res; });
  }
  function drawDots() {
    const n = Math.max(4, pin.val.length);
    $('pin-dots').innerHTML = Array.from({ length: n }, (_, i) => `<i class="${i < pin.val.length ? 'on' : ''}"></i>`).join('');
  }
  function closePin(ok) { $('pin').hidden = true; if (pin.resolve) pin.resolve(ok); pin.resolve = null; }
  async function pinEnter() {
    if (pin.val.length < 4) { $('pin-e').textContent = 'At least 4 digits'; return; }
    if (pin.mode === 'setup') {
      if (!pin.first) { pin.first = pin.val; pin.val = ''; drawDots(); $('pin-p').textContent = 'Type it again to be sure.'; return; }
      if (pin.first !== pin.val) { pin.first = null; pin.val = ''; drawDots(); $('pin-e').textContent = "They didn't match. Start again."; return; }
      store.set('pin', await digest(pin.val)); closePin(true); return;
    }
    if (await digest(pin.val) === store.get('pin', '')) { closePin(true); return; }
    pin.val = ''; drawDots(); $('pin-e').textContent = 'Wrong PIN'; haptic(80);
  }
  ['1', '2', '3', '4', '5', '6', '7', '8', '9', '⌫', '0', '✓'].forEach(k => {
    const b = document.createElement('button'); b.textContent = k;
    b.onclick = () => {
      haptic(8);
      if (k === '⌫') pin.val = pin.val.slice(0, -1);
      else if (k === '✓') { pinEnter(); return; }
      else if (pin.val.length < 8) pin.val += k;
      $('pin-e').textContent = ''; drawDots();
    };
    $('pad').appendChild(b);
  });
  $('pin-x').onclick = () => closePin(false);
  async function needPin(why) { return pinPad('Enter the PIN', why); }

  // ------------------------------------------------------------------ kiosk
  let kioskTimer = 0;
  function kioskIdle() { clearTimeout(kioskTimer); kioskTimer = setTimeout(() => { $('kiosk').hidden = true; }, 45000); }
  async function openKiosk() {
    if (!$('kiosk').hidden || !$('pin').hidden || !$('set').hidden) return;
    if (!await needPin('To choose where I go.')) return;
    const pl = (S.st && S.st.places) || { places: [], rooms: [] };
    const tile = (name, sub, req) => {
      const b = document.createElement('button');
      b.innerHTML = `${esc(name)}${sub ? `<small>${esc(sub)}</small>` : ''}`;
      b.onclick = () => send(req, name);
      return b;
    };
    $('k-places').replaceChildren(...(pl.places || []).map(p => tile(p.name, p.note || '', { type: 'goto_place', name: p.name })));
    $('k-rooms').replaceChildren(...(pl.rooms || []).map(r => tile(r.name, (r.contents || []).slice(0, 3).join(', '), { type: 'goto_room', name: r.name })));
    if (!(pl.places || []).length) $('k-places').innerHTML = '<p class="h-muted">No places saved yet.</p>';
    if (!(pl.rooms || []).length) $('k-rooms').innerHTML = '<p class="h-muted">No rooms saved yet.</p>';
    $('k-msg').textContent = ''; $('k-text').value = '';
    $('kiosk').hidden = false; kioskIdle();
  }
  async function send(req, label) {
    kioskIdle();
    $('k-msg').textContent = 'Sending…';
    try {
      await nav(Object.assign({ priority: 1, source: 'head' }, req));
      $('k-msg').textContent = `On my way${label ? ' to ' + label : ''}!`;
      setTimeout(() => { $('kiosk').hidden = true; }, 900);
    } catch (e) { $('k-msg').textContent = e.message; }
  }
  $('face-wrap').onclick = () => { S.faceT = performance.now(); openKiosk(); };
  $('k-send').onclick = () => { const t = $('k-text').value.trim(); if (t) send({ type: 'command', text: t }, ''); };
  $('k-text').onkeydown = e => { kioskIdle(); if (e.key === 'Enter') $('k-send').click(); };
  $('k-home').onclick = () => send({ type: 'go_home' }, 'home');
  $('k-cancel').onclick = () => send({ type: 'cancel' }, '').then(() => { $('k-msg').textContent = 'Trip cancelled.'; });
  $('k-close').onclick = () => { $('kiosk').hidden = true; };
  // speak: the phone's speech recognizer (the robot's own speaker stays the robot's voice)
  if (window.PathikNative && PathikNative.listen) {
    $('k-mic').hidden = false;
    $('k-mic').onclick = () => { kioskIdle(); $('k-msg').textContent = 'Listening…'; PathikNative.listen('en-IN'); };
    window.__pathikVoice = (kind, text) => {
      if (kind === 'partial' || kind === 'final') $('k-text').value = text;
      if (kind === 'final' && text) $('k-send').click();
      if (kind === 'error') $('k-msg').textContent = text;
    };
  }

  // ------------------------------------------------------------------ settings + leaving
  async function openSettings() {
    if (!await needPin('For the head settings.')) return;
    $('s-robot').textContent = DEMO ? 'Demo: no robot connected (open head mode from the app while connected).'
      : `Robot: ${R.base}`;
    $('s-detect').checked = S.det.on; $('s-test').checked = S.det.test;
    $('set').hidden = false;
  }
  $('gear').onclick = openSettings;
  $('s-close').onclick = () => { $('set').hidden = true; };
  $('s-detect').onchange = e => { S.det.on = e.target.checked; store.set('detect', S.det.on); };
  $('s-test').onchange = e => { S.det.test = e.target.checked; stopDetect(); };
  $('s-pin').onclick = async () => { $('set').hidden = true; await pinPad('New PIN', 'Choose a new PIN (4–8 digits).', 'setup'); };
  $('s-release').onclick = async () => {
    try { await api('POST', '/api/estop', { on: false }); await nav({ type: 'resume' }).catch(() => {}); $('set').hidden = true; }
    catch (e) { toastErr(e); }
  };
  function leave() {
    stopDetect();
    if (window.PathikNative && PathikNative.keepAwake) PathikNative.keepAwake(false);
    if (window.PathikNative && PathikNative.closeDetector) PathikNative.closeDetector();
    else if (history.length > 1) history.back();
    else location.href = 'index.html';
  }
  $('s-exit').onclick = leave;
  window.__pathikBack = () => { openSettings(); return true; };   // Android back -> PIN -> settings (exit there)

  // ------------------------------------------------------------------ detection on the phone
  function wantDetect() {
    const d = S.det;
    if (!S.model) return false;
    if (d.test) return true;                                    // camera test: always
    if (DEMO || !d.on || !S.st) return false;
    return (performance.now() - S.busyT) / 1000 < BUSY_KEEP_S;  // only while the robot is (recently) busy
  }
  function stopDetect() {
    const d = S.det;
    d.running = false; d.err = '';
    if (d.src) d.src.stop();
    d.src = null;
  }
  async function detectLoop() {
    const d = S.det;
    for (;;) {
      if (!wantDetect()) { if (d.running) stopDetect(); await sleep(300); continue; }
      try {
        if (!d.src) {
          d.src = d.test ? new DT.PhoneCamera('environment') : new DT.RobotCamera(R.base, R.token, true);
          d.mode = d.test ? 'test' : 'robot';
          await d.src.start();
          d.running = true; d.fps = 0; d.last = 0;
        }
        const f = await d.src.frame();
        if (!f) { await sleep(30); continue; }
        const r = await DT.run(f, 0.45);
        d.ms = r.ms; d.err = '';
        const now = performance.now();
        if (d.last) { const inst = 1000 / (now - d.last); d.fps = d.fps ? d.fps * 0.8 + inst * 0.2 : inst; }
        d.last = now;
        if (d.mode === 'robot') {
          await api('POST', '/api/vision/phone', { boxes: r.boxes.map(b => b.map(v => Math.round(v * 100) / 100)),
                                                   w: f.w, h: f.h, ms: Math.round(r.ms) });
        } else {
          // test mode: show what this phone's camera would decide (box size rule, no LiDAR)
          const people = (d.tracker = d.tracker || new V.PersonTracker()).update(r.boxes, now / 1000)
            .map(p => Object.assign(p, { bearing_deg: V.bearingDeg(p.box, f.w, 68) }));
          const dec = V.decide(people, f.h);
          d.testDec = `${dec.action}: ${dec.reason}`;
        }
      } catch (e) {
        d.err = e.message || String(e); d.running = true;
        if (d.src && d.mode === 'test') { d.src.stop(); d.src = null; }
        await sleep(1000);
      }
      await sleep(0);
    }
  }

  // ------------------------------------------------------------------ helpers + start
  const sleep = ms => new Promise(r => setTimeout(r, ms));
  const esc = s => String(s).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
  function haptic(ms) { try { if (window.PathikNative && PathikNative.haptic) PathikNative.haptic(ms); } catch (e) { /* none */ } }

  async function start() {
    if (window.PathikNative && PathikNative.keepAwake) PathikNative.keepAwake(true);
    if (!store.get('pin', '')) await pinPad('Set a PIN for head mode', 'People need it to send the robot from this screen and to leave head mode (4–8 digits).', 'setup');
    poll();
    DT.load().then(() => { S.model = true; }).catch(e => { S.det.err = e.message; S.model = false; render(); });
    detectLoop();
  }
  window.__head = { S, faceFor: F.faceFor, demo };            // for tests
  start();
})();
