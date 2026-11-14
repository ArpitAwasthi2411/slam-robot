/* Head mode: the phone mounted on the robot.
 *   face        mood from the robot's state (face.js), big line + subtitle, OK / answer buttons when it asks
 *   STOP        always, no PIN
 *   side icons  Go (places, rooms, home, cancel) · Ask (questions answered at once; a trip asked for in words
 *               is shown first) · Deliver (errand). The PIN is asked LAST: after choosing, just before the robot
 *               starts. Questions never need the PIN.
 *   face tap    the robot says its name and asks what it can do (the icons bounce); while busy, what it is doing
 *   detection   people detection ON THE PHONE on the Pi camera's plain picture, boxes POSTed to the robot
 *               (/api/vision/phone): the Pi then skips its own YOLO; only while the robot is busy
 *   errands     kiosk: go to a place, check its door, say a message, wait, come back, speak a report (mission.js)
 *   doors       settings: teach a door closed / open with the camera, check it (doorcheck.js)
 *   no robot    demo state (moods cycle) + optional detection test with the phone's own camera
 * Robot address + login arrive in the URL hash from the Pathik app: head.html#base=http://..:8080&token=...
 */
(function () {
  'use strict';
  const V = window.PathikVision, DT = window.PathikDetector, F = window.PathikFace;
  const M = window.PathikMission, DOOR = window.PathikDoor;
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
              det: { on: store.get('detect', true), test: false, running: false, ms: 0, fps: 0, last: 0, err: '' },
              ov: null, errand: null, lastGreet: -1e12, waitEnd: null };
  const MOVING = ['PLANNING', 'DRIVING', 'WAITING', 'SCANNING', 'DOCKING', 'UNDOCKING', 'RECOVERING'];
  const robotName = () => store.get('name', 'Pathik') || 'Pathik';

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
  const DEMO_TRIP = [['PLANNING', 1.5], ['DRIVING', 4], ['DRIVING-STOP', 2], ['DRIVING', 3]];   // errand legs: no OK stop
  function demoState() {
    let state = 'IDLE', vision = null, n = {};
    if (demo.goal) {
      let t = (performance.now() - demo.t0) / 1000, i = 0;
      const steps = demo.steps || DEMO_STEPS;
      while (i < steps.length && t > steps[i][1]) { t -= steps[i][1]; i++; }
      if (i >= steps.length) { demo.goal = null; } else {
        state = steps[i][0];
        if (state === 'DRIVING-STOP') { state = 'DRIVING'; vision = { action: 'STOP', reason: 'person 0.7 m ahead: too close, waiting' }; }
        n = { goal: { label: demo.goal }, distance: Math.max(0.5, 18 - i * 4 - t), eta_s: Math.max(3, 60 - i * 12) };
        if (state === 'CONFIRM') n.confirm = { label: demo.goal };
      }
    }
    return { estop: demo.estop, vision, places: DEMO_PLACES, pose: { x: 1.2, y: 0.4, yaw: 0 }, nav: Object.assign({ state, localized: true, message: '',
      battery: { pct: 78, level: 'ok', charging: false } }, n), net: { kind: 'demo' } };
  }
  async function demoApi(method, path, body) {
    if (path === '/api/state') return demoState();
    if (path === '/api/estop') { demo.estop = !!body.on; return { ok: true }; }
    if (path === '/api/nav') {
      const t = body.type;
      demo.steps = body.source === 'errand' ? DEMO_TRIP : DEMO_STEPS;
      if (['goto_place', 'goto_room'].includes(t)) { demo.goal = body.name; demo.t0 = performance.now(); }
      else if (t === 'goto_pose') { demo.goal = body.label || 'there'; demo.t0 = performance.now(); }
      else if (t === 'say') { /* the phone speaks it (say() below) */ }
      else if (t === 'command') {
        const text = body.text.trim(), q = /\?$|^(what|when|who|why|which|how|kya|kaun|kab|hello|hi)\b/i.test(text) && !/\b(go|take|bring)\b/i.test(text);
        const place = text.replace(/^(please\s+)?(take me to|go to|bring me to)\s+(the\s+)?/i, '').replace(/[?.!]+$/, '') || 'there';
        if (q) return { ok: true, interpretation: { intent: 'chat', targets: [], reply: /timing|open|kab/i.test(text)
          ? 'The department office is open from 9:30 am to 4:30 pm, Monday to Saturday.'
          : "I'm Pathik. In demo mode I only know a few things: ask me about office timings, or tell me where to go.",
          chat: { basis: 'knowledge' } }, plan: { steps: [] } };
        if (body.dry_run) return { ok: true, interpretation: { intent: 'navigate', targets: [place], reply: `Going to ${place}.` }, plan: { steps: [{ label: place }] } };
        demo.goal = place; demo.t0 = performance.now();
        return { ok: true, interpretation: { intent: 'navigate', targets: [place], reply: `Going to ${place}.` } };
      }
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
    let f = F.faceFor(st, { idleS });
    const moving = st && st.nav && MOVING.includes(st.nav.state);
    if (S.ov && !(st && st.estop) && !(moving && !S.ov.always)) f = Object.assign({}, f, S.ov);
    else if (S.errand && moving) f = Object.assign({}, f, { sub: `Errand ${S.errand.i + 1}/${S.errand.n} · ${f.sub || ''}` });
    else if (S.errand && !(st && st.estop) && !(st && st.nav && st.nav.recovery && st.nav.recovery.status === 'asking')) {
      f = Object.assign({}, f, { mood: 'happy', line: S.errand.line || 'On an errand', sub: `Errand ${S.errand.i + 1}/${S.errand.n}`, ok: false });
    }
    if (S.saying && !(st && st.estop)) f = Object.assign({}, f, { sub: `🔊 ${S.saying}` });
    S.face.set(f.mood);
    S.face.talk(!!S.saying);
    $('line').textContent = f.line;
    $('sub').textContent = f.sub || '';
    document.body.classList.toggle('estop', !!(st && st.estop));
    renderActs(f);
    renderErrand();
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
      txt = `camera test (${DT.backend}) · ${Math.round(d.ms)} ms · ${d.fps.toFixed(1)}/s · ${d.testDec || '…'}`; cls = 'ok';
    }
    else if (d.running) { txt = d.last ? `detecting on this phone (${DT.backend}) · ${Math.round(d.ms)} ms · ${d.fps.toFixed(1)}/s` : 'detecting on this phone · starting…'; cls = 'ok'; }
    else if (robotSide && robotSide.paused) txt = 'detection resting (robot idle)';
    else if (robotSide && robotSide.detected_by === 'pi') { txt = 'detection on the robot (Pi)'; cls = 'warn'; }
    else txt = S.model ? 'detection ready' : d.err ? `detector not loaded: ${d.err}` : 'loading detector…';
    $('c-det').textContent = txt; $('c-det').className = 'h-chip ' + cls;
  }

  let actsKey = '';
  function renderActs(f) {
    const key = f.mood + (f.ok ? 'ok' : '') + (f.okErrand ? 'oke' : '') + (f.ask ? JSON.stringify(f.ask) : '');
    if (key === actsKey) return;
    actsKey = key;
    const box = $('acts');
    box.innerHTML = '';
    const btn = (label, cls, fn) => {
      const b = document.createElement('button'); b.className = 'h-b ' + cls; b.textContent = label;
      b.onclick = e => { e.stopPropagation(); fn(); }; box.appendChild(b);
    };
    if (f.okErrand) btn('OK, done', 'go', () => { if (S.waitEnd) S.waitEnd('ok'); });
    else if (f.ok) btn('OK', 'go', () => nav({ type: 'confirm' }).catch(toastErr));
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
  async function needPin(why, title) { return pinPad(title || 'Enter the PIN', why); }

  // ------------------------------------------------------------------ Go / Ask / Deliver sheets
  // The PIN comes LAST: people look, choose and ask freely; only the moment the robot would start moving
  // (or someone's trip would be cancelled) asks for it. Questions are answered without it.
  const SHEETS = ['kiosk', 'ask', 'deliver'];
  let kioskTimer = 0;
  const anySheet = () => SHEETS.concat(['pin', 'set']).some(id => !$(id).hidden);
  function closeSheets() { SHEETS.forEach(id => { $(id).hidden = true; }); }
  function kioskIdle() { clearTimeout(kioskTimer); kioskTimer = setTimeout(closeSheets, 45000); }
  function targets() {
    const pl = (S.st && S.st.places) || { places: [], rooms: [] };
    S.targets = [].concat((pl.places || []).map(p => ({ name: p.name, req: { type: 'goto_place', name: p.name } })),
                          (pl.rooms || []).map(r => ({ name: r.name, req: { type: 'goto_room', name: r.name } })));
    return pl;
  }
  function openKiosk() {
    if (anySheet()) return;
    const pl = targets();
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
    $('k-msg').textContent = '';
    $('kiosk').hidden = false; kioskIdle();
  }
  const ASK_PIN = { goto_place: l => `Go to ${l}?`, goto_room: l => `Go to ${l}?`, go_home: () => 'Go home?',
                    cancel: () => 'Cancel the trip?' };
  async function send(req, label) {
    kioskIdle();
    const q = ASK_PIN[req.type];
    if (q && !await needPin(req.type === 'cancel' ? 'Enter the PIN to cancel it.' : 'Enter the PIN and I start.', q(label))) return;
    $('k-msg').textContent = 'Sending…';
    try {
      await nav(Object.assign({ priority: 1, source: 'head' }, req));
      $('k-msg').textContent = req.type === 'cancel' ? 'Trip cancelled.' : `On my way${label ? ' to ' + label : ''}!`;
      if (req.type === 'goto_place' || req.type === 'goto_room') say(`Okay, going to ${label}. Please follow me.`);
      else if (req.type === 'go_home') say('Okay, going home.');
      setTimeout(closeSheets, 900);
    } catch (e) { $('k-msg').textContent = e.message; }
  }
  $('ic-go').onclick = e => { e.stopPropagation(); haptic(10); S.faceT = performance.now(); openKiosk(); };
  $('k-home').onclick = () => send({ type: 'go_home' }, 'home');
  $('k-cancel').onclick = () => send({ type: 'cancel' }, '');
  $('k-close').onclick = closeSheets;

  // ---- Ask: a question is answered at once (shown, and spoken by the robot); a trip is shown, then the PIN
  const ANSWERS = ['chat', 'status', 'explain', 'find'];
  function askChips() {
    const pl = (S.st && S.st.places) || { places: [], rooms: [] };
    const thing = ((pl.rooms || []).find(r => (r.contents || []).length) || {}).contents;
    const place = ((pl.places || [])[0] || {}).name;
    const qs = ['What can you do?', 'Tell me about the department', thing ? `Where is the ${thing[0]}?` : null,
                place ? `Take me to ${place}` : null, 'What are you doing?'].filter(Boolean);
    $('a-chips').replaceChildren(...qs.map(q => {
      const b = document.createElement('button'); b.textContent = q; b.onclick = () => ask(q); return b; }));
  }
  function openAsk() {
    if (anySheet()) return;
    askChips();
    $('k-text').value = ''; $('a-msg').textContent = '';
    $('ask').hidden = false; kioskIdle();
  }
  function bubble(cls, text) {
    const d = document.createElement('div'); d.className = cls; d.textContent = text;
    const list = $('a-list'); list.appendChild(d);
    while (list.children.length > 12) list.firstChild.remove();
    list.scrollTop = list.scrollHeight;
  }
  let asking = false;
  async function ask(text) {
    text = (text || '').trim();
    if (!text || asking) return;
    asking = true; kioskIdle();
    $('k-text').value = ''; bubble('q', text); $('a-msg').textContent = 'Thinking…';
    S.ov = { mood: 'thinking', line: 'Let me think…', sub: text, always: true };
    try {
      const r = await nav({ type: 'command', text, dry_run: true, source: 'head' });
      const it = r.interpretation || {};
      S.ov = null;
      if (it.intent === 'stop') { await nav({ type: 'command', text, source: 'head' }); bubble('a', 'Stopped.'); }
      else if (ANSWERS.includes(it.intent) || it.intent === 'unknown' || !it.intent) {
        const done = it.intent && it.intent !== 'unknown' ? await nav({ type: 'command', text, source: 'head' }) : r;
        const reply = done.answer || done.status || (done.interpretation || it).reply || "I didn't understand that.";
        bubble('a', reply); $('a-msg').textContent = '';
        S.ov = { mood: 'happy', line: reply.length > 90 ? reply.slice(0, 88) + '…' : reply, sub: '', answer: true };
        setTimeout(() => { if (S.ov && S.ov.answer) S.ov = null; }, 9000);
        say(reply);
      } else {                                           // it wants the robot to move: show it, then the PIN
        const stops = ((r.plan || {}).steps || []).map(x => x.label || x.name).filter(Boolean);
        const what = it.intent === 'return_home' ? 'Go home' : it.intent === 'dock' ? 'Go and charge'
          : it.intent === 'confirm' ? 'Continue' : stops.length ? `Go to ${stops.join(', then ')}`
            : (it.targets || []).length ? `Go to ${it.targets.join(', then ')}` : (it.reply || 'Do this');
        bubble('a', `${what}. I need the PIN to start.`); $('a-msg').textContent = '';
        if (await needPin('Enter the PIN and I start.', `${what}?`)) {
          const done = await nav({ type: 'command', text, source: 'head', priority: 1 });
          const reply = (done.interpretation || it).reply || 'On my way.';
          bubble('a', reply); say(reply);
          setTimeout(closeSheets, 1200);
        } else bubble('a', 'Okay, I stay here.');
      }
    } catch (e) { S.ov = null; $('a-msg').textContent = e.message || String(e); }
    asking = false;
  }
  $('ic-ask').onclick = e => { e.stopPropagation(); haptic(10); S.faceT = performance.now(); openAsk(); };
  $('k-send').onclick = () => ask($('k-text').value);
  $('k-text').onkeydown = e => { kioskIdle(); if (e.key === 'Enter') $('k-send').click(); };
  $('a-close').onclick = closeSheets;
  // speak: the phone's speech recognizer (the robot's own speaker stays the robot's voice)
  if (window.PathikNative && PathikNative.listen) {
    $('k-mic').hidden = false;
    $('k-mic').onclick = () => {
      kioskIdle(); $('a-msg').textContent = 'Listening…';
      S.ov = { mood: 'listening', line: "I'm listening…", sub: '', always: true, mic: true };
      PathikNative.listen('en-IN');
    };
    window.__pathikVoice = (kind, text) => {
      if (kind === 'partial' || kind === 'final') $('k-text').value = text;
      if (kind === 'final' || kind === 'error') { if (S.ov && S.ov.mic) S.ov = null; }
      if (kind === 'final' && text) $('k-send').click();
      if (kind === 'error') $('a-msg').textContent = text;
    };
  }

  // ---- Deliver (errand)
  function openDeliver() {
    if (anySheet()) return;
    targets();
    $('e-to').innerHTML = S.targets.map((o, i) => `<option value="${i}">${esc(o.name)}</option>`).join('') ||
      '<option value="">no places saved</option>';
    $('e-info').textContent = '';
    $('deliver').hidden = false; kioskIdle();
  }
  $('ic-send').onclick = e => { e.stopPropagation(); haptic(10); S.faceT = performance.now(); openDeliver(); };
  $('d-close').onclick = closeSheets;

  // ---- tap the face: the robot tells about itself (one line per tap); while busy, what it is doing
  const ABOUT = [                                          // short and inviting: the name, then a question back
    n => `Hi, I'm ${n}! What can I do for you today?`,
    n => `I'm ${n}. Where would you like to go today?`,
    n => `Hello, ${n} here! Ask me something, or tell me where to go.`,
  ];
  let aboutI = 0;
  $('face-wrap').onclick = ev => {
    const now = performance.now(), st = S.st, idle = !st || !st.nav || ['IDLE', 'CHARGING'].includes(st.nav.state);
    S.faceT = now;
    const box = $('face').getBoundingClientRect();
    S.face.look((ev.clientX - box.left) / box.width * 2 - 1, (ev.clientY - box.top) / box.height * 2 - 1, 1500);
    if (anySheet() || S.saying || now - S.lastGreet < 1500) return;
    S.lastGreet = now;
    if (!idle || S.errand) {                               // busy: say what the face already shows
      const f = F.faceFor(st, {});
      if (store.get('greet', true) && !(st && st.estop)) say(f.line);
      return;
    }
    const text = ABOUT[aboutI++ % ABOUT.length](robotName());
    S.ov = { mood: 'happy', line: text, sub: 'Tap Go, Ask or Deliver', greet: true };
    setTimeout(() => { if (S.ov && S.ov.greet) S.ov = null; }, 700 + 62 * text.length + 2500);
    document.body.classList.add('invite');                 // the three icons bounce once: "your turn"
    setTimeout(() => document.body.classList.remove('invite'), 2600);
    if (store.get('greet', true)) say(text);
  };

  // ------------------------------------------------------------------ settings + leaving
  async function openSettings() {
    if (!await needPin('For the head settings.')) return;
    $('s-robot').textContent = DEMO ? 'Demo: no robot connected (open head mode from the app while connected).'
      : `Robot: ${R.base}`;
    $('s-detect').checked = S.det.on; $('s-test').checked = S.det.test;
    $('s-greet').checked = store.get('greet', true); $('s-name').value = robotName();
    const pl = (S.st && S.st.places) || { places: [], rooms: [] };
    const names = [...new Set([].concat((pl.places || []).map(p => p.name), (pl.rooms || []).map(r => r.name),
                                        DOOR.places().map(d => d.name)))];
    $('d-place').innerHTML = names.map(n => `<option>${esc(n)}</option>`).join('');
    $('d-demo').value = store.get('doorDemo', 'camera');
    $('d-demo-l').hidden = !DEMO;
    $('d-src').textContent = DEMO ? '(Demo: uses this phone\'s camera.)' : '(Uses the robot\'s camera.)';
    doorShow();
    $('set').hidden = false;
  }
  $('gear').onclick = openSettings;
  $('s-close').onclick = () => { $('set').hidden = true; };
  $('s-detect').onchange = e => { S.det.on = e.target.checked; store.set('detect', S.det.on); };
  $('s-test').onchange = e => { S.det.test = e.target.checked; stopDetect(); };
  $('s-greet').onchange = e => store.set('greet', e.target.checked);
  $('s-name').onchange = e => store.set('name', e.target.value.trim() || 'Pathik');
  $('d-demo').onchange = e => store.set('doorDemo', e.target.value);
  $('d-place').onchange = () => doorShow();
  function doorShow(res) {
    const p = $('d-place').value, t = p ? DOOR.taught(p) : { open: 0, closed: 0 };
    $('d-taught').textContent = p ? `taught: closed ×${t.closed}, open ×${t.open}` : 'no places yet';
    $('d-res').textContent = res ? res.state.toUpperCase() : ''; $('d-res').className = res ? res.state : '';
    $('d-why').textContent = res ? `${res.why} (closed ${res.closed}, open ${res.open})` : '';
  }
  async function doorFrame() {
    const src = DEMO ? new DT.PhoneCamera('environment') : new DT.RobotCamera(R.base, R.token, true);
    await src.start();
    try {
      for (let k = 0; k < 40; k++) { const f = await src.frame(); if (f) { await sleep(250); return (await src.frame()) || f; } await sleep(100); }
      throw new Error('no camera picture');
    } finally { setTimeout(() => src.stop(), 50); }
  }
  async function doorBtn(fn) {
    const p = $('d-place').value;
    if (!p) return;
    $('d-why').textContent = 'Looking…'; $('d-res').textContent = '';
    try { const f = await doorFrame(); doorShow(fn(p, f)); }
    catch (e) { $('d-why').textContent = e.message || String(e); }
  }
  $('d-closed').onclick = () => doorBtn((p, f) => { DOOR.teach(p, 'closed', f); return null; });
  $('d-open').onclick = () => doorBtn((p, f) => { DOOR.teach(p, 'open', f); return null; });
  $('d-check').onclick = () => doorBtn((p, f) => DOOR.check(p, f));
  $('d-forget').onclick = () => { DOOR.forget($('d-place').value); doorShow(); };
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

  // ------------------------------------------------------------------ speaking + errands
  async function say(text) {
    S.saying = text; render();
    try {
      if (DEMO) {
        if (window.PathikNative && PathikNative.speak) PathikNative.speak(text);
        else if (window.speechSynthesis) { const u = new SpeechSynthesisUtterance(text); u.lang = 'en-IN'; speechSynthesis.speak(u); }
      } else await nav({ type: 'say', text });
    } catch (e) { /* the robot can't speak: the subtitle still shows it */ }
    await sleep(700 + 62 * text.length);                     // let it finish before the next line
    if (S.saying === text) S.saying = '';
  }
  const lastFail = () => JSON.stringify(((S.st || {}).nav || {}).last_failure || null);
  async function trip(req, label) {
    const E = S.errand, before = lastFail();
    await nav(Object.assign({ priority: 1, source: 'errand', replace: true }, req));
    const t0 = performance.now();
    let started = false;
    for (;;) {
      await sleep(400);
      if (E.cancel) { await nav({ type: 'cancel' }).catch(() => {}); return { result: 'cancelled' }; }
      const n = (S.st || {}).nav || {}, st = n.state;
      if (S.st && S.st.estop) continue;
      if (!started) {
        if (st && st !== 'IDLE') started = true;
        else if (performance.now() - t0 > 8000) return { result: 'failed', why: n.message || 'the robot did not start' };
        continue;
      }
      if (st === 'CONFIRM') { await nav({ type: 'confirm' }).catch(() => {}); return { result: 'arrived' }; }
      if (st === 'IDLE' || st === 'DWELL' || st === 'CHARGING') {
        const lf = lastFail();
        if (lf !== before && lf !== 'null') {
          const f = n.last_failure || {};
          return { result: 'failed', why: f.message || f.reason || f.code || 'the trip failed' };
        }
        return { result: 'arrived' };
      }
      if (performance.now() - t0 > 15 * 60000) return { result: 'failed', why: 'it took too long' };
    }
  }
  function waitHere(seconds, label) {
    return new Promise(res => {
      const end = performance.now() + seconds * 1000, E = S.errand;
      const fin = how => { clearInterval(iv); S.waitEnd = null; S.ov = null; res(how); };
      S.waitEnd = fin;
      const tick = () => {
        if (E.cancel) return fin('cancelled');
        const left = Math.max(0, Math.ceil((end - performance.now()) / 1000));
        S.ov = { mood: 'happy', line: label, sub: `${Math.floor(left / 60)}:${String(left % 60).padStart(2, '0')} left · press OK when you are done`,
                 okErrand: true, always: true };
        if (left <= 0) fin('done');
      };
      const iv = setInterval(tick, 250); tick();
    });
  }
  async function checkDoor(place) {
    const mode = DEMO ? store.get('doorDemo', 'camera') : 'camera';
    S.ov = { mood: 'thinking', line: `Is the door of ${place} open?`, sub: 'Looking…', always: true };
    try {
      if (mode === 'open' || mode === 'closed') { await sleep(1200); return { state: mode, why: 'demo setting' }; }
      if (mode === 'random') { await sleep(1200); return { state: Math.random() < 0.5 ? 'open' : 'closed', why: 'demo: random' }; }
      const t = DOOR.taught(place);
      if (!t.open && !t.closed) return { state: 'unknown', why: 'this door was not taught (head settings → Doors)' };
      const votes = { open: 0, closed: 0, unknown: 0 };
      let last = null;
      for (let k = 0; k < 3; k++) { last = DOOR.check(place, await doorFrame()); votes[last.state]++; }
      const state = votes.closed >= 2 ? 'closed' : votes.open >= 2 ? 'open' : 'unknown';
      return { state, why: last.why };
    } catch (e) { return { state: 'unknown', why: e.message || String(e) }; }
    finally { S.ov = null; }
  }
  async function startErrand() {
    if (S.errand) return;
    const tgt = (S.targets || [])[+$('e-to').value];
    if (!tgt) { $('e-info').textContent = 'No place to go to yet.'; return; }
    const e = { to: tgt.name, waitS: +$('e-wait').value, message: $('e-msg').value, back: $('e-back').value };
    if (!await needPin('Enter the PIN and I start.', `Deliver to ${tgt.name}?`)) return;
    if (S.errand) return;
    const steps = M.plan(e), start = (S.st && S.st.pose) || null;
    const E = S.errand = { i: 0, n: steps.length, cancel: false, steps, plan: e };
    if (S.ov && S.ov.greet) S.ov = null;
    closeSheets();
    const ops = {
      say, cancelled: () => E.cancel,
      go: () => trip(tgt.req, tgt.name),
      goBack: back => back === 'home' ? trip({ type: 'go_home' }, 'home')
        : back === 'here' && start ? trip({ type: 'goto_pose', x: start.x, y: start.y, yaw: start.yaw || 0, label: 'where I started' }, 'where I started')
          : back === 'here' ? trip({ type: 'go_home' }, 'home') : trip({ type: 'goto_place', name: back }, back),
      door: checkDoor,
      wait: (sec, label) => waitHere(sec, label),
    };
    let r;
    try { r = await new M.Errand(ops).run(steps, u => { E.i = u.i; E.line = u.line; }); }
    catch (err) { r = { ok: false, report: `Errand stopped: ${err.message || err}` }; }
    S.errand = null;
    S.ov = { mood: r.ok ? 'happy' : 'help', line: r.ok ? 'Errand done' : 'Errand ended', sub: r.report, always: false };
    setTimeout(() => { if (S.ov && !S.ov.greet && !S.waitEnd) S.ov = null; }, 20000);
  }
  function renderErrand() {
    const E = S.errand;
    $('c-err').hidden = !E;
    $('e-new').hidden = !!E; $('e-run').hidden = !E;
    const bar = (el, E) => {
      el.hidden = !E;
      if (E) el.innerHTML = E.steps.map((_, k) => `<i class="${k < E.i ? 'done' : k === E.i ? 'now' : ''}"></i>`).join('');
    };
    bar($('steps'), E);
    if (!E) return;
    $('c-err').textContent = `errand ${E.i + 1}/${E.n}`;
    bar($('e-steps'), E);
    $('e-now').textContent = E.line || 'Errand running';
    $('e-plan').textContent = `${E.plan.to} · wait ${M.fmtWait(E.plan.waitS)} · then ${E.plan.back === 'here' ? 'come back' : E.plan.back === 'home' ? 'go home' : 'stay'}`;
  }
  $('e-start').onclick = () => { kioskIdle(); startErrand(); };
  $('e-cancel').onclick = async () => {
    if (!await needPin('Enter the PIN to cancel it.', 'Cancel the errand?')) return;
    if (S.errand) { S.errand.cancel = true; if (S.waitEnd) S.waitEnd('cancelled'); }
    closeSheets();
  };

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
        const r = await DT.run(f, 0.25);   // 0.25-0.45: only continues a tracked person (PersonTracker)
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
  window.__head = { S, faceFor: F.faceFor, demo, store };            // for tests
  start();
})();
