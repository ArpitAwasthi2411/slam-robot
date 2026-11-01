/* Peak test (peak.html): everything the head phone does, at once, for N minutes; one sample per second; verdict
 * from peakcheck.js. Phone health (battery, temperature, Android heat state, memory) comes from the Pathik app
 * (PathikNative.deviceStats); in a plain browser those rows say "not available".
 */
(function () {
  'use strict';
  const V = window.PathikVision, DT = window.PathikDetector, F = window.PathikFace, DOOR = window.PathikDoor, PK = window.PathikPeak;
  const $ = id => document.getElementById(id);
  const N = window.PathikNative;
  const hash = new URLSearchParams(location.hash.slice(1));
  const R = { base: (hash.get('base') || '').replace(/\/$/, ''), token: hash.get('token') || '' };
  const sleep = ms => new Promise(r => setTimeout(r, ms));
  const MOODS = ['idle', 'thinking', 'driving', 'blocked', 'happy', 'driving', 'help', 'happy'];
  const LINES = ['Going to the HOD office.', 'Excuse me, may I pass?', 'The door of Lab 2 is closed.',
                 'I will wait two minutes. Press OK on my screen if you are done earlier.', 'I am back.'];

  if (!R.base) { $('src').value = 'phone'; $('src').querySelector('[value="robot"]').disabled = true; }
  $('back').onclick = () => { stop(); if (N && N.closeDetector) N.closeDetector(); else if (history.length > 1) history.back(); else location.href = 'index.html'; };
  window.__pathikBack = () => { $('back').click(); return true; };

  let run = null;
  const face = new F.Face($('face'));
  face.set('idle');

  function deviceStats() {
    if (N && N.deviceStats) { try { return JSON.parse(N.deviceStats()); } catch (e) { /* old app */ } }
    const m = performance.memory;
    return { ok: false, app_mb: m ? m.usedJSHeapSize / 1048576 : null };
  }

  // ------------------------------------------------------------------ tiny charts (one line, last value)
  function spark(id, vals, color, fmt) {
    const c = $(id), dpr = window.devicePixelRatio || 1, w = c.clientWidth || 140, h = c.clientHeight || 44;
    if (c.width !== Math.round(w * dpr)) { c.width = Math.round(w * dpr); c.height = Math.round(h * dpr); }
    const x = c.getContext('2d');
    x.setTransform(dpr, 0, 0, dpr, 0, 0); x.clearRect(0, 0, w, h);
    const v = vals.filter(p => p[1] != null);
    if (v.length < 2) return;
    let lo = Math.min(...v.map(p => p[1])), hi = Math.max(...v.map(p => p[1]));
    if (hi - lo < 1e-6) { hi += 1; lo -= 1; }
    const t0 = v[0][0], t1 = Math.max(v[v.length - 1][0], t0 + 1);
    const X = t => 2 + (t - t0) / (t1 - t0) * (w - 4), Y = y => h - 13 - (y - lo) / (hi - lo) * (h - 26);
    x.strokeStyle = getComputedStyle(document.body).getPropertyValue('--line-2') || '#333';
    x.lineWidth = 1; x.beginPath(); x.moveTo(0, h - 11.5); x.lineTo(w, h - 11.5); x.stroke();
    x.strokeStyle = color; x.lineWidth = 2; x.lineJoin = 'round'; x.beginPath();
    v.forEach((p, i) => (i ? x.lineTo(X(p[0]), Y(p[1])) : x.moveTo(X(p[0]), Y(p[1]))));
    x.stroke();
    x.fillStyle = getComputedStyle(document.body).getPropertyValue('--dim') || '#999';
    x.font = '10px system-ui, sans-serif';
    x.textAlign = 'left'; x.fillText(fmt(lo), 2, h - 1);
    x.textAlign = 'right'; x.fillText(fmt(hi), w - 2, 9);
  }

  // ------------------------------------------------------------------ the run
  async function start() {
    $('err').textContent = '';
    const cfg = { dur: +$('dur').value, src: $('src').value, rate: +$('rate').value, det: $('c-det').checked,
                  face: $('c-face').checked, door: $('c-door').checked, say: $('c-say').checked };
    run = { cfg, on: true, t0: 0, samples: [], cur: { n: 0, ms: 0, frames: 0, long: 0 }, dev: {}, frame: null, engine: '', ids: new Set(), tracker: null };
    $('go').textContent = 'Stop'; $('go').classList.add('stop');
    $('live').hidden = false; $('result').hidden = true;
    $('status').textContent = 'Loading the detector…';
    if (N && N.keepAwake) N.keepAwake(true);
    try {
      if (cfg.det) run.engine = await DT.load();
      run.src = cfg.src === 'robot' ? new DT.RobotCamera(R.base, R.token, true) : new DT.PhoneCamera('environment');
      await run.src.start();
    } catch (e) {
      $('err').textContent = e.message || String(e); stop(); return;
    }
    run.t0 = performance.now();
    try {
      run.obs = new PerformanceObserver(l => { if (run) for (const e of l.getEntries()) run.cur.long += e.duration; });
      run.obs.observe({ type: 'longtask', buffered: false });
    } catch (e) { run.noLong = true; }
    frameLoop(run); faceLoop(run); sampleLoop(run);
    if (cfg.say) sayLoop(run);
  }

  async function frameLoop(r) {
    const cv = $('cv'), ctx = cv.getContext('2d');
    let lastDoor = 0, next = performance.now();
    while (r.on) {
      try {
        const f = await r.src.frame();
        if (!f) { await sleep(20); continue; }
        if (cv.width !== 320) { cv.width = 320; cv.height = 240; }
        let people = null;
        if (r.cfg.det) {
          const res = await DT.run(f, 0.25);
          r.cur.n++; r.cur.ms += res.ms;
          // what the robot uses: tracked people (weak boxes only continue a person, never flicker in on their own)
          people = (r.tracker = r.tracker || new V.PersonTracker()).update(res.boxes, performance.now() / 1000);
          for (const p of people) r.ids.add(p.id);
        }
        // picture and its boxes drawn together, so the boxes stay on screen until the next result
        ctx.drawImage(f.img, 0, 0, f.w, f.h, 0, 0, 320, 240);
        if (people) {
          const sx = 320 / f.w, sy = 240 / f.h;
          ctx.lineWidth = 2; ctx.font = 'bold 12px system-ui, sans-serif';
          for (const p of people) {
            const [x1, y1, x2, y2] = p.box;
            ctx.strokeStyle = '#ff9f2e'; ctx.strokeRect(x1 * sx, y1 * sy, (x2 - x1) * sx, (y2 - y1) * sy);
            ctx.fillStyle = '#ff9f2e'; ctx.fillRect(x1 * sx, Math.max(0, y1 * sy - 15), 30, 15);
            ctx.fillStyle = '#111'; ctx.fillText(`#${p.id}`, x1 * sx + 3, Math.max(11, y1 * sy - 3));
          }
        }
        const now = performance.now();
        if (r.cfg.door && now - lastDoor > 2000) { lastDoor = now; DOOR.check('__peak test__', f); r.cur.doors = (r.cur.doors || 0) + 1; }
        if (r.cfg.rate) { next += 1000 / r.cfg.rate; const wait = next - performance.now(); if (wait > 0) await sleep(wait); else next = performance.now(); }
        else if (!r.cfg.det) await sleep(200);
        else await sleep(0);
      } catch (e) {
        $('err').textContent = e.message || String(e);
        await sleep(500);
      }
    }
  }

  function faceLoop(r) {
    let k = 0, lastMood = 0;
    const tick = now => {
      if (!r.on) return;
      r.cur.frames++;
      if (r.cfg.face && now - lastMood > 2500) { lastMood = now; face.set(MOODS[k++ % MOODS.length]); }
      requestAnimationFrame(tick);
    };
    requestAnimationFrame(tick);
  }

  async function sayLoop(r) {
    let k = 0;
    while (r.on) {
      const text = LINES[k++ % LINES.length];
      try {
        if (N && N.speak) N.speak(text);
        else if (window.speechSynthesis) speechSynthesis.speak(new SpeechSynthesisUtterance(text));
      } catch (e) { /* no voice */ }
      for (let i = 0; i < 20 && r.on; i++) await sleep(1000);
    }
  }

  async function sampleLoop(r) {
    let last = performance.now(), devT = 0;
    while (r.on) {
      await sleep(1000);
      if (!r.on) break;
      const now = performance.now(), dt = (now - last) / 1000, t = Math.round((now - r.t0) / 1000);
      last = now;
      if (now - devT > 2000) { devT = now; r.dev = deviceStats(); }
      const c = r.cur, d = r.dev || {};
      r.samples.push({ t, det_n: r.cfg.det ? Math.round(c.n / dt * 10) / 10 : null, det_ms: c.n ? c.ms / c.n : null,
                       face_fps: Math.round(c.frames / dt), temp_c: d.temp_c ?? null, battery_pct: d.battery_pct ?? null,
                       charging: !!d.charging, thermal: d.thermal ?? null, app_mb: d.app_mb ?? null,
                       long_ms: r.noLong ? null : c.long / dt });
      r.cur = { n: 0, ms: 0, frames: 0, long: 0 };
      draw(r);
      if (t >= r.cfg.dur) { finish(r); break; }
    }
  }

  function draw(r) {
    const s = r.samples, L = s[s.length - 1], pick = k => s.map(x => [x.t, x[k]]);
    const left = Math.max(0, r.cfg.dur - L.t);
    $('prog').style.width = `${Math.min(100, L.t / r.cfg.dur * 100)}%`;
    $('status').textContent = `${Math.floor(left / 60)}:${String(left % 60).padStart(2, '0')} left · ` +
      (r.cfg.det ? `detector: ${r.engine || DT.backend}` : 'detection off') + (r.dev && r.dev.model ? ` · ${r.dev.model}` : '');
    const set = (id, v, unit) => { $(id).innerHTML = v == null ? '–' : `${v}<small> ${unit}</small>`; };
    set('v-ms', L.det_ms == null ? null : Math.round(L.det_ms), 'ms');
    set('v-fps', L.det_n, '/s');
    set('v-face', L.face_fps, 'fps');
    set('v-temp', L.temp_c == null ? null : L.temp_c.toFixed(1), '°C');
    set('v-bat', L.battery_pct == null ? null : Math.round(L.battery_pct), '%' + (L.charging ? ' ⚡' : ''));
    set('v-mem', L.app_mb == null ? null : Math.round(L.app_mb), 'MB');
    spark('g-ms', pick('det_ms'), '#ff9f2e', v => Math.round(v));
    spark('g-fps', pick('det_n'), '#2fbfa8', v => v.toFixed(1));
    spark('g-face', pick('face_fps'), '#2fbfa8', v => Math.round(v));
    spark('g-temp', pick('temp_c'), '#ff6a4d', v => v.toFixed(1));
    spark('g-bat', pick('battery_pct'), '#8a8af0', v => v.toFixed(0));
    spark('g-mem', pick('app_mb'), '#8a8af0', v => Math.round(v));
  }

  function stop() {
    if (!run) return;
    run.on = false;
    try { if (run.src) run.src.stop(); } catch (e) { /* gone */ }
    try { if (run.obs) run.obs.disconnect(); } catch (e) { /* none */ }
    if (N && N.keepAwake) N.keepAwake(false);
    $('go').textContent = 'Start peak test'; $('go').classList.remove('stop');
  }

  function finish(r) {
    stop();
    const res = PK.evaluate(r.samples, { rate: r.cfg.rate });
    r.result = res;
    $('result').hidden = false;
    $('verdict').className = 'pk-verdict ' + res.verdict;
    $('v-word').textContent = res.verdict;
    $('v-line').textContent = res.line;
    const cls = { ok: 'ok', warn: 'warn', bad: 'bad', na: 'na' }, mark = { ok: '✓', warn: '!', bad: '✗', na: '–' };
    $('checks').innerHTML = res.checks.map(c => `<tr><td>${c.name}</td><td>${c.target}</td><td>${c.value}</td>` +
      `<td class="${cls[c.status]}">${mark[c.status]}</td></tr>`).join('');
    const h = loadHist();
    h.unshift({ at: new Date().toISOString().slice(0, 16).replace('T', ' '), verdict: res.verdict, line: res.line,
                engine: r.engine || DT.backend || '', model: (r.dev && r.dev.model) || '' });
    try { localStorage.setItem('pathik.peak', JSON.stringify(h.slice(0, 8))); } catch (e) { /* private */ }
    showHist();
    const inf = DT.info || {};
    r.shareText = `Pathik peak test ${res.verdict}\n${res.line}\nDetector: ${r.engine || DT.backend}` +
      (inf.ms_gpu != null ? ` (start test: GPU ${inf.ms_gpu < 0 ? 'not available' : Math.round(inf.ms_gpu) + ' ms'}, CPU ${Math.round(inf.ms_cpu)} ms)` : '') +
      `\nPeople ids used: ${r.ids.size}` +
      (r.dev && r.dev.model ? `\nPhone: ${r.dev.model} (Android SDK ${r.dev.sdk}, ${r.dev.cores} cores)` : '') + '\n\n' +
      res.checks.map(c => `${{ ok: '✓', warn: '!', bad: '✗', na: '–' }[c.status]} ${c.name}: ${c.value} (target ${c.target})`).join('\n');
    window.__peakResult = { res, samples: r.samples, text: r.shareText };
  }

  function loadHist() { try { return JSON.parse(localStorage.getItem('pathik.peak') || '[]'); } catch (e) { return []; } }
  function showHist() {
    const h = loadHist();
    $('hist').innerHTML = h.length ? h.map(x => `<li><b>${x.verdict}</b> · ${x.at} · ${x.line}${x.engine ? ' · ' + x.engine : ''}</li>`).join('')
      : '<li>none yet</li>';
  }
  showHist();

  $('go').onclick = () => { if (run && run.on) { const r = run; stop(); if (r.samples.length > 15) finish(r); } else start(); };
  $('again').onclick = () => { $('result').hidden = true; start(); };
  $('share').onclick = () => {
    const t = run && run.shareText;
    if (!t) return;
    if (N && N.share) N.share('pathik-peak-test.txt', t);
    else if (navigator.clipboard) navigator.clipboard.writeText(t).then(() => { $('share').textContent = 'Copied'; });
  };
  window.__peak = { start, stop, get run() { return run; } };
})();
