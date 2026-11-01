/* Phone detector (lab): people detection running ON THE PHONE, with the robot's own rules (vision.js).
 *
 * Picture source is the only thing that changes:
 *   PhoneCamera  the phone's camera (getUserMedia): works anywhere, no robot needed
 *   RobotCamera  the Pi camera's picture, fetched from the robot (/api/camera.jpg), when the app is connected
 * Detection: YOLO11n (same model family as the robot), 320 px, ONNX Runtime Web (WebAssembly) in the phone.
 * Robot address + login come in the URL hash from the Pathik app: detect.html#base=http://..:8080&token=...
 */
(function () {
  'use strict';
  const V = window.PathikVision;
  const $ = id => document.getElementById(id);
  const COL = { STOP: '#e6282a', SLOW_DOWN: '#ffaa00', CONTINUE: '#3cc850', UNKNOWN: '#a0a0a0' };

  const hash = new URLSearchParams(location.hash.slice(1));
  const robot = { base: hash.get('base') || '', token: hash.get('token') || '' };

  const st = { running: false, src: null, srcKind: 'phone', facing: 'environment',
               tracker: new V.PersonTracker(), fps: 0, lastT: 0, ms: 0, backend: '' };

  const DT = window.PathikDetector;
  const PhoneCamera = DT.PhoneCamera;
  const RobotCamera = DT.RobotCamera;

  async function loadModel() { st.backend = await DT.load(); }

  async function detect(f) {
    const conf = +$('conf').value;
    st.tracker.highConf = conf;                    // weaker boxes (down to 0.25) only continue a person's track
    const { boxes, ms } = await DT.run(f, Math.min(0.25, conf));
    st.ms = ms;
    const hfov = +$('fov').value;
    const people = st.tracker.update(boxes, performance.now() / 1000);
    for (const p of people) {
      p.bearing_deg = Math.round(V.bearingDeg(p.box, f.w, hfov) * 10) / 10;
      // No LiDAR on the phone: the camera's own distance estimate stands in for it (same metre thresholds as the
      // robot). A person cut off at the top/bottom edge has a too-short box (= looks too far): then fall back
      // to the box-size rule, exactly like the robot when the LiDAR has no hit.
      p.estimate_m = V.cameraDistance(p.box, f.w, hfov);
      const clipped = p.box[1] <= 2 || p.box[3] >= f.h - 2;
      p.distance_m = clipped || p.estimate_m == null ? null : Math.round(p.estimate_m * 100) / 100;
    }
    return { people, decision: V.decide(people, f.h) };
  }

  // ------------------------------------------------------------------ drawing
  const cv = $('cv'), ctx = cv.getContext('2d');
  function draw(f, r) {
    if (cv.width !== f.w || cv.height !== f.h) { cv.width = f.w; cv.height = f.h; }
    ctx.drawImage(f.img, 0, 0, f.w, f.h);
    const w = f.w, h = f.h, hfov = +$('fov').value, dec = r.decision;
    const half = V.DEFAULTS.path_half_angle_deg / hfov * w;
    ctx.strokeStyle = 'rgba(230,230,230,.8)'; ctx.lineWidth = Math.max(1, w / 400);
    for (const x of [w / 2 - half, w / 2 + half]) { ctx.beginPath(); ctx.moveTo(x, h); ctx.lineTo(x, h * 0.45); ctx.stroke(); }
    ctx.font = `bold ${Math.max(12, Math.round(w / 40))}px system-ui, sans-serif`;
    for (const p of r.people) {
      const [x1, y1, x2, y2] = p.box, inPath = Math.abs(p.bearing_deg) <= V.DEFAULTS.path_half_angle_deg;
      const c = inPath && dec.action === 'STOP' ? COL.STOP : inPath ? COL.SLOW_DOWN : '#00c8ff';
      ctx.strokeStyle = c; ctx.lineWidth = Math.max(2, w / 220); ctx.strokeRect(x1, y1, x2 - x1, y2 - y1);
      const label = `#${p.id}  ${p.bearing_deg > 0 ? '+' : ''}${Math.round(p.bearing_deg)}°  ~${p.estimate_m ? p.estimate_m.toFixed(1) : '?'} m${p.approaching ? '  >>' : ''}`;
      ctx.fillStyle = 'rgba(0,0,0,.6)';
      const tw = ctx.measureText(label).width + 10, th = Math.max(16, w / 32);
      ctx.fillRect(x1, Math.max(0, y1 - th), tw, th);
      ctx.fillStyle = c; ctx.fillText(label, x1 + 5, Math.max(th - 4, y1 - 5));
    }
    const bar = Math.max(22, h / 14);
    ctx.fillStyle = 'rgba(0,0,0,.75)'; ctx.fillRect(0, 0, w, bar);
    ctx.fillStyle = COL[dec.action] || COL.UNKNOWN;
    ctx.fillText(`${dec.action}  x${dec.speed_factor.toFixed(1)}  ${dec.reason}`.slice(0, 80), 8, bar * 0.72);
  }

  function show(r) {
    const d = r.decision;
    $('dec').className = 'decision ' + d.action;
    $('dec-a').textContent = { STOP: 'STOP', SLOW_DOWN: 'SLOW DOWN', CONTINUE: 'CONTINUE' }[d.action] || d.action;
    $('dec-r').textContent = `${d.reason} · speed ×${d.speed_factor.toFixed(1)}`;
    $('m-ms').textContent = Math.round(st.ms);
    $('m-fps').textContent = st.fps ? st.fps.toFixed(1) : '–';
    $('m-n').textContent = `${r.people.length} / ${d.people_in_path}`;
    $('people').innerHTML = r.people.map(p =>
      `<li>#${p.id} · ${p.bearing_deg > 0 ? '+' : ''}${p.bearing_deg}° · ~${p.estimate_m ? p.estimate_m.toFixed(1) : '?'} m (camera estimate) · ` +
      `conf ${p.conf}${p.approaching ? ' · approaching' : p.moving_away ? ' · moving away' : ''}</li>`).join('');
  }

  // ------------------------------------------------------------------ loop
  async function loop() {
    while (st.running) {
      try {
        const f = await st.src.frame();
        if (!f) { await sleep(30); continue; }
        const r = await detect(f);
        const now = performance.now();
        if (st.lastT) { const inst = 1000 / (now - st.lastT); st.fps = st.fps ? st.fps * 0.8 + inst * 0.2 : inst; }
        st.lastT = now;
        draw(f, r); show(r);
        $('msg').hidden = true;
      } catch (e) {
        msg(e.message || String(e));
        await sleep(st.srcKind === 'robot' ? 1000 : 300);
      }
      await sleep(0);                                         // let the page breathe between frames
    }
  }
  const sleep = ms => new Promise(r => setTimeout(r, ms));
  function msg(t) { $('msg').textContent = t; $('msg').hidden = false; }

  async function start() {
    if (st.running) return;
    try {
      st.src = st.srcKind === 'robot' ? new RobotCamera(robot.base, robot.token, true) : new PhoneCamera(st.facing);
      msg(st.srcKind === 'robot' ? 'Getting the robot camera picture…' : 'Starting the phone camera…');
      await st.src.start();
    } catch (e) {
      msg(e.name === 'NotAllowedError' ? 'Camera permission was refused: allow it for Pathik in the phone settings.' : e.message);
      return;
    }
    st.running = true; st.fps = 0; st.lastT = 0; st.tracker = new V.PersonTracker();
    $('run').textContent = 'Stop';
    loop();
  }
  function stop() {
    st.running = false;
    if (st.src) st.src.stop();
    $('run').textContent = 'Start';
  }

  function pick(kind) {
    const was = st.running;
    stop();
    st.srcKind = kind;
    $('src-phone').setAttribute('aria-pressed', kind === 'phone');
    $('src-robot').setAttribute('aria-pressed', kind === 'robot');
    $('flip').disabled = kind !== 'phone';
    $('fov').value = kind === 'robot' ? 102 : 68;           // Camera Module 3 Wide / a typical phone camera
    $('fov-v').textContent = $('fov').value + '°';
    if (was) start();
  }

  // ------------------------------------------------------------------ wiring
  $('src-phone').onclick = () => pick('phone');
  $('src-robot').onclick = () => pick('robot');
  $('src-robot').disabled = !robot.base;
  if (!robot.base) $('src-robot').title = 'Connect to a robot in the Pathik app first';
  $('run').onclick = () => (st.running ? stop() : start());
  $('flip').onclick = () => { st.facing = st.facing === 'environment' ? 'user' : 'environment'; if (st.running) { stop(); start(); } };
  $('conf').oninput = () => { $('conf-v').textContent = (+$('conf').value).toFixed(2); };
  $('fov').oninput = () => { $('fov-v').textContent = $('fov').value + '°'; };
  function goBack() {
    stop();
    if (window.PathikNative && PathikNative.closeDetector) PathikNative.closeDetector();
    else if (history.length > 1) history.back();
    else location.href = 'index.html';
    return true;
  }
  $('back').onclick = goBack;
  window.__pathikBack = goBack;                               // Android back button
  document.addEventListener('visibilitychange', () => { if (document.hidden && st.running) stop(); });

  $('run').disabled = true;
  loadModel().then(() => {
    const inf = DT.info, f1 = v => (v > 0 ? Math.round(v) + ' ms' : '–');
    $('m-be').textContent = st.backend + (inf ? ` (start test: GPU ${f1(inf.ms_gpu)}, CPU ${f1(inf.ms_cpu)})`
      : DT.nativeError ? ` (native: ${DT.nativeError})` : '');
    $('run').disabled = false;
    msg('Ready. Press Start.');
    $('dec-r').textContent = 'Press Start';
  }).catch(e => { msg(e.message || String(e)); $('m-be').textContent = 'not loaded'; });

  window.__detector = { st, detect, start, stop, pick };       // for tests
})();
