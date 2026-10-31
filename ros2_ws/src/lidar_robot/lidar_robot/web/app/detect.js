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
  const SIZE = 320;
  const MODEL = 'vendor/yolo11n_320.onnx';
  const $ = id => document.getElementById(id);
  const COL = { STOP: '#e6282a', SLOW_DOWN: '#ffaa00', CONTINUE: '#3cc850', UNKNOWN: '#a0a0a0' };

  const hash = new URLSearchParams(location.hash.slice(1));
  const robot = { base: hash.get('base') || '', token: hash.get('token') || '' };

  const st = { session: null, input: 'images', running: false, src: null, srcKind: 'phone', facing: 'environment',
               tracker: new V.PersonTracker(), fps: 0, lastT: 0, ms: 0, backend: '' };

  // ------------------------------------------------------------------ picture sources
  class PhoneCamera {
    constructor(facing) { this.facing = facing; this.video = null; this.stream = null; }
    async start() {
      if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
        throw new Error(window.isSecureContext ? 'This browser has no camera access.'
          : 'The camera only works in the Pathik app (or on https / localhost).');
      }
      this.stream = await navigator.mediaDevices.getUserMedia({
        audio: false, video: { facingMode: { ideal: this.facing }, width: { ideal: 640 }, height: { ideal: 480 } } });
      const v = document.createElement('video');
      v.playsInline = true; v.muted = true; v.srcObject = this.stream;
      await v.play();
      this.video = v;
    }
    async frame() {
      const v = this.video;
      if (!v || v.readyState < 2 || !v.videoWidth) return null;
      return { img: v, w: v.videoWidth, h: v.videoHeight };
    }
    stop() { if (this.stream) this.stream.getTracks().forEach(t => t.stop()); this.stream = null; this.video = null; }
  }

  class RobotCamera {
    constructor(base, token) { this.base = base.replace(/\/$/, ''); this.token = token; this.last = null; }
    async start() { if (!this.base) throw new Error('Not connected to a robot: open this from the Pathik app while connected.'); }
    async frame() {
      const r = await fetch(`${this.base}/api/camera.jpg?t=${Date.now()}`,
        { headers: this.token ? { Authorization: 'Bearer ' + this.token } : {}, cache: 'no-store' });
      if (!r.ok) throw new Error(r.status === 404 ? 'The robot has no camera picture (is the camera node running?)'
        : `Robot camera: HTTP ${r.status}`);
      const bmp = await createImageBitmap(await r.blob());
      if (this.last && this.last.close) this.last.close();
      this.last = bmp;
      return { img: bmp, w: bmp.width, h: bmp.height };
    }
    stop() { if (this.last && this.last.close) this.last.close(); this.last = null; }
  }

  // ------------------------------------------------------------------ model
  async function loadModel() {
    if (!window.ort) throw new Error('Detector files are missing (vendor/ort). Use the Pathik app (APK).');
    const threads = window.crossOriginIsolated ? Math.max(1, Math.min(4, (navigator.hardwareConcurrency || 2) - 1)) : 1;
    ort.env.wasm.wasmPaths = new URL('vendor/ort/', location.href).href;
    ort.env.wasm.numThreads = threads;
    ort.env.wasm.proxy = false;
    const res = await fetch(MODEL);
    if (!res.ok) throw new Error(`Model not found (${MODEL}).`);
    const buf = await res.arrayBuffer();
    st.session = await ort.InferenceSession.create(buf, { executionProviders: ['wasm'], graphOptimizationLevel: 'all' });
    st.input = st.session.inputNames[0];
    st.backend = `phone CPU, ${threads} thread${threads > 1 ? 's' : ''}`;
    // warm-up so the first real frame isn't slow
    await st.session.run({ [st.input]: new ort.Tensor('float32', new Float32Array(3 * SIZE * SIZE), [1, 3, SIZE, SIZE]) });
  }

  // ------------------------------------------------------------------ detection on one frame
  const pre = document.createElement('canvas'); pre.width = pre.height = SIZE;
  const pctx = pre.getContext('2d', { willReadFrequently: true });
  const chw = new Float32Array(3 * SIZE * SIZE);

  function toTensor(f) {
    const [s, px, py, nw, nh] = V.letterboxParams(f.w, f.h, SIZE);
    pctx.fillStyle = 'rgb(114,114,114)';
    pctx.fillRect(0, 0, SIZE, SIZE);
    pctx.drawImage(f.img, 0, 0, f.w, f.h, px, py, nw, nh);
    const d = pctx.getImageData(0, 0, SIZE, SIZE).data, n = SIZE * SIZE;
    for (let i = 0, j = 0; i < n; i++, j += 4) {
      chw[i] = d[j] / 255; chw[n + i] = d[j + 1] / 255; chw[2 * n + i] = d[j + 2] / 255;
    }
    return new ort.Tensor('float32', chw, [1, 3, SIZE, SIZE]);
  }

  async function detect(f) {
    const t0 = performance.now();
    const out = (await st.session.run({ [st.input]: toTensor(f) }))[st.session.outputNames[0]];
    const n = out.dims[2];
    const boxes = V.decodeYolo(out.data, n, f.w, f.h, SIZE, +$('conf').value, 0);
    st.ms = performance.now() - t0;
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
      const label = `${p.bearing_deg > 0 ? '+' : ''}${Math.round(p.bearing_deg)}°  ~${p.estimate_m ? p.estimate_m.toFixed(1) : '?'} m${p.approaching ? '  >>' : ''}`;
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
      st.src = st.srcKind === 'robot' ? new RobotCamera(robot.base, robot.token) : new PhoneCamera(st.facing);
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
    $('m-be').textContent = st.backend;
    $('run').disabled = false;
    msg('Ready. Press Start.');
    $('dec-r').textContent = 'Press Start';
  }).catch(e => { msg(e.message || String(e)); $('m-be').textContent = 'not loaded'; });

  window.__detector = { st, detect, start, stop, pick };       // for tests
})();
