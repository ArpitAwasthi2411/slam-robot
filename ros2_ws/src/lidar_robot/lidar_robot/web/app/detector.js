/* Shared by detect.html (lab) and head.html (the phone in the robot's head):
 *   PathikDetector.load()            YOLO11n 320 px. In the Pathik APK: NATIVE TensorFlow Lite on the phone's GPU or
 *                                    all its big CPU cores (NativeDetector.java, the faster one is picked at start).
 *                                    Elsewhere / if that fails: ONNX Runtime Web (WebAssembly; one core in a WebView).
 *   PathikDetector.run(frame, conf)  -> {boxes: [[x1,y1,x2,y2,conf]] in frame pixels, ms}
 *   PhoneCamera / RobotCamera        picture sources: {img, w, h} per frame
 * Needs vision.js and vendor/ort/ort.wasm.min.js loaded first.
 */
(function (root) {
  'use strict';
  const V = root.PathikVision;
  const SIZE = 320;
  const MODEL = 'vendor/yolo11n_320.onnx';
  const st = { session: null, input: 'images', output: 'output0', backend: '', native: false, info: null };

  // ---- native engine (Android app): jobs go to Java, results come back through window.__pathikDetDone(id, json)
  const N = root.PathikNative;
  const jobs = new Map();
  let jobId = 1;
  root.__pathikDetDone = (id, json) => {
    const j = jobs.get(id); if (!j) return;
    jobs.delete(id);
    let r; try { r = JSON.parse(json); } catch (e) { r = { ok: false, error: 'bad reply' }; }
    if (r.ok) j.resolve(r); else j.reject(new Error(r.error || 'native detector failed'));
  };
  function nativeCall(fn, ...args) {
    return new Promise((resolve, reject) => {
      const id = jobId++;
      jobs.set(id, { resolve, reject });
      try { N[fn](id, ...args); } catch (e) { jobs.delete(id); reject(e); }
      setTimeout(() => { if (jobs.has(id)) { jobs.delete(id); reject(new Error('native detector timed out')); } }, fn === 'detectorInit' ? 30000 : 5000);
    });
  }
  async function loadNative() {
    if (!(N && N.hasNativeDetector && N.detect)) return false;
    try {
      const info = await nativeCall('detectorInit');
      st.native = true; st.info = info; st.backend = info.engine;
      return true;
    } catch (e) {
      st.nativeError = e.message;
      return false;
    }
  }

  // ---- web engine in a worker (detworker.js): the page's thread stays free for the face
  let worker = null, wjobs = new Map(), wid = 1;
  function loadWorker() {
    return new Promise((resolve, reject) => {
      let w;
      try { w = new Worker('detworker.js'); } catch (e) { reject(e); return; }
      const t = setTimeout(() => { w.terminate(); reject(new Error('worker timed out')); }, 60000);
      w.onmessage = ({ data }) => {
        if (data.ready) { clearTimeout(t); worker = w; resolve(); return; }
        if (data.error && data.id == null) { clearTimeout(t); w.terminate(); reject(new Error(data.error)); return; }
        const j = wjobs.get(data.id); if (!j) return;
        wjobs.delete(data.id);
        if (data.error) j.reject(new Error(data.error)); else j.resolve(data);
      };
      w.onerror = e => { clearTimeout(t); reject(new Error(e.message || 'worker failed')); };
      w.postMessage({ init: { model: new URL(MODEL, location.href).href, wasmPaths: new URL('vendor/ort/', location.href).href } });
    });
  }
  function workerRun(chwCopy) {
    return new Promise((resolve, reject) => {
      const id = wid++;
      wjobs.set(id, { resolve, reject });
      worker.postMessage({ id, run: chwCopy }, [chwCopy.buffer]);
    });
  }

  async function load() {
    if (st.session || st.native || worker) return st.backend;
    if (await loadNative()) return st.backend;
    if (root.Worker && !root.crossOriginIsolated) {
      try { await loadWorker(); st.backend = 'phone CPU, 1 thread (background)'; return st.backend; }
      catch (e) { st.workerError = e.message; }          // fall back to the page's own thread
    }
    if (!root.ort) throw new Error('Detector files are missing (vendor/ort). Use the Pathik app (APK).');
    const threads = root.crossOriginIsolated ? Math.max(1, Math.min(4, (navigator.hardwareConcurrency || 2) - 1)) : 1;
    ort.env.wasm.wasmPaths = new URL('vendor/ort/', location.href).href;
    ort.env.wasm.numThreads = threads;
    ort.env.wasm.proxy = false;
    const res = await fetch(MODEL);
    if (!res.ok) throw new Error(`Model not found (${MODEL}).`);
    st.session = await ort.InferenceSession.create(await res.arrayBuffer(),
      { executionProviders: ['wasm'], graphOptimizationLevel: 'all' });
    st.input = st.session.inputNames[0];
    st.output = st.session.outputNames[0];
    st.backend = `phone CPU, ${threads} thread${threads > 1 ? 's' : ''}`;
    await st.session.run({ [st.input]: new ort.Tensor('float32', new Float32Array(3 * SIZE * SIZE), [1, 3, SIZE, SIZE]) });
    return st.backend;
  }

  let pre = null, pctx = null;
  const chw = new Float32Array(3 * SIZE * SIZE);
  function letterbox(f) {
    if (!pre) {
      pre = document.createElement('canvas'); pre.width = pre.height = SIZE;
      pctx = pre.getContext('2d', { willReadFrequently: true });
    }
    const [, px, py, nw, nh] = V.letterboxParams(f.w, f.h, SIZE);
    pctx.fillStyle = 'rgb(114,114,114)';
    pctx.fillRect(0, 0, SIZE, SIZE);
    pctx.drawImage(f.img, 0, 0, f.w, f.h, px, py, nw, nh);
  }

  function toTensor(f, raw) {
    letterbox(f);
    const d = pctx.getImageData(0, 0, SIZE, SIZE).data, n = SIZE * SIZE;
    for (let i = 0, j = 0; i < n; i++, j += 4) {
      chw[i] = d[j] / 255; chw[n + i] = d[j + 1] / 255; chw[2 * n + i] = d[j + 2] / 255;
    }
    return raw ? chw : new ort.Tensor('float32', chw, [1, 3, SIZE, SIZE]);
  }

  async function run(f, conf = 0.45) {
    if (st.native) return runNative(f, conf);
    if (worker) {
      const t0 = performance.now();
      toTensor(f, true);
      const r = await workerRun(new Float32Array(chw));
      return { boxes: V.decodeYolo(r.out, r.dims[2], f.w, f.h, SIZE, conf, 0), ms: performance.now() - t0, inferMs: r.ms };
    }
    const t0 = performance.now();
    const out = (await st.session.run({ [st.input]: toTensor(f) }))[st.output];
    const boxes = V.decodeYolo(out.data, out.dims[2], f.w, f.h, SIZE, conf, 0);
    return { boxes, ms: performance.now() - t0 };
  }

  // the letterboxed 320 x 320 picture as a JPEG -> Java -> person candidates in 320 px -> frame pixels + NMS
  async function runNative(f, conf) {
    const t0 = performance.now();
    letterbox(f);
    const b64 = pre.toDataURL('image/jpeg', 0.92).slice('data:image/jpeg;base64,'.length);
    const r = await nativeCall('detect', b64, conf);
    const [s, px, py] = V.letterboxParams(f.w, f.h, SIZE);
    const boxes = r.boxes.map(b => [Math.max(0, (b[0] - px) / s), Math.max(0, (b[1] - py) / s),
                                    Math.min(f.w, (b[2] - px) / s), Math.min(f.h, (b[3] - py) / s), b[4]])
      .filter(b => b[2] > b[0] && b[3] > b[1]);
    return { boxes: V.nms(boxes), ms: performance.now() - t0, inferMs: r.infer_ms };
  }

  class PhoneCamera {
    constructor(facing = 'environment') { this.facing = facing; this.video = null; this.stream = null; }
    async start() {
      if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
        throw new Error(root.isSecureContext ? 'This browser has no camera access.'
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

  /** The Pi camera's picture. raw = the plain frame (no boxes drawn), as the head's detector needs. */
  class RobotCamera {
    constructor(base, token, raw = false) {
      this.base = (base || '').replace(/\/$/, ''); this.token = token; this.raw = raw; this.last = null;
    }
    async start() { if (!this.base) throw new Error('Not connected to a robot: open this from the Pathik app while connected.'); }
    async frame() {
      const r = await fetch(`${this.base}/api/camera.jpg?${this.raw ? 'raw=1&' : ''}t=${Date.now()}`,
        { headers: this.token ? { Authorization: 'Bearer ' + this.token } : {}, cache: 'no-store' });
      if (!r.ok) {
        throw new Error(r.status === 404 ? 'No camera picture from the robot yet (is the camera node running?)'
          : r.status === 401 || r.status === 403 ? 'The robot refused the login: open this again from the app'
            : `Robot camera: HTTP ${r.status}`);
      }
      const bmp = await createImageBitmap(await r.blob());
      if (this.last && this.last.close) this.last.close();
      this.last = bmp;
      return { img: bmp, w: bmp.width, h: bmp.height };
    }
    stop() { if (this.last && this.last.close) this.last.close(); this.last = null; }
  }

  root.PathikDetector = { load, run, PhoneCamera, RobotCamera, SIZE, get backend() { return st.backend; },
                          get native() { return st.native; }, get info() { return st.info; },
                          get nativeError() { return st.nativeError; } };
})(window);
