/* ONNX Runtime Web in a worker: YOLO runs off the page's thread, so the face and the page stay smooth while it
 * works (in an Android WebView the web engine gets one thread; on the page it froze everything ~80 ms a frame).
 * Messages: {init: {model, wasmPaths}} -> {ready, input, output} | {run: Float32Array (1x3x320x320)} -> {out, dims, ms}
 */
/* global ort */
self.importScripts('vendor/ort/ort.wasm.min.js');
let session = null;
self.onmessage = async ({ data }) => {
  try {
    if (data.init) {
      ort.env.wasm.wasmPaths = data.init.wasmPaths;
      ort.env.wasm.numThreads = 1;
      const res = await fetch(data.init.model);
      if (!res.ok) throw new Error(`Model not found (${data.init.model}).`);
      session = await ort.InferenceSession.create(await res.arrayBuffer(), { executionProviders: ['wasm'], graphOptimizationLevel: 'all' });
      self.postMessage({ ready: true, input: session.inputNames[0], output: session.outputNames[0] });
    } else if (data.run) {
      const t0 = performance.now();
      const r = await session.run({ [session.inputNames[0]]: new ort.Tensor('float32', data.run, [1, 3, 320, 320]) });
      const o = r[session.outputNames[0]];
      const out = new Float32Array(o.data);
      self.postMessage({ id: data.id, out, dims: o.dims, ms: performance.now() - t0 }, [out.buffer]);
    }
  } catch (e) {
    self.postMessage({ id: data.id, error: e.message || String(e) });
  }
};
