package com.arpit.pathik;

import android.content.Context;
import android.content.res.AssetFileDescriptor;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.SystemClock;
import android.util.Base64;

import org.tensorflow.lite.Interpreter;
import org.tensorflow.lite.gpu.CompatibilityList;
import org.tensorflow.lite.gpu.GpuDelegate;

import java.io.FileInputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;
import java.util.Locale;

/**
 * People detection with YOLO11n (320 px) in TensorFlow Lite, on the phone's GPU or on all its big CPU cores.
 *
 * The web page letterboxes the picture to 320 x 320 and sends it as a JPEG; this returns every person candidate
 * above the confidence as [x1, y1, x2, y2, conf] in those 320 px (the page maps them back to the frame and runs
 * the same NMS as vision.js). At start it times the GPU and the CPU engines and keeps the faster one.
 */
final class NativeDetector {
    static final String MODEL = "app/vendor/yolo11n_320.tflite";
    static final int SIZE = 320;

    private Interpreter interp;
    private GpuDelegate gpu;
    private String engine = "none";
    private double startGpu = -1, startCpu = -1;   // the start test, kept for later pages (init runs once per app)
    private int outA, outB;               // output shape [1, outA, outB]: [1, 84, 2100] or [1, 2100, 84]
    private float[][][] out;
    private final ByteBuffer input = ByteBuffer.allocateDirect(SIZE * SIZE * 3 * 4).order(ByteOrder.nativeOrder());
    private final int[] pixels = new int[SIZE * SIZE];

    /** Load the model, pick the fastest engine. Returns a JSON string {ok, engine, ms_gpu, ms_cpu, error}. */
    synchronized String init(Context ctx) {
        if (interp != null) return info(null, startGpu, startCpu);
        MappedByteBuffer model;
        try {
            AssetFileDescriptor fd = ctx.getAssets().openFd(MODEL);
            FileInputStream in = new FileInputStream(fd.getFileDescriptor());
            model = in.getChannel().map(FileChannel.MapMode.READ_ONLY, fd.getStartOffset(), fd.getDeclaredLength());
        } catch (Exception e) {
            return "{\"ok\":false,\"error\":\"no native model: " + esc(e.getMessage()) + "\"}";
        }
        int threads = Math.max(1, Math.min(4, Runtime.getRuntime().availableProcessors() - 1));
        double msGpu = -1, msCpu = -1;
        Interpreter gpuInterp = null, cpuInterp = null;
        GpuDelegate gd = null;
        try {
            CompatibilityList cl = new CompatibilityList();
            if (cl.isDelegateSupportedOnThisDevice()) {
                gd = new GpuDelegate(cl.getBestOptionsForThisDevice());
                Interpreter.Options o = new Interpreter.Options().addDelegate(gd);
                gpuInterp = new Interpreter(model, o);
                msGpu = bench(gpuInterp);
            }
        } catch (Throwable t) {                       // GPU missing / op not supported: CPU only
            if (gpuInterp != null) gpuInterp.close();
            if (gd != null) gd.close();
            gpuInterp = null; gd = null; msGpu = -1;
        }
        try {
            Interpreter.Options o = new Interpreter.Options().setNumThreads(threads).setUseXNNPACK(true);
            cpuInterp = new Interpreter(model, o);
            msCpu = bench(cpuInterp);
        } catch (Throwable t) {
            if (cpuInterp != null) cpuInterp.close();
            cpuInterp = null; msCpu = -1;
        }
        if (gpuInterp != null && (cpuInterp == null || msGpu <= msCpu)) {
            interp = gpuInterp; gpu = gd; engine = "phone GPU";
            if (cpuInterp != null) cpuInterp.close();
        } else if (cpuInterp != null) {
            interp = cpuInterp; engine = "phone CPU × " + threads + " (native)";
            if (gpuInterp != null) gpuInterp.close();
            if (gd != null) gd.close();
        } else {
            return "{\"ok\":false,\"error\":\"TensorFlow Lite could not start\"}";
        }
        shape(interp);
        startGpu = msGpu; startCpu = msCpu;
        return info(null, msGpu, msCpu);
    }

    private void shape(Interpreter it) {
        int[] s = it.getOutputTensor(0).shape();
        outA = s[1]; outB = s[2];
        out = new float[1][outA][outB];
    }

    private double bench(Interpreter it) {
        shape(it);
        input.rewind();
        for (int i = 0; i < SIZE * SIZE * 3; i++) input.putFloat(0.45f);
        double best = 1e9;
        for (int k = 0; k < 4; k++) {                 // first run builds the GPU program: ignore it
            input.rewind();
            long t0 = SystemClock.elapsedRealtimeNanos();
            it.run(input, out);
            double ms = (SystemClock.elapsedRealtimeNanos() - t0) / 1e6;
            if (k > 0) best = Math.min(best, ms);
        }
        return best;
    }

    private String info(String err, double msGpu, double msCpu) {
        return String.format(Locale.US, "{\"ok\":%s,\"engine\":\"%s\",\"ms_gpu\":%.1f,\"ms_cpu\":%.1f%s}",
                err == null ? "true" : "false", esc(engine), msGpu, msCpu,
                err == null ? "" : ",\"error\":\"" + esc(err) + "\"");
    }

    /** One 320 x 320 letterboxed JPEG (base64) -> {"ok":true,"ms":..,"engine":..,"boxes":[[x1,y1,x2,y2,c],..]}. */
    synchronized String detect(String jpegB64, float conf) {
        if (interp == null) return "{\"ok\":false,\"error\":\"detector not started\"}";
        try {
            long t0 = SystemClock.elapsedRealtimeNanos();
            byte[] jpg = Base64.decode(jpegB64, Base64.DEFAULT);
            Bitmap bmp = BitmapFactory.decodeByteArray(jpg, 0, jpg.length);
            if (bmp == null) return "{\"ok\":false,\"error\":\"bad picture\"}";
            if (bmp.getWidth() != SIZE || bmp.getHeight() != SIZE) {
                Bitmap s = Bitmap.createScaledBitmap(bmp, SIZE, SIZE, true);
                bmp.recycle();
                bmp = s;
            }
            bmp.getPixels(pixels, 0, SIZE, 0, 0, SIZE, SIZE);
            bmp.recycle();
            input.rewind();
            FloatBuffer fb = input.asFloatBuffer();
            for (int p : pixels) {                    // NHWC, RGB, 0..1
                fb.put(((p >> 16) & 0xFF) / 255f);
                fb.put(((p >> 8) & 0xFF) / 255f);
                fb.put((p & 0xFF) / 255f);
            }
            input.rewind();
            long t1 = SystemClock.elapsedRealtimeNanos();
            interp.run(input, out);
            double inferMs = (SystemClock.elapsedRealtimeNanos() - t1) / 1e6;
            StringBuilder sb = new StringBuilder(256);
            boolean first = true;
            boolean chFirst = outA < outB;            // [84, N] (ultralytics) or [N, 84]
            int n = chFirst ? outB : outA;
            float[][] o = out[0];
            // onnx2tf keeps pixel boxes; ultralytics' own TFLite export gives 0..1: decided per frame from all centres
            float maxC = 0;
            for (int i = 0; i < n; i++) maxC = Math.max(maxC, chFirst ? o[0][i] : o[i][0]);
            boolean normalized = maxC <= 1.5f;
            for (int i = 0; i < n; i++) {
                float sc = chFirst ? o[4][i] : o[i][4];
                if (sc < conf) continue;
                float cx = chFirst ? o[0][i] : o[i][0], cy = chFirst ? o[1][i] : o[i][1];
                float w = chFirst ? o[2][i] : o[i][2], h = chFirst ? o[3][i] : o[i][3];
                if (normalized) { cx *= SIZE; cy *= SIZE; w *= SIZE; h *= SIZE; }
                float x1 = Math.max(0, cx - w / 2), y1 = Math.max(0, cy - h / 2);
                float x2 = Math.min(SIZE, cx + w / 2), y2 = Math.min(SIZE, cy + h / 2);
                if (x2 <= x1 || y2 <= y1) continue;
                if (!first) sb.append(',');
                first = false;
                sb.append(String.format(Locale.US, "[%.2f,%.2f,%.2f,%.2f,%.3f]", x1, y1, x2, y2, sc));
            }
            double totalMs = (SystemClock.elapsedRealtimeNanos() - t0) / 1e6;
            return String.format(Locale.US, "{\"ok\":true,\"engine\":\"%s\",\"ms\":%.1f,\"infer_ms\":%.1f,\"boxes\":[%s]}",
                    esc(engine), totalMs, inferMs, sb);
        } catch (Throwable t) {
            return "{\"ok\":false,\"error\":\"" + esc(String.valueOf(t.getMessage())) + "\"}";
        }
    }

    synchronized void close() {
        if (interp != null) interp.close();
        if (gpu != null) gpu.close();
        interp = null; gpu = null;
    }

    static String esc(String s) {
        return s == null ? "" : s.replace("\\", "\\\\").replace("\"", "'").replace("\n", " ");
    }

}
