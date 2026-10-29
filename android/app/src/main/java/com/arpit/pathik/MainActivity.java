package com.arpit.pathik;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.Manifest;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.speech.RecognitionListener;
import android.speech.RecognizerIntent;
import android.speech.SpeechRecognizer;
import android.speech.tts.TextToSpeech;
import java.util.ArrayList;
import java.util.Locale;
import android.graphics.Color;
import android.os.Build;
import android.os.Bundle;
import android.os.VibrationEffect;
import android.os.Vibrator;
import android.view.View;
import android.view.WindowManager;
import android.webkit.JavascriptInterface;
import android.webkit.WebChromeClient;
import android.webkit.WebResourceRequest;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;

import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Set;

/** Pathik: a full-screen WebView around the app bundled in assets/app, plus a few native helpers. */
public class MainActivity extends Activity {
    private WebView web;
    private SpeechRecognizer recognizer;
    private TextToSpeech tts;
    private boolean ttsReady = false;
    private static final int REQ_MIC = 7;
    private String pendingLang = null;

    @SuppressLint("SetJavaScriptEnabled")
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        web = new WebView(this);
        web.setBackgroundColor(Color.parseColor("#0d0d10"));
        web.setOverScrollMode(View.OVER_SCROLL_NEVER);
        WebSettings s = web.getSettings();
        s.setJavaScriptEnabled(true);
        s.setDomStorageEnabled(true);          // remembered robots, speed setting
        s.setAllowFileAccess(true);            // the app itself is file:///android_asset/app/
        s.setCacheMode(WebSettings.LOAD_NO_CACHE);
        s.setMediaPlaybackRequiresUserGesture(true);
        // follow the phone's font size (accessibility), capped at 130 % so the map screens still fit
        float fontScale = getResources().getConfiguration().fontScale;
        s.setTextZoom(Math.round(Math.max(1f, Math.min(1.3f, fontScale)) * 100));
        web.setWebChromeClient(new WebChromeClient());
        web.setWebViewClient(new WebViewClient() {
            @Override
            public boolean shouldOverrideUrlLoading(WebView view, WebResourceRequest request) {
                return !request.getUrl().toString().startsWith("file:///android_asset/");
            }
        });
        web.addJavascriptInterface(new Bridge(this), "PathikNative");
        setContentView(web);
        tts = new TextToSpeech(this, status -> {
            if (status == TextToSpeech.SUCCESS) {
                tts.setLanguage(new Locale("en", "IN"));
                ttsReady = true;
            }
        });
        if (savedInstanceState != null) web.restoreState(savedInstanceState);
        else web.loadUrl("file:///android_asset/app/index.html");
    }

    @Override
    protected void onSaveInstanceState(Bundle out) {
        super.onSaveInstanceState(out);
        web.saveState(out);
    }

    @Override
    protected void onResume() { super.onResume(); web.onResume(); }

    @Override
    protected void onPause() { web.onPause(); super.onPause(); }

    @Override
    public void onBackPressed() {
        // the app closes a dialog / sub-page / goes Home first; only when it has nothing to close do we leave
        // (to the background: keep the connection, don't tear the app down)
        web.evaluateJavascript("window.__pathikBack ? String(window.__pathikBack()) : 'false'",
                v -> { if (!"\"true\"".equals(v) && !"true".equals(v)) moveTaskToBack(true); });
    }

    @Override
    protected void onDestroy() {
        if (recognizer != null) recognizer.destroy();
        if (tts != null) tts.shutdown();
        super.onDestroy();
    }

    // ---------------------------------------------------------------- voice
    /** Send a voice event to the page: window.__pathikVoice(kind, text). */
    void voiceEvent(String kind, String text) {
        String safe = text == null ? "" : text.replace("\\", "\\\\").replace("'", "\\'").replace("\n", " ");
        runOnUiThread(() -> web.evaluateJavascript("window.__pathikVoice && window.__pathikVoice('" + kind + "','" + safe + "')", null));
    }

    void startListening(String lang) {
        if (checkSelfPermission(Manifest.permission.RECORD_AUDIO) != PackageManager.PERMISSION_GRANTED) {
            pendingLang = lang;
            requestPermissions(new String[]{Manifest.permission.RECORD_AUDIO}, REQ_MIC);
            return;
        }
        if (!SpeechRecognizer.isRecognitionAvailable(this)) { voiceEvent("error", "No speech recognizer on this phone (install Google app)"); return; }
        if (recognizer == null) {
            recognizer = SpeechRecognizer.createSpeechRecognizer(this);
            recognizer.setRecognitionListener(new RecognitionListener() {
                public void onReadyForSpeech(Bundle b) { voiceEvent("ready", ""); }
                public void onBeginningOfSpeech() { }
                public void onRmsChanged(float rms) { voiceEvent("level", String.valueOf(rms)); }
                public void onBufferReceived(byte[] buf) { }
                public void onEndOfSpeech() { voiceEvent("end", ""); }
                public void onError(int err) {
                    String m = err == SpeechRecognizer.ERROR_NO_MATCH || err == SpeechRecognizer.ERROR_SPEECH_TIMEOUT
                        ? "Didn't catch that. Try again." : err == SpeechRecognizer.ERROR_NETWORK || err == SpeechRecognizer.ERROR_NETWORK_TIMEOUT
                        ? "Speech recognition needs internet (or the offline English pack)" : "Voice error " + err;
                    voiceEvent("error", m);
                }
                public void onResults(Bundle r) {
                    ArrayList<String> l = r.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION);
                    voiceEvent("final", l != null && !l.isEmpty() ? l.get(0) : "");
                }
                public void onPartialResults(Bundle r) {
                    ArrayList<String> l = r.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION);
                    if (l != null && !l.isEmpty()) voiceEvent("partial", l.get(0));
                }
                public void onEvent(int t, Bundle b) { }
            });
        }
        Intent i = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
        i.putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL, RecognizerIntent.LANGUAGE_MODEL_FREE_FORM);
        i.putExtra(RecognizerIntent.EXTRA_LANGUAGE, lang == null || lang.isEmpty() ? "en-IN" : lang);
        i.putExtra(RecognizerIntent.EXTRA_PARTIAL_RESULTS, true);
        i.putExtra(RecognizerIntent.EXTRA_MAX_RESULTS, 1);
        recognizer.startListening(i);
    }

    @Override
    public void onRequestPermissionsResult(int code, String[] perms, int[] res) {
        super.onRequestPermissionsResult(code, perms, res);
        if (code == REQ_MIC) {
            if (res.length > 0 && res[0] == PackageManager.PERMISSION_GRANTED) startListening(pendingLang);
            else voiceEvent("error", "Microphone permission is needed for voice commands");
        }
    }

    /** Methods callable from JavaScript as window.PathikNative.* */
    static class Bridge {
        private final MainActivity act;
        Bridge(MainActivity a) { act = a; }

        /** JSON list of this phone's IPv4 /24 networks, e.g. ["192.168.43","10.42.0"] (Wi-Fi and hotspot). */
        @JavascriptInterface
        public String subnets() {
            Set<String> nets = new LinkedHashSet<>();
            try {
                for (NetworkInterface ni : Collections.list(NetworkInterface.getNetworkInterfaces())) {
                    if (!ni.isUp() || ni.isLoopback()) continue;
                    String name = ni.getName();
                    if (name.startsWith("rmnet") || name.startsWith("ccmni") || name.startsWith("dummy")) continue;  // mobile data
                    for (InetAddress a : Collections.list(ni.getInetAddresses())) {
                        if (a instanceof Inet4Address && a.isSiteLocalAddress()) {
                            String ip = a.getHostAddress();
                            nets.add(ip.substring(0, ip.lastIndexOf('.')));
                        }
                    }
                }
            } catch (Exception ignored) { }
            StringBuilder sb = new StringBuilder("[");
            for (String n : nets) { if (sb.length() > 1) sb.append(','); sb.append('"').append(n).append('"'); }
            return sb.append(']').toString();
        }

        @JavascriptInterface
        public void haptic(int ms) {
            Vibrator v = (Vibrator) act.getSystemService(Context.VIBRATOR_SERVICE);
            if (v == null || !v.hasVibrator()) return;
            int d = Math.max(5, Math.min(ms, 200));
            if (Build.VERSION.SDK_INT >= 26) v.vibrate(VibrationEffect.createOneShot(d, VibrationEffect.DEFAULT_AMPLITUDE));
        }

        /** Keep the screen on while connected (driving with a dimmed screen is unsafe). */
        @JavascriptInterface
        public void keepAwake(final boolean on) {
            act.runOnUiThread(() -> {
                if (on) act.getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
                else act.getWindow().clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
            });
        }

        /** Voice commands: results arrive in window.__pathikVoice(kind, text). */
        @JavascriptInterface
        public void listen(final String lang) { act.runOnUiThread(() -> act.startListening(lang)); }

        @JavascriptInterface
        public void stopListening() { act.runOnUiThread(() -> { if (act.recognizer != null) act.recognizer.stopListening(); }); }

        @JavascriptInterface
        public boolean canListen() { return true; }

        /** Speak a short reply through the phone. */
        @JavascriptInterface
        public void speak(String text) {
            if (act.ttsReady && text != null) act.tts.speak(text, TextToSpeech.QUEUE_FLUSH, null, "pathik");
        }

        /** Share text (e.g. the mission log as CSV) to Drive, WhatsApp, email... */
        @JavascriptInterface
        public void share(final String name, final String text) {
            act.runOnUiThread(() -> {
                Intent i = new Intent(Intent.ACTION_SEND);
                i.setType("text/plain");
                i.putExtra(Intent.EXTRA_SUBJECT, name);
                i.putExtra(Intent.EXTRA_TITLE, name);
                i.putExtra(Intent.EXTRA_TEXT, text);
                act.startActivity(Intent.createChooser(i, "Share " + name));
            });
        }

        @JavascriptInterface
        public String version() {
            try { return act.getPackageManager().getPackageInfo(act.getPackageName(), 0).versionName; }
            catch (Exception e) { return "?"; }
        }
    }
}
