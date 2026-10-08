package com.arpit.pathik;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.content.Context;
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

    @SuppressLint("SetJavaScriptEnabled")
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        web = new WebView(this);
        web.setBackgroundColor(Color.parseColor("#221628"));
        web.setOverScrollMode(View.OVER_SCROLL_NEVER);
        WebSettings s = web.getSettings();
        s.setJavaScriptEnabled(true);
        s.setDomStorageEnabled(true);          // remembered robots, speed setting
        s.setAllowFileAccess(true);            // the app itself is file:///android_asset/app/
        s.setCacheMode(WebSettings.LOAD_NO_CACHE);
        s.setMediaPlaybackRequiresUserGesture(true);
        s.setTextZoom(100);                    // the layout is designed for 100 %; ignore system font scaling
        web.setWebChromeClient(new WebChromeClient());
        web.setWebViewClient(new WebViewClient() {
            @Override
            public boolean shouldOverrideUrlLoading(WebView view, WebResourceRequest request) {
                return !request.getUrl().toString().startsWith("file:///android_asset/");
            }
        });
        web.addJavascriptInterface(new Bridge(this), "PathikNative");
        setContentView(web);
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
    public void onBackPressed() { moveTaskToBack(true); }   // keep the connection; don't tear the app down

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

        @JavascriptInterface
        public String version() {
            try { return act.getPackageManager().getPackageInfo(act.getPackageName(), 0).versionName; }
            catch (Exception e) { return "?"; }
        }
    }
}
