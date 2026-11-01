/* Peak test verdict: one sample per second -> checks + PASS / WARN / FAIL. Pure: test/test_peak_check.py runs it in Node.
 *
 * sample = {t (s from start), det_n (detections finished in that second), det_ms (mean ms), face_fps,
 *           temp_c, battery_pct, charging, thermal (Android 0 none .. 6 shutdown, -1 unknown), app_mb, long_ms}
 * Targets (what the head phone needs on the robot):
 *   detection        >= 5 per second over the last minute (the Pi camera sends ~5 pictures/s)
 *   slow-down        last-minute time per detection <= 1.25 x the first minute's (heat throttling shows here)
 *   face             >= 45 frames/s on average (a stuttering face looks broken)
 *   temperature      <= 42 C peak (warn to 46): a hot phone throttles and ages its battery
 *   thermal state    Android says none/light (warn at moderate)
 *   battery          <= 25 %/hour while not charging (warn to 40): the head needs its own supply anyway
 *   page stalls      <= 1 s per minute of the page frozen > 50 ms at a time
 */
(function (root) {
  'use strict';
  const mean = a => a.length ? a.reduce((x, y) => x + y, 0) / a.length : null;
  const r1 = x => x == null ? null : Math.round(x * 10) / 10;

  function evaluate(samples, opts) {
    const o = Object.assign({ rate: 0 }, opts || {});
    const S = samples.filter(s => s.t >= 10);                 // first 10 s: loading, warming up
    const dur = samples.length ? samples[samples.length - 1].t : 0;
    const win = Math.min(60, Math.max(10, Math.floor(dur / 3)));
    const first = S.filter(s => s.t < 10 + win), last = S.filter(s => s.t > dur - win);
    const checks = [];
    const add = (name, target, value, status, note) => checks.push({ name, target, value, status, note: note || '' });
    const grade = (v, okMax, warnMax) => v <= okMax ? 'ok' : v <= warnMax ? 'warn' : 'bad';

    const detOn = samples.some(s => s.det_n != null);
    if (detOn) {
      const fps = mean(last.map(s => s.det_n || 0)), need = o.rate ? o.rate * 0.9 : 5;
      add('Detection keeps up', `≥ ${need} per second`, fps == null ? 'no data' : `${r1(fps)} per second`,
          fps == null ? 'bad' : fps >= need ? 'ok' : fps >= 3 ? 'warn' : 'bad');
      const ms = arr => mean(arr.filter(s => s.det_ms != null && s.det_n > 0).map(s => s.det_ms));
      const a = ms(first), b = ms(last);
      if (a && b) {
        const ratio = b / a;
        add('Slows down when hot', '≤ 1.25 × first minute', `${Math.round(a)} → ${Math.round(b)} ms (× ${ratio.toFixed(2)})`,
            grade(ratio, 1.25, 1.6));
      } else add('Slows down when hot', '≤ 1.25 × first minute', 'no data', 'na');
    }
    const face = S.filter(s => s.face_fps != null).map(s => s.face_fps);
    if (face.length) {
      const f = mean(face), low = face.filter(x => x < 30).length;
      add('Face smoothness', '≥ 45 frames/s', `${Math.round(f)} frames/s${low ? ` (${low} s below 30)` : ''}`,
          f >= 45 ? 'ok' : f >= 30 ? 'warn' : 'bad');
    }
    const temps = samples.filter(s => s.temp_c != null).map(s => s.temp_c);
    if (temps.length) {
      const mx = Math.max(...temps);
      add('Phone temperature', '≤ 42 °C peak', `${r1(temps[0])} → ${r1(mx)} °C peak`, grade(mx, 42, 46));
    } else add('Phone temperature', '≤ 42 °C peak', 'not available (Pathik app only)', 'na');
    const th = samples.filter(s => s.thermal != null && s.thermal >= 0).map(s => s.thermal);
    if (th.length) {
      const mx = Math.max(...th), names = ['none', 'light', 'moderate', 'severe', 'critical', 'emergency', 'shutdown'];
      add('Android heat state', 'none or light', names[mx] || String(mx), mx <= 1 ? 'ok' : mx === 2 ? 'warn' : 'bad');
    }
    const bat = samples.filter(s => s.battery_pct != null);
    if (bat.length >= 2) {
      const charging = bat.some(s => s.charging), mins = (bat[bat.length - 1].t - bat[0].t) / 60;
      const drop = bat[0].battery_pct - bat[bat.length - 1].battery_pct;
      if (charging) add('Battery use', '≤ 25 % per hour', 'charging during the test', 'na');
      else if (mins < 3) add('Battery use', '≤ 25 % per hour', `${r1(drop)} % in ${r1(mins)} min (run 5+ min to judge)`, 'na');
      else {
        const perH = drop / mins * 60;
        add('Battery use', '≤ 25 % per hour', `${r1(drop)} % in ${Math.round(mins)} min ≈ ${Math.round(perH)} %/h`, grade(perH, 25, 40));
      }
    }
    const stalls = S.filter(s => s.long_ms != null);
    if (stalls.length) {
      const perMin = mean(stalls.map(s => s.long_ms)) * 60;
      add('Page stalls', '≤ 1 s per minute', `${(perMin / 1000).toFixed(1)} s per minute`, grade(perMin, 1000, 3000));
    }
    const graded = checks.filter(c => c.status !== 'na');
    const verdict = !graded.length ? 'WARN' : graded.some(c => c.status === 'bad') ? 'FAIL'
      : graded.some(c => c.status === 'warn') ? 'WARN' : 'PASS';
    const fpsAll = mean(last.map(s => s.det_n || 0)), msAll = mean(last.filter(s => s.det_n > 0).map(s => s.det_ms));
    const parts = [];
    if (detOn && fpsAll != null) parts.push(`${r1(fpsAll)} detections/s at ${Math.round(msAll || 0)} ms`);
    if (face.length) parts.push(`face ${Math.round(mean(face))} fps`);
    if (temps.length) parts.push(`peak ${r1(Math.max(...temps))} °C`);
    const line = `${Math.round(dur / 60 * 10) / 10} min: ` + (parts.join(', ') || 'no measurements');
    return { verdict, checks, line };
  }

  const api = { evaluate };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.PathikPeak = api;
})(typeof window !== 'undefined' ? window : globalThis);
