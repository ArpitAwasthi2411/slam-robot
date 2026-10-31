/* Pathik's face (head mode): which mood to show for the robot's state, and drawing the eyes.
 *
 *   faceFor(st, now, opts) -> {mood, line, sub, ok?, ask?}   pure: robot state (/api/state) -> what the face shows
 *   Face(svg)                                                draws a mood (eyes, lids, brows, mouth) + blinking
 *
 * test/test_head_face.py runs faceFor in Node on the robot's real states.
 */
(function (root) {
  'use strict';
  const MOODS = ['idle', 'sleepy', 'thinking', 'driving', 'blocked', 'happy', 'help', 'charging', 'lowbat',
                 'stopped', 'offline', 'lost'];
  const SLEEPY_AFTER_S = 120;

  function goalName(n) { return (n && n.goal && n.goal.label) || 'my next stop'; }
  function fmtDist(n) {
    if (!n || n.distance == null) return '';
    const m = n.distance, eta = n.eta_s;
    return `${m < 10 ? m.toFixed(1) : Math.round(m)} m to go` + (eta ? ` · about ${eta < 90 ? Math.round(eta) + ' s' : Math.round(eta / 60) + ' min'}` : '');
  }

  /** st: /api/state (or null when the robot can't be reached). opts.idleS: seconds since the robot was last busy. */
  function faceFor(st, opts) {
    const o = opts || {};
    if (!st) return { mood: 'offline', line: "I can't reach the robot", sub: 'Check the hotspot / Wi-Fi' };
    const n = st.nav || {}, bat = n.battery || {}, v = st.vision || {};
    const state = n.state || 'IDLE';
    if (st.estop || n.estop) return { mood: 'stopped', line: 'Stopped', sub: 'Someone pressed STOP. The robot team can release it.' };
    const rec = n.recovery;
    if (rec && rec.status === 'asking' && rec.ask) {
      return { mood: 'help', line: rec.ask.question || 'I need help', sub: 'Choose an answer below', ask: rec.ask, recId: rec.id };
    }
    if (n.localized === false) return { mood: 'lost', line: 'Where am I?', sub: 'The robot team needs to confirm my position.' };
    if (state === 'CONFIRM' && n.confirm) {
      return { mood: 'happy', line: `We're at ${n.confirm.label}!`, sub: 'Press OK when you are done', ok: true };
    }
    const moving = ['PLANNING', 'DRIVING', 'WAITING', 'SCANNING', 'DOCKING', 'UNDOCKING', 'RECOVERING', 'DWELL'].includes(state);
    if (moving && v.action === 'STOP') return { mood: 'blocked', line: 'Excuse me, may I pass?', sub: v.reason || '' };
    if (moving && n.blocked) return { mood: 'blocked', line: 'Excuse me, may I pass?', sub: 'Something is in my way' };
    if (state === 'RECOVERING' || state === 'PLANNING') {
      return { mood: 'thinking', line: state === 'PLANNING' ? `Finding the way to ${goalName(n)}…` : 'Let me think…', sub: n.message || '' };
    }
    if (state === 'DWELL') return { mood: 'happy', line: `We're at ${goalName(n)}!`, sub: n.message || '' };
    if (state === 'DOCKING') return { mood: 'charging', line: 'Docking…', sub: n.message || '' };
    if (state === 'CHARGING' || bat.charging) {
      return { mood: 'charging', line: 'Charging' + (bat.pct != null ? ` · ${bat.pct} %` : '…'), sub: 'Tap me if you need me' };
    }
    if (state === 'DRIVING' || state === 'UNDOCKING' || state === 'SCANNING' || state === 'WAITING') {
      const line = state === 'SCANNING' ? 'Looking around…' : state === 'UNDOCKING' ? 'Leaving the dock' : `Going to ${goalName(n)}`;
      return { mood: v.action === 'SLOW_DOWN' ? 'blocked' : 'driving', line, sub: fmtDist(n) || n.message || '' };
    }
    if (bat.level === 'low' || bat.level === 'critical') {
      return { mood: 'lowbat', line: 'My battery is low', sub: bat.pct != null ? `${bat.pct} %: I'll go and charge soon` : '' };
    }
    if ((o.idleS || 0) > SLEEPY_AFTER_S) return { mood: 'sleepy', line: 'Resting…', sub: 'Tap me to choose where to go' };
    return { mood: 'idle', line: 'Hello! Where do you want to go?', sub: 'Tap me to choose a place' };
  }

  // ------------------------------------------------------------------ drawing
  // per mood: eye height (0..1 open), pupil offset (dx, dy), brow angle (deg, + = worried), mouth, colour
  const LOOK = {
    idle:     { open: 1.0,  px: 0,   py: 0,   brow: 0,   mouth: 'smile',  col: '--teal' },
    sleepy:   { open: 0.18, px: 0,   py: 6,   brow: 0,   mouth: 'flat',   col: '--dim' },
    thinking: { open: 0.85, px: 14,  py: -14, brow: -6,  mouth: 'flat',   col: '--orange' },
    driving:  { open: 0.9,  px: 0,   py: 0,   brow: -4,  mouth: 'smile',  col: '--teal' },
    blocked:  { open: 1.15, px: 0,   py: 0,   brow: 10,  mouth: 'o',      col: '--orange' },
    happy:    { open: 0.35, px: 0,   py: -4,  brow: 0,   mouth: 'big',    col: '--good', arcs: true },
    help:     { open: 1.0,  px: 0,   py: 4,   brow: 14,  mouth: 'frown',  col: '--warn' },
    charging: { open: 0.3,  px: 0,   py: 4,   brow: 0,   mouth: 'smile',  col: '--good' },
    lowbat:   { open: 0.5,  px: 0,   py: 6,   brow: 8,   mouth: 'frown',  col: '--warn' },
    stopped:  { open: 1.0,  px: 0,   py: 0,   brow: 0,   mouth: 'flat',   col: '--red', cross: true },
    offline:  { open: 0.6,  px: 0,   py: 0,   brow: 0,   mouth: 'flat',   col: '--dim-2' },
    lost:     { open: 1.0,  px: -14, py: 0,   brow: 12,  mouth: 'wavy',   col: '--warn' },
  };
  const MOUTH = {
    smile: 'M -46 0 Q 0 30 46 0', big: 'M -56 -4 Q 0 50 56 -4 Z', flat: 'M -36 6 L 36 6',
    frown: 'M -40 18 Q 0 -8 40 18', o: 'M -14 6 a 14 16 0 1 0 28 0 a 14 16 0 1 0 -28 0', wavy: 'M -44 8 q 11 -12 22 0 t 22 0 t 22 0 t 22 0',
  };

  class Face {
    constructor(svg) {
      this.svg = svg; this.mood = null; this.blinkUntil = 0; this.nextBlink = performance.now() + 2500;
      const ns = 'http://www.w3.org/2000/svg', el = (t, a, p = svg) => {
        const e = document.createElementNS(ns, t); for (const k in a) e.setAttribute(k, a[k]); p.appendChild(e); return e; };
      svg.setAttribute('viewBox', '-300 -200 600 400');
      this.eyes = [-120, 120].map(x => {
        const g = el('g', { transform: `translate(${x} -40)` });
        return { g, x, white: el('rect', { x: -62, y: -70, width: 124, height: 140, rx: 56, class: 'f-eye' }, g),
                 pupil: el('circle', { r: 30, class: 'f-pupil' }, g),
                 shine: el('circle', { r: 9, class: 'f-shine' }, g),
                 arc: el('path', { d: 'M -52 18 Q 0 -46 52 18', class: 'f-arc' }, g),
                 cross: el('path', { d: 'M -40 -40 L 40 40 M 40 -40 L -40 40', class: 'f-cross' }, g),
                 brow: el('rect', { x: -60, y: -112, width: 120, height: 16, rx: 8, class: 'f-brow' }, g) };
      });
      this.mouth = el('path', { d: MOUTH.smile, class: 'f-mouth', transform: 'translate(0 120)' });
      this.zz = el('text', { x: 200, y: -120, class: 'f-zz' }); this.zz.textContent = 'z z';
      this.tick = this.tick.bind(this);
      requestAnimationFrame(this.tick);
    }

    set(mood) {
      if (mood === this.mood || !LOOK[mood]) return;
      this.mood = mood;
      this.svg.setAttribute('data-mood', mood);
      const L = LOOK[mood];
      this.svg.style.setProperty('--face', `var(${L.col})`);
      this.mouth.setAttribute('d', MOUTH[L.mouth]);
      this.zz.style.opacity = mood === 'sleepy' ? 1 : 0;
      this.eyes.forEach((e, i) => {
        const side = i === 0 ? -1 : 1;
        e.brow.setAttribute('transform', `rotate(${side * L.brow})`);
        e.arc.style.opacity = L.arcs ? 1 : 0;
        e.cross.style.opacity = L.cross ? 1 : 0;
      });
      this.draw(performance.now());
    }

    draw(now) {
      const L = LOOK[this.mood] || LOOK.idle;
      const blink = now < this.blinkUntil ? 0.08 : 1;
      const open = Math.max(0.06, Math.min(1.2, L.open * blink));
      const wob = this.mood === 'thinking' ? Math.sin(now / 400) * 6 : this.mood === 'lost' ? Math.sin(now / 300) * 18 : 0;
      this.eyes.forEach(e => {
        const h = 140 * open;
        const hide = L.arcs || L.cross;
        e.white.setAttribute('y', -h / 2); e.white.setAttribute('height', h); e.white.style.opacity = hide ? 0 : 1;
        const py = Math.max(-h / 2 + 26, Math.min(h / 2 - 26, L.py));
        e.pupil.setAttribute('cx', L.px + wob); e.pupil.setAttribute('cy', py);
        e.pupil.setAttribute('r', open < 0.4 ? 0 : (this.mood === 'blocked' ? 22 : 30));
        e.shine.setAttribute('cx', L.px + wob + 10); e.shine.setAttribute('cy', py - 10);
        e.shine.setAttribute('r', open < 0.4 || hide ? 0 : 9);
        e.pupil.style.opacity = hide ? 0 : 1;
      });
    }

    tick(now) {
      if (now > this.nextBlink && !['sleepy', 'happy', 'stopped'].includes(this.mood)) {
        this.blinkUntil = now + 140;
        this.nextBlink = now + 2500 + Math.random() * 3500;
      }
      this.draw(now);
      requestAnimationFrame(this.tick);
    }
  }

  const api = { MOODS, SLEEPY_AFTER_S, faceFor, Face };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.PathikFace = api;
})(typeof window !== 'undefined' ? window : globalThis);
