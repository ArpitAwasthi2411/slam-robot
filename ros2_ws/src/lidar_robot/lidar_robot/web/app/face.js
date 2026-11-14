/* Pathik's face (head mode): which mood to show for the robot's state, and drawing the eyes.
 *
 *   faceFor(st, now, opts) -> {mood, line, sub, ok?, ask?}   pure: robot state (/api/state) -> what the face shows
 *   Face(svg)                                                draws a mood (visor, glowing eyes, lids, mouth), eases between
 *                                                            moods, blinks, glances, .talk(on) moves the mouth, .look(x, y)
 *
 * test/test_head_face.py runs faceFor in Node on the robot's real states.
 */
(function (root) {
  'use strict';
  const MOODS = ['idle', 'sleepy', 'thinking', 'driving', 'blocked', 'happy', 'listening', 'help', 'charging', 'lowbat',
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
    const what = (n.blocker || {}).now;                       // person | object | unknown (blockers.py)
    if (moving && n.blocked && what === 'object') {
      return { mood: 'thinking', line: 'Something is in my way', sub: 'No person there: finding another way' };
    }
    if (moving && n.blocked) {
      return { mood: 'blocked', line: 'Excuse me, may I pass?', sub: what === 'person' ? 'Thank you for making way' : 'Something is in my way' };
    }
    if (state === 'RECOVERING' || state === 'PLANNING') {
      return { mood: 'thinking', line: state === 'PLANNING' ? `Finding the way to ${goalName(n)}…` : 'Let me think…', sub: n.message || '' };
    }
    if (state === 'DWELL') return { mood: 'happy', line: `We're at ${goalName(n)}!`, sub: n.message || '' };
    if (state === 'DOCKING') return { mood: 'charging', line: 'Docking…', sub: n.message || '' };
    if (state === 'CHARGING' || bat.charging) {
      return { mood: 'charging', line: 'Charging' + (bat.pct != null ? ` · ${bat.pct} %` : '…'), sub: 'Tap Go or Ask if you need me' };
    }
    if (state === 'DRIVING' || state === 'UNDOCKING' || state === 'SCANNING' || state === 'WAITING') {
      const line = state === 'SCANNING' ? 'Looking around…' : state === 'UNDOCKING' ? 'Leaving the dock' : `Going to ${goalName(n)}`;
      return { mood: v.action === 'SLOW_DOWN' ? 'blocked' : 'driving', line, sub: fmtDist(n) || n.message || '' };
    }
    if (bat.level === 'low' || bat.level === 'critical') {
      return { mood: 'lowbat', line: 'My battery is low', sub: bat.pct != null ? `${bat.pct} %: I'll go and charge soon` : '' };
    }
    if ((o.idleS || 0) > SLEEPY_AFTER_S) return { mood: 'sleepy', line: 'Resting…', sub: 'Tap Go to choose a place, or Ask me something' };
    return { mood: 'idle', line: 'Hello! How can I help?', sub: 'Tap Go to choose a place, or Ask me something' };
  }

  // ------------------------------------------------------------------ drawing
  // A dark visor with two glowing eyes. Each mood is a target shape (eye width, height, corner radius, gaze,
  // lid slant, mouth, colour); every frame the face eases towards the target, so moods flow into each other.
  // It also blinks, glances around when idle, follows a touch (look), and moves its mouth while speaking (talk).
  const LOOK = {
    idle:      { w: 104, h: 128, r: 44, dx: 0,  dy: 0,   tilt: 0,  mouth: 'smile', col: '--teal', glance: true },
    sleepy:    { w: 112, h: 16,  r: 8,  dx: 0,  dy: 16,  tilt: 0,  mouth: 'flat',  col: '--dim' },
    thinking:  { w: 96,  h: 112, r: 42, dx: 18, dy: -18, tilt: -5, mouth: 'flat',  col: '--orange', uneven: 0.8 },
    driving:   { w: 108, h: 106, r: 40, dx: 0,  dy: 0,   tilt: -4, mouth: 'smile', col: '--teal', glance: true },
    blocked:   { w: 114, h: 146, r: 54, dx: 0,  dy: 0,   tilt: 9,  mouth: 'o',     col: '--orange', pulse: true },
    happy:     { shape: 'up',    mouth: 'big',   col: '--good', cheeks: true },
    listening: { w: 108, h: 138, r: 50, dx: 0,  dy: -4,  tilt: 0,  mouth: 'dot',   col: '--teal', pulse: true },
    help:      { w: 100, h: 116, r: 44, dx: 0,  dy: 6,   tilt: 13, mouth: 'frown', col: '--warn', pulse: true },
    charging:  { shape: 'down',  mouth: 'smile', col: '--good', bolt: true },
    lowbat:    { w: 104, h: 62,  r: 28, dx: 0,  dy: 12,  tilt: 9,  mouth: 'frown', col: '--warn' },
    stopped:   { shape: 'cross', mouth: 'flat',  col: '--red' },
    offline:   { shape: 'line',  mouth: 'flat',  col: '--dim-2' },
    lost:      { w: 100, h: 120, r: 44, dx: 0,  dy: 0,   tilt: 11, mouth: 'wavy',  col: '--warn', dart: true },
  };
  const MOUTH = {
    smile: 'M -34 0 Q 0 22 34 0', big: 'M -44 -4 Q 0 40 44 -4 Z', flat: 'M -24 6 L 24 6',
    frown: 'M -30 14 Q 0 -6 30 14', o: 'M -12 6 a 12 14 0 1 0 24 0 a 12 14 0 1 0 -24 0',
    dot: 'M -6 6 a 6 6 0 1 0 12 0 a 6 6 0 1 0 -12 0', wavy: 'M -40 8 q 10 -11 20 0 t 20 0 t 20 0 t 20 0',
  };
  const TALK = ['M -18 4 Q 0 14 18 4', MOUTH.o, 'M -16 2 a 16 10 0 1 0 32 0 a 16 10 0 1 0 -32 0', MOUTH.dot];
  const EASE = 0.2;

  class Face {
    constructor(svg) {
      this.svg = svg; this.mood = null; this.blinkUntil = 0; this.nextBlink = performance.now() + 2500;
      this.gaze = { x: 0, y: 0, until: 0 }; this.nextGlance = performance.now() + 3000;
      this.talking = false; this.talkT = 0; this.talkI = 0; this.mouthD = '';
      const ns = 'http://www.w3.org/2000/svg', el = (t, a, p = svg) => {
        const e = document.createElementNS(ns, t); for (const k in a) e.setAttribute(k, a[k]); p.appendChild(e); return e; };
      svg.setAttribute('viewBox', '-300 -200 600 400');
      const defs = el('defs', {});
      const vg = el('linearGradient', { id: 'f-visor-g', x1: 0, y1: 0, x2: 0, y2: 1 }, defs);
      el('stop', { offset: 0, 'stop-color': '#171b24' }, vg); el('stop', { offset: 1, 'stop-color': '#07080c' }, vg);
      const eg = el('linearGradient', { id: 'f-eye-g', x1: 0, y1: 0, x2: 0, y2: 1 }, defs);
      el('stop', { offset: 0, 'stop-color': '#fff', 'stop-opacity': 0.55 }, eg);
      el('stop', { offset: 0.55, 'stop-color': '#fff', 'stop-opacity': 0 }, eg);
      this.ring = el('rect', { x: -278, y: -184, width: 556, height: 336, rx: 128, class: 'f-ring' });
      el('rect', { x: -270, y: -176, width: 540, height: 320, rx: 120, class: 'f-visor' });
      el('path', { d: 'M -226 -112 Q -206 -158 -150 -160 L 40 -160 Q -150 -150 -226 -112 Z', class: 'f-gloss' });
      this.cheeks = [-196, 196].map(x => el('ellipse', { cx: x, cy: 64, rx: 30, ry: 18, class: 'f-cheek' }));
      this.eyes = [-116, 116].map((x, i) => {
        // the lid is a slanted cut across the top of the eye (worried = inner end up, focused = inner end down)
        const cut = el('polygon', { points: '-200,-300 200,-300 200,300 -200,300' },
                       el('clipPath', { id: `f-lid-${i}` }, defs));
        const g = el('g', { transform: `translate(${x} -34)`, 'clip-path': `url(#f-lid-${i})` });
        const glow = el('g', { class: 'f-glow' }, g);
        return { g, x,
                 body: el('rect', { x: -52, y: -64, width: 104, height: 128, rx: 44, class: 'f-eye' }, glow),
                 shine: el('rect', { x: -52, y: -64, width: 104, height: 128, rx: 44, class: 'f-eye-shine' }, glow),
                 up: el('path', { d: 'M -50 22 Q 0 -52 50 22', class: 'f-arc' }, glow),
                 down: el('path', { d: 'M -48 -6 Q 0 34 48 -6', class: 'f-arc' }, glow),
                 cross: el('path', { d: 'M -38 -38 L 38 38 M 38 -38 L -38 38', class: 'f-arc' }, glow),
                 line: el('path', { d: 'M -44 0 L 44 0', class: 'f-arc' }, glow),
                 cut,
                 cur: { w: 104, h: 128, r: 44, dx: 0, dy: 0, tilt: 0 }, last: '' };
      });
      this.mouth = el('path', { d: MOUTH.smile, class: 'f-mouth', transform: 'translate(0 104)' });
      this.zz = el('text', { x: 188, y: -112, class: 'f-zz' }); this.zz.textContent = 'z z';
      this.bolt = el('path', { d: 'M 8 -150 L -14 -112 L 2 -112 L -8 -80 L 16 -122 L 0 -122 Z', class: 'f-bolt' });
      this.tick = this.tick.bind(this);
      requestAnimationFrame(this.tick);
    }

    set(mood) {
      if (mood === this.mood || !LOOK[mood]) return;
      this.mood = mood;
      const L = LOOK[mood];
      this.svg.setAttribute('data-mood', mood);
      this.svg.classList.toggle('pulse', !!L.pulse);
      this.svg.style.setProperty('--face', `var(${L.col})`);
      this.zz.style.opacity = mood === 'sleepy' ? 1 : 0;
      this.bolt.style.opacity = L.bolt ? 1 : 0;
      this.cheeks.forEach(c => { c.style.opacity = L.cheeks ? 0.4 : 0; });
      this.eyes.forEach(e => {
        ['up', 'down', 'cross', 'line'].forEach(k => { e[k].style.opacity = L.shape === k ? 1 : 0; });
        e.body.style.opacity = e.shine.style.opacity = L.shape ? 0 : 1;
        if (L.shape) { e.cut.setAttribute('points', '-200,-300 200,-300 200,300 -200,300'); e.last = ''; }
      });
      this.setMouth();
    }

    /** Move the mouth as if speaking (head.js turns it on while the robot says something). */
    talk(on) { if (on !== this.talking) { this.talking = !!on; this.setMouth(); } }

    /** Glance towards a point: nx, ny in -1..1 from the middle of the face (e.g. where it was touched). */
    look(nx, ny, ms) {
      this.gaze = { x: Math.max(-1, Math.min(1, nx)) * 22, y: Math.max(-1, Math.min(1, ny)) * 12,
                    until: performance.now() + (ms || 1200) };
    }

    setMouth(now) {
      const L = LOOK[this.mood] || LOOK.idle;
      let d = MOUTH[L.mouth];
      if (this.talking && !['stopped', 'offline', 'sleepy'].includes(this.mood)) d = TALK[this.talkI % TALK.length];
      if (d !== this.mouthD) {
        this.mouthD = d;
        this.mouth.setAttribute('d', d);
        this.mouth.classList.toggle('fill', d.endsWith('Z') || this.talking && d !== TALK[0]);
      }
    }

    draw(now) {
      const L = LOOK[this.mood] || LOOK.idle;
      if (this.talking && now - this.talkT > 130) { this.talkT = now; this.talkI++; this.setMouth(); }
      if (L.shape) return;
      const blink = now < this.blinkUntil ? 0.07 : 1;
      let gx = L.dx || 0, gy = L.dy || 0;
      if (now < this.gaze.until) { gx += this.gaze.x; gy += this.gaze.y; }
      if (L.dart) gx += Math.sin(now / 300) * 20;
      if (this.mood === 'thinking') gx += Math.sin(now / 420) * 5;
      this.eyes.forEach((e, i) => {
        const side = i === 0 ? -1 : 1, c = e.cur;
        const small = L.uneven && i === 0 ? L.uneven : 1;
        const t = { w: L.w * small, h: Math.max(8, L.h * small * blink), r: L.r, dx: gx, dy: gy, tilt: L.tilt };
        for (const k in t) c[k] += (t[k] - c[k]) * (k === 'h' && blink < 1 ? 0.6 : EASE);
        const r = Math.min(c.r, c.h / 2, c.w / 2);
        const key = [c.w, c.h, r, c.dx, c.dy, c.tilt].map(v => v.toFixed(1)).join();
        if (key === e.last) return;                      // settled: nothing to repaint
        e.last = key;
        for (const b of [e.body, e.shine]) {
          b.setAttribute('x', -c.w / 2 + c.dx); b.setAttribute('y', -c.h / 2 + c.dy);
          b.setAttribute('width', c.w); b.setAttribute('height', c.h); b.setAttribute('rx', r);
        }
        // no slant: the cut sits well above the eye (its glow stays whole); slanted: it comes down across the top
        const a = Math.abs(c.tilt), k = Math.tan(side * c.tilt * Math.PI / 180);
        const y0 = c.dy - c.h / 2 + a * 2.2 - 34 * (1 - Math.min(1, a / 5));
        const y = x => (y0 + k * (x - c.dx)).toFixed(1);
        e.cut.setAttribute('points', `-200,${y(-200)} 200,${y(200)} 200,300 -200,300`);
      });
    }

    tick(now) {
      const L = LOOK[this.mood] || LOOK.idle;
      if (now > this.nextBlink && !L.shape && this.mood !== 'sleepy') {
        this.blinkUntil = now + 130;
        this.nextBlink = now + 2400 + Math.random() * 3600;
      }
      if (L.glance && now > this.nextGlance && now > this.gaze.until) {
        this.look(Math.random() * 2 - 1, Math.random() - 0.5, 500 + Math.random() * 500);
        this.nextGlance = now + 2600 + Math.random() * 4000;
      }
      this.draw(now);
      requestAnimationFrame(this.tick);
    }
  }

  const api = { MOODS, SLEEPY_AFTER_S, faceFor, Face };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.PathikFace = api;
})(typeof window !== 'undefined' ? window : globalThis);
