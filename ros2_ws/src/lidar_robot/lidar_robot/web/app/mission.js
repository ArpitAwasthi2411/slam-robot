/* Errands for head mode: "go to Shashi sir's cabin, wait 2 minutes, come back here", spoken all the way.
 *
 *   plan(e)            -> steps   e = {to, waitS (0 = until OK), waitOkS, message, back: 'here'|'home'|'stay'|<place>}
 *   new Errand(ops).run(steps, onStep) -> {ok, report, log}
 *
 * ops (given by head.js, demo or robot):
 *   say(text)                     robot speaker (demo: the phone's voice)
 *   go(place)                     -> {result: 'arrived'|'blocked'|'failed'|'cancelled', why}
 *   goBack(back)                  same, for 'here' (where the errand started) / 'home' / a place
 *   door(place)                   -> {state: 'open'|'closed'|'unknown', why}
 *   wait(seconds, label)          -> 'done' | 'ok'  (OK pressed on the face ends it early)
 *   cancelled()                   -> true once someone cancelled
 * The robot keeps driving, avoiding people and asking for help by itself; this only strings the steps together
 * and says what happens. Pure (no DOM): test/test_errand.py runs it in Node.
 */
(function (root) {
  'use strict';

  function fmtWait(s) {
    if (!s) return 'until someone presses OK';
    return s < 60 ? `${s} seconds` : s % 60 ? `${Math.floor(s / 60)} min ${s % 60} s` : `${s / 60} minute${s === 60 ? '' : 's'}`;
  }

  function plan(e) {
    const to = String(e.to || '').trim();
    if (!to) throw new Error('Choose where to go');
    const back = e.back || 'here';
    const steps = [
      { kind: 'say', text: `Going to ${to}.` },
      { kind: 'go', place: to },
      { kind: 'door', place: to },
      { kind: 'deliver', message: String(e.message || '').trim() },
      { kind: 'wait', seconds: Math.max(0, Math.min(600, Math.round(+e.waitS || 0))), okLimitS: e.waitOkS || 300 },
    ];
    if (back !== 'stay') {
      const name = back === 'here' ? 'where I started' : back === 'home' ? 'home' : back;
      steps.push({ kind: 'say', text: `Going back to ${name}.` }, { kind: 'back', back, name });
    }
    steps.push({ kind: 'report', to });
    return steps;
  }

  class Errand {
    constructor(ops) { this.ops = ops; this.log = []; this.facts = {}; }

    async run(steps, onStep) {
      const o = this.ops, f = this.facts, note = (k, v) => { this.log.push(Object.assign({ step: k }, v)); };
      const show = (i, s, extra) => onStep && onStep(Object.assign({ i, n: steps.length, kind: s.kind }, extra || {}));
      for (let i = 0; i < steps.length; i++) {
        const s = steps[i];
        if (o.cancelled()) { note('cancelled', {}); await o.say('Errand cancelled.'); return this._end(false, 'Errand cancelled.'); }
        if (s.kind === 'say') { show(i, s, { line: s.text }); await o.say(s.text); note('say', { text: s.text }); }
        else if (s.kind === 'go' || s.kind === 'back') {
          const label = s.kind === 'go' ? s.place : s.name;
          show(i, s, { line: `Going to ${label}` });
          const r = s.kind === 'go' ? await o.go(s.place) : await o.goBack(s.back);
          note(s.kind, { place: label, result: r.result, why: r.why || '' });
          if (s.kind === 'go') f.reached = r.result === 'arrived';
          else f.back = r.result === 'arrived';
          if (r.result === 'cancelled') return this._end(false, 'Errand cancelled.');
          if (r.result !== 'arrived') {
            const msg = `I could not reach ${label}${r.why ? ': ' + r.why : ''}.`;
            await o.say(msg);
            if (s.kind === 'go') {                             // never got there: skip to going back
              f.why = r.why || r.result;
              const j = steps.findIndex(x => x.kind === 'back');
              if (j > i) { i = j - 1; await o.say(`I'm going back.`); continue; }
            }
            return this._end(false, this._report(steps));
          }
        } else if (s.kind === 'door') {
          show(i, s, { line: `Checking the door of ${s.place}` });
          const d = await o.door(s.place);
          f.door = d.state;
          note('door', { place: s.place, state: d.state, why: d.why || '' });
          if (d.state === 'closed') await o.say(`The door of ${s.place} is closed.`);
        } else if (s.kind === 'deliver') {
          const text = f.door === 'closed'
            ? `I'll wait here outside${s.message ? '. My message: ' + s.message : '.'}`
            : s.message ? `Hello! ${s.message}` : 'Hello! I am here.';
          show(i, s, { line: text });
          await o.say(text);
          note('deliver', { text });
        } else if (s.kind === 'wait') {
          const how = fmtWait(s.seconds);
          if (s.seconds) await o.say(`I will wait ${how}. Press OK on my screen if you are done earlier.`);
          else await o.say('Please press OK on my screen when you are done.');
          show(i, s, { line: `Waiting ${how}`, wait: s.seconds || s.okLimitS, untilOk: !s.seconds });
          const w = await o.wait(s.seconds || s.okLimitS, `Waiting ${how}`);
          f.okPressed = w === 'ok';
          f.waited = s.seconds;
          note('wait', { seconds: s.seconds, ended: w });
          if (w === 'cancelled') return this._end(false, 'Errand cancelled.');
          if (w === 'ok') await o.say('Thank you!');
        } else if (s.kind === 'report') {
          const text = this._report(steps);
          show(i, s, { line: 'Done', report: text });
          await o.say(text);
          return this._end(true, text);
        }
      }
      return this._end(true, this._report(steps));
    }

    _report(steps) {
      const f = this.facts, to = (steps.find(s => s.kind === 'go') || {}).place || 'there';
      const parts = [];
      if (f.reached === false) parts.push(`I could not reach ${to}${f.why ? ' (' + f.why + ')' : ''}`);
      else {
        parts.push(`I went to ${to}`);
        if (f.door === 'closed') parts.push('the door was closed');
        else if (f.door === 'open') parts.push('the door was open');
        if (f.okPressed) parts.push('someone pressed OK');
        else if (f.waited) parts.push(`I waited ${fmtWait(f.waited)}`);
        else if (f.okPressed === false) parts.push('nobody pressed OK');
      }
      if (f.back === true) parts.push('and I am back');
      else if (f.back === false) parts.push('but I could not get back');
      const s = parts.length > 1 ? parts.slice(0, -1).join(', ') + (parts.at(-1).startsWith('and') || parts.at(-1).startsWith('but') ? ' ' : ', ') + parts.at(-1) : parts[0];
      return s.charAt(0).toUpperCase() + s.slice(1) + '.';
    }

    _end(ok, report) { return { ok, report, log: this.log }; }
  }

  const api = { plan, Errand, fmtWait };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.PathikMission = api;
})(typeof window !== 'undefined' ? window : globalThis);
