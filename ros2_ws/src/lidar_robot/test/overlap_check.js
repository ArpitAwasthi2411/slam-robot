// Overlap check for test_no_overlap.py: controls hidden under the kiosk STOP or the Drive cockpit. node overlap_check.js <app dir>
const { chromium } = require('playwright');
const APP = process.argv[2];
const SIZES = [[360,780],[412,915],[390,844],[780,360],[915,412],[844,390],[1280,800],[800,1280]];
const inter = (a, b, pad = 0) => !(a.x + a.w <= b.x - pad || b.x + b.w <= a.x - pad || a.y + a.h <= b.y - pad || b.y + b.h <= a.y - pad);
async function hits(p, coverSel, scopeSel) {
  return p.evaluate(([c, s]) => {
    const cov = document.querySelector(c); if (!cov) return ['(no ' + c + ')'];
    const r = cov.getBoundingClientRect(); if (!r.width) return [];
    const out = [];
    for (const el of document.querySelectorAll(s)) {
      if (el === cov || cov.contains(el) || el.contains(cov)) continue;
      const q0 = el.getBoundingClientRect(); if (!q0.width || !q0.height) continue;
      const q = { left: q0.left, right: q0.right, top: q0.top, bottom: q0.bottom };   // visible part (scrolling parents clip it)
      for (let a = el.parentElement; a; a = a.parentElement) {
        const o = getComputedStyle(a).overflowY; if (o === 'visible') continue;
        const r2 = a.getBoundingClientRect();
        q.left = Math.max(q.left, r2.left); q.right = Math.min(q.right, r2.right); q.top = Math.max(q.top, r2.top); q.bottom = Math.min(q.bottom, r2.bottom);
      }
      if (q.right <= q.left || q.bottom <= q.top) continue;
      const st = getComputedStyle(el); if (st.visibility === 'hidden' || st.display === 'none') continue;
      if (el.closest('[hidden]')) continue;
      const ox = Math.min(r.right, q.right) - Math.max(r.left, q.left), oy = Math.min(r.bottom, q.bottom) - Math.max(r.top, q.top);
      if (ox > 2 && oy > 2) out.push((el.id ? '#' + el.id : el.tagName.toLowerCase() + '.' + (el.className || '').toString().split(' ')[0]) + ' "' + (el.textContent || el.getAttribute('aria-label') || '').trim().slice(0, 18) + '"');
    }
    return out;
  }, [coverSel, scopeSel]);
}
(async () => {
  const b = await chromium.launch();
  let bad = 0;
  for (const [w, h] of SIZES) {
    // ---- kiosk
    const p = await b.newPage({ viewport: { width: w, height: h } });
    await p.goto('file://' + APP + '/head.html#guide=1'); await p.waitForTimeout(500);
    const report = async tag => { const x = await hits(p, '#stop', 'button, input, select, a, [role=button]'); if (x.length) { bad++; console.log(`KIOSK ${w}x${h} ${tag}: ${x.join(', ')}`); } };

    await report("PIN setup"); for (const k of ["1","2","3","4","✓","1","2","3","4","✓"]) await p.evaluate(k => [...document.querySelectorAll("#pad button")].find(x => x.textContent === k).click(), k);
    await p.waitForTimeout(800);
    await report('face');
    for (const [open, close, tag] of [['#ic-go', '#k-close', 'Go sheet'], ['#ic-ask', '#a-close', 'Ask sheet'], ['#ic-send', null, 'Deliver sheet']]) {
      await p.evaluate(s => document.querySelector(s).click(), open); await p.waitForTimeout(350);
      await report(tag);
      await p.evaluate(() => { document.querySelectorAll('.h-sheet').forEach(x => { if (x.id !== 'pin') x.hidden = true; }); });
    }
    // PIN pad + settings
    await p.evaluate(() => { document.getElementById('pin').hidden = false; }); await p.waitForTimeout(200); await report('PIN pad');
    await p.evaluate(() => { document.getElementById('pin').hidden = true; const s = document.getElementById('set'); if (s) s.hidden = false; }); await p.waitForTimeout(200); await report('settings');
    await p.close();
    // ---- app Drive tab (demo, operator)
    const q = await b.newPage({ viewport: { width: w, height: h } });
    await q.goto('file://' + APP + '/index.html'); await q.waitForTimeout(800);
    await q.click('#c-demo'); await q.click('#au-demo-row button[data-u="operator"]'); await q.waitForTimeout(1200);
    await q.click('#tab-drive'); await q.waitForTimeout(900);
    const x = [];
    for (const cov of ['.joy-wrap', '.readout', '.speed', '.rail'])
      for (const y of await hits(q, cov, 'button, input, select, a, [role=button], .chip, .readout, .speed, .joy-wrap, .rail, #tabbar, nav'))
        x.push(cov + ' over ' + y);
    if (x.length) { bad++; console.log(`APP ${w}x${h} Drive: ${x.join(', ')}`); }
    await q.close();
  }
  console.log('problems:', bad);
  await b.close();
})();
