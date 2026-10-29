// Feature parity check for the Pathik app (docs/APP_FEATURES.md), against the built-in demo robot.
// Opens the app in Chromium at phone size, logs in as staff, operator and admin, and checks that every key
// feature is reachable for the roles that should have it (and hidden for the ones that should not).
// Run: node test/app_parity.js   (needs the playwright package; test_app_parity.py runs it under pytest)
// Prints one JSON line: {"ok": bool, "failures": [...], "checked": N}
const path = require('path');
let chromium;
try { ({ chromium } = require('playwright')); } catch (e) { console.log(JSON.stringify({ skip: 'playwright not installed' })); process.exit(0); }

const APP = 'file://' + path.resolve(__dirname, '../lidar_robot/web/app/index.html');
const ROLES = { staff: 0, operator: 1, admin: 2 };
const failures = [];
let checked = 0;
const check = (cond, what) => { checked++; if (!cond) failures.push(what); };

async function visible(p, sel) { return p.locator(sel).first().isVisible().catch(() => false); }
async function tab(p, t) { await p.click(`#tab-${t}`); await p.waitForTimeout(350); }

async function run(browser, role, theme) {
  const ctx = await browser.newContext({ viewport: { width: 360, height: 780 }, colorScheme: theme });
  const p = await ctx.newPage();
  const errs = [];
  p.on('pageerror', e => errs.push(String(e)));
  const r = ROLES[role], op = r >= 1, admin = r >= 2, tag = `${role}/${theme}`;
  await p.goto(APP);
  // connect screen (C1-C8)
  for (const s of ['#c-scan', '#c-demo', '#c-addr', '#c-list', '.help', '#connect .theme-toggle']) check(await visible(p, s), `${tag}: connect ${s}`);
  await p.click('#c-demo');
  // login screen (L1-L3, L7)
  for (const s of ['#au-user', '#au-pass', '#au-show', '#au-switch-b', '#au-back', '#au-demo-row button']) check(await visible(p, s), `${tag}: login ${s}`);
  await p.click(`#au-demo-row button[data-u="${role}"]`);
  await p.waitForSelector('#main:not([hidden])');
  await p.waitForTimeout(1200);

  // frame (M5-M8, M18), tabs per role
  for (const s of ['#t-robot', '#t-mode', '#t-bat', '#t-cam', '#estop', '#tab-home', '#tab-send', '#tab-map', '#tab-more']) check(await visible(p, s), `${tag}: frame ${s}`);
  check((await visible(p, '#tab-drive')) === op, `${tag}: Drive tab ${op ? 'missing' : 'shown to staff'}`);
  check((await p.textContent('#tab-more-t')).trim() === (op ? 'Operate' : 'Me'), `${tag}: Operate/Me label`);

  // Home (G1, G2, G6, G10, G11, new status card)
  await tab(p, 'home');
  for (const s of ['#g-hello', '#h-state', '#h-bat', '#h-link', '#h-send', '#q-home', '#q-charge', '#q-why', '#q-cam', '#g-charge', '#g-bat']) check(await visible(p, s), `${tag}: home ${s}`);
  check((await visible(p, '#g-dock')) === op, `${tag}: home dock button role gating`);
  check((await p.textContent('#h-state')).trim() !== 'connecting…', `${tag}: home status filled`);
  await p.click('#q-why'); await p.waitForTimeout(400);
  check(await visible(p, '#m-why'), `${tag}: why answer card (M14)`);
  await p.click('#q-cam'); await p.waitForTimeout(500);
  check(await visible(p, '#cam'), `${tag}: camera card (M16)`);
  await p.click('#cam-x');

  // Send (G3-G5, G7-G9, G12)
  await p.click('#h-send'); await p.waitForTimeout(400);
  check(await p.evaluate(() => document.body.classList.contains('tab-send')), `${tag}: Send or call → Send tab`);
  for (const s of ['#g-cmd-in', '#g-voice', '#g-search', '#g-places li', '#g-rooms li']) check(await visible(p, s), `${tag}: send ${s}`);
  for (const s of ['#g-here', '#g-home', '#g-sched-add', '#g-sched']) check((await visible(p, s)) === op, `${tag}: send ${s} role gating`);
  await p.fill('#g-cmd-in', 'take me to the 3D printer');
  await p.click('#g-cmd .send'); await p.waitForTimeout(500);
  check(await visible(p, '#g-plan [data-p=send]'), `${tag}: plan preview with Send to robot (G3)`);
  await p.locator('#g-places button[data-a=route]').nth(1).click(); await p.waitForTimeout(400);
  check(await visible(p, '#g-places .p-route'), `${tag}: inline route preview`);
  check(await visible(p, '#g-places [data-a=show]'), `${tag}: show route on map`);
  check((await p.locator('#g-places [data-a=delete]').count()) > 0 === op, `${tag}: delete place role gating`);
  check((await p.locator('#g-rooms [data-a=edit]').count()) > 0 === op, `${tag}: edit room role gating (G9)`);
  await p.fill('#g-search', 'printer'); await p.waitForTimeout(300);
  check(await visible(p, '#g-rooms .things .hit'), `${tag}: search finds a thing in a room (G7/G9)`);
  await p.fill('#g-search', '');
  await p.click('#g-plan [data-p=send]'); await p.waitForTimeout(800);

  // mission card on Home (G6)
  await tab(p, 'home');
  check(await visible(p, '#g-mission [data-a=cancel]'), `${tag}: mission card with Cancel (G6)`);

  // Map (M1-M4, P1-P7, T1-T8)
  await tab(p, 'map');
  if (await visible(p, '#m-why-x')) await p.click('#m-why-x');
  for (const s of ['#map', '#sm-where', '#m-home', '#m-follow', '.tool[data-tool=measure]', '#sheet-grip']) check(await visible(p, s), `${tag}: map ${s}`);
  for (const t of ['goal', 'draw', 'route', 'room', 'locate']) check((await visible(p, `.tool[data-tool=${t}]`)) === op, `${tag}: map tool ${t} role gating`);
  for (const s of ['#x-go', '#d-savemap']) check((await visible(p, s)) === op, `${tag}: map ${s} role gating`);
  await p.click('.tool[data-tool=measure]');
  check(await visible(p, '#m-mode'), `${tag}: measure mode bar (T4)`);
  await p.click('#m-mode [data-m=close]');
  await p.click('#sheet-grip'); await p.waitForTimeout(300);
  check(await p.evaluate(() => document.body.classList.contains('sheet-min')), `${tag}: sheet shrinks`);
  await p.click('#sheet-grip');
  if (op) {
    await p.click('.tool[data-tool=route]');
    const box = await p.locator('#map').boundingBox();
    await p.mouse.click(box.x + box.width * 0.6, box.y + box.height * 0.42);
    await p.waitForTimeout(200);
    check((await p.textContent('#m-mode')).includes('1 point'), `${tag}: route tool adds a point (T3)`);
    await p.click('#m-mode [data-m=close]');
  }

  // Drive (D1-D4)
  if (op) {
    await tab(p, 'drive');
    for (const s of ['#joy', '#d-speed', '#d-v', '.rail']) check(await visible(p, s), `${tag}: drive ${s}`);
  }

  // Operate / Me (E1-E9, B1-B8)
  await tab(p, 'more');
  for (const s of ['#me-name', '#me-role', '[data-theme-pick=dark]', '#me-switch', '#me-pass', '#me-out']) check(await visible(p, s), `${tag}: more ${s}`);
  for (const sub of ['health', 'wheels', 'path', 'tests', 'log']) {
    const btn = `[data-sub=${sub}]`;
    check((await visible(p, btn)) === op, `${tag}: Operate → ${sub} role gating`);
    if (!op) continue;
    await p.click(btn); await p.waitForTimeout(500);
    check(await visible(p, `#lab-${sub}`), `${tag}: ${sub} page opens`);
    if (sub === 'health') check((await p.locator('#s-list .tile').count()) > 10, `${tag}: health tiles (B1)`);
    if (sub === 'wheels') check(await visible(p, '#w-step') && await visible(p, '#w-save'), `${tag}: wheel tuning controls (B4/B5)`);
    if (sub === 'path') check((await p.locator('#f-params .param').count()) > 3, `${tag}: follower params (B6)`);
    if (sub === 'tests') check(await visible(p, '#t-run'), `${tag}: drive tests (B7)`);
    if (sub === 'log') check(await visible(p, '#l-csv'), `${tag}: mission log export (B8)`);
    await p.click(`#lab-${sub} [data-back]`); await p.waitForTimeout(200);
    check(await visible(p, '#more-hub'), `${tag}: back to the Operate list`);
  }
  check((await visible(p, '[data-sub=people]')) === admin, `${tag}: People role gating (E3)`);
  if (admin) {
    await p.click('[data-sub=people]'); await p.waitForTimeout(500);
    check((await p.locator('#people li').count()) >= 3, `${tag}: people list (E3)`);
    check(await p.evaluate(() => window.__pathikBack()), `${tag}: Android back closes a sub-page`);
  }
  await p.click('[data-theme-pick=light]');
  check(await p.evaluate(() => document.documentElement.dataset.theme === 'light'), `${tag}: theme switch (E5)`);

  // STOP (M18, M19) on any tab
  await tab(p, 'send');
  await p.click('#estop'); await p.waitForTimeout(400);
  check(await visible(p, '#estopped'), `${tag}: STOP overlay`);
  check((await visible(p, '#es-release')) === op, `${tag}: release role gating (M19)`);
  check(errs.length === 0, `${tag}: page errors ${errs.join(' | ')}`);

  // nothing wider than the phone (360 px)
  const wide = await p.evaluate(() => [...document.querySelectorAll('#main *')].filter(e => { const r = e.getBoundingClientRect(); return r.width && r.right > innerWidth + 1 && getComputedStyle(e).position !== 'fixed'; }).map(e => e.id || e.className).slice(0, 5));
  check(!wide.length, `${tag}: wider than 360 px: ${wide.join(', ')}`);
  await ctx.close();
}

// error recovery in the demo: a trolley blocks the Seminar hall door once (wait, then recovered), and a goal
// inside furniture makes the robot ask a person; the answer is given on Home
async function runRecovery(browser) {
  const ctx = await browser.newContext({ viewport: { width: 360, height: 780 } });
  const p = await ctx.newPage();
  const errs = [];
  p.on('pageerror', e => errs.push(String(e)));
  await p.goto(APP); await p.click('#c-demo');
  await p.click('#au-demo-row button[data-u="operator"]');
  await p.waitForSelector('#main:not([hidden])');
  await tab(p, 'send');
  await p.fill('#g-cmd-in', 'go to the seminar hall'); await p.click('#g-cmd .send');
  await p.waitForSelector('#g-plan [data-p=send]'); await p.click('#g-plan [data-p=send]');
  await tab(p, 'home');
  const shown = await p.waitForSelector('#h-recovery:not([hidden])', { timeout: 90000 }).then(() => true, () => false);
  check(shown, 'recovery: card appears on Home when the trolley blocks the door');
  if (shown) {
    const txt = await p.textContent('#h-recovery');
    check(/What happened/.test(txt) && /What I'm doing/.test(txt) && /safety rules/.test(txt), `recovery: card text (${txt.slice(0, 80)})`);
    const ok = await p.waitForFunction(() => /Recovered/.test(document.querySelector('#h-recovery').textContent), null, { timeout: 60000 }).then(() => true, () => false);
    check(ok, 'recovery: wait-and-retry recovers');
  }
  const tok = await p.evaluate(() => JSON.parse(localStorage.getItem('pathik.session')).token);
  await p.evaluate(t => window.PathikDemo.handle('/api/nav', 'POST', { type: 'goto_pose', x: 13.3, y: 1.6, label: 'store rack', replace: true }, t), tok);
  const asked = await p.waitForSelector('#h-recovery [data-rc=home]', { timeout: 20000 }).then(() => true, () => false);
  check(asked, 'recovery: asks a person (options on Home)');
  if (asked) {
    check(!(await visible(p, '#h-recovery [data-rc=skip]')), 'recovery: no Skip for a one-stop mission');
    await p.click('#h-recovery [data-rc=home]'); await p.waitForTimeout(600);
    const st = await p.evaluate(t => window.PathikDemo.handle('/api/state', 'GET', undefined, t).nav, tok);
    check(st.recovery.source === 'person' && st.recovery.outcome === 'gave_up_home', 'recovery: the answer is carried out');
  }
  // front docking: Charge on Home -> the docking card walks through the steps until charging
  await p.evaluate(t => window.PathikDemo.handle('/api/nav', 'POST', { type: 'cancel' }, t), tok);
  await tab(p, 'home');
  await p.click('#q-charge');
  const dockCard = await p.waitForSelector('#h-docking:not([hidden])', { timeout: 60000 }).then(() => true, () => false);
  check(dockCard, 'docking: progress card on Home');
  const docked = await p.waitForFunction(() => /Docked and charging/.test(document.querySelector('#h-docking').textContent), null, { timeout: 120000 }).then(() => true, () => false);
  check(docked, 'docking: the demo robot docks nose-first and the card says so');
  // voice: hold "talk to the robot" -> the demo robot hears a sample command and answers
  check(await visible(p, '#h-voice-card') && await visible(p, '#h-talk'), 'voice: robot voice card with the talk button');
  await p.locator('#h-talk').scrollIntoViewIfNeeded();
  const tb = await p.locator('#h-talk').boundingBox();
  await p.mouse.move(tb.x + tb.width / 2, tb.y + tb.height / 2);
  await p.mouse.down(); await p.waitForTimeout(700);
  check(/Listening/.test(await p.textContent('#h-voice')), 'voice: listening while held');
  await p.mouse.up();
  const heard = await p.waitForFunction(() => /Heard:/.test(document.querySelector('#h-voice').textContent), null, { timeout: 8000 }).then(() => true, () => false);
  check(heard, 'voice: what it heard and its reply are shown');
  check(errs.length === 0, `recovery: page errors ${errs.join(' | ')}`);
  await ctx.close();
}

(async () => {
  const browser = await chromium.launch();
  try {
    for (const role of Object.keys(ROLES)) await run(browser, role, role === 'operator' ? 'light' : 'dark');
    await runRecovery(browser);
  } catch (e) { failures.push('crash: ' + (e && e.message || e)); }
  await browser.close();
  console.log(JSON.stringify({ ok: !failures.length, failures, checked }));
  process.exit(failures.length ? 1 : 0);
})();
