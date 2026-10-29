#!/usr/bin/env python3
"""Error-recovery evaluation: scripted failures on the simulated floor, LLM vs rules vs no recovery.

    python3 tools/eval_recovery.py                      # none + rules (+ llm when GROQ_API_KEY is set)
    GROQ_API_KEY=... python3 tools/eval_recovery.py --policies llm --model llama-3.3-70b-versatile
    python3 tools/eval_recovery.py --rounds 3 --out results/

Every scenario starts the same mission on tools/dashboard_sim.py's CSE floor and adds one kind of trouble
(a person standing in the corridor, a trolley parked in a door, a stop that is inside furniture, the position
lost for a while, a blocked corridor with a low battery ...). Simulated time runs faster than real time.
When the robot asks a person, a simulated person answers in the app after 15 s.

Per run: success (the scenario's good outcome), time to recover (first failure -> the mission moves again
or ends), recoveries used, go-home / go-dock decisions that were not needed (battery fine), questions to a person.
Writes results/recovery_eval.csv, .json and .md (the table for the report).
"""
import argparse
import csv
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dashboard_sim as sim  # noqa: E402
from lidar_robot.recovery import Recovery, RecoveryLLM  # noqa: E402


class SimClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def _near(b, x, y, d=0.35):
    return math.hypot(b.x - x, b.y - y) < d


def place(name):
    return next(p for p in sim.PLACES if p[0] == name)


# name, what happens, mission request, trouble(b, t0), answer a person gives, good(b, missions) -> bool
def _arrived(b, label):
    return any(k == 'arrived' and label in t for k, t in b.nav.say_queue)


def _add_bench(b, t0):
    b.places.add('Bench', 2.0, 0.85, 0.0)        # a saved place that ended up inside a lab bench


SCENARIOS = [
    dict(name='person_in_corridor', what='a person stands in the corridor for 25 s',
         req={'type': 'goto_place', 'name': 'Store'},
         trouble=lambda b, t0: b.blocks.append((5.0, 4.25, 0.55, t0 + 25.0)),
         answer='retry', good=lambda b, ms: ms[0].state == 'DONE' and _near(b, *place('Store')[1:3])),
    dict(name='trolley_in_door', what='a trolley parked for good in the Lab 2 door; trip Lab 2 then HOD office',
         req={'type': 'command', 'text': 'go to lab 2 then hod office'},
         trouble=lambda b, t0: b.blocks.append((7.0, 3.45, 0.6, None)),
         answer='skip', good=lambda b, ms: _arrived(b, 'HOD office')),
    dict(name='door_blocked_100s', what='the Seminar hall door is blocked for 100 s',
         req={'type': 'goto_place', 'name': 'Seminar hall'},
         trouble=lambda b, t0: b.blocks.append((12.5, 5.05, 0.6, t0 + 100.0)),
         answer='retry', good=lambda b, ms: ms[0].state == 'DONE' and _near(b, *place('Seminar hall')[1:3])),
    dict(name='stop_in_furniture', what='trip "Bench then Lab 2": the Bench place is inside a lab bench',
         req={'type': 'command', 'text': 'go to bench then lab 2'}, trouble=_add_bench, before=True,
         answer='skip', good=lambda b, ms: _arrived(b, 'Lab 2')),
    dict(name='position_lost_40s', what='no position for 40 s on the way (SLAM restart, loose cable)',
         req={'type': 'goto_place', 'name': 'Faculty room'},
         trouble=lambda b, t0: setattr(b, 'pose_drop_until', t0 + 48.0) or setattr(b, '_drop_from', t0 + 8.0),
         answer='retry', good=lambda b, ms: ms[0].state == 'DONE' and _near(b, *place('Faculty room')[1:3])),
    dict(name='blocked_low_battery', what='urgent trip, corridor blocked for good, battery already low',
         req={'type': 'command', 'text': 'urgent: go to the store'}, settle_s=60.0,
         pre=lambda b: setattr(b, 'bat_v', 10.75),
         trouble=lambda b, t0: b.blocks.append((5.0, 4.25, 0.55, None)),
         answer='dock', good=lambda b, ms: b.nav.docked or _arrived(b, 'dock') or b.nav.state in ('DOCKING', 'CHARGING')),
]


def run_one(sc, policy, model, seed_round, max_s=300.0, person_after=15.0):
    clock = SimClock()
    llm_time = [0.0, 0]
    if policy == 'none':
        rec = Recovery(mode='off')
    else:
        llm = RecoveryLLM(os.environ.get('GROQ_API_KEY'), model=model) if policy == 'llm' else None
        if llm is not None:
            real = llm.decide

            def timed(ctx):
                t = time.monotonic()
                try:
                    return real(ctx)
                finally:
                    llm_time[0] += time.monotonic() - t
                    llm_time[1] += 1
            llm.decide = timed
        rec = Recovery(llm=llm, mode='llm' if policy == 'llm' else 'rules', threaded=False)
    b = sim.SimBackend(start_thread=False, clock=clock, recovery=rec,
                       data_dir=os.path.join(OUT_TMP, f'{sc["name"]}_{policy}_{seed_round}'))
    b.drain = 0.0
    dt = 0.02
    if sc.get('pre'):
        sc['pre'](b)
    for _ in range(int(sc.get('settle_s', 2.0) / dt)):     # settle (first scan, map, battery average)
        b.tick(dt, clock.t)
        clock.t += dt
    t0 = clock.t
    if sc.get('trouble'):
        sc['trouble'](b, t0)
    drop_from = getattr(b, '_drop_from', None)
    if drop_from is not None:                             # position goes away later, not at once
        until, b.pose_drop_until = b.pose_drop_until, -1.0
    r = b.nav.handle(dict(sc['req']), (b.x, b.y, b.yaw))
    if not r.get('ok'):
        return {'scenario': sc['name'], 'policy': policy, 'success': False, 'result': 'REJECTED', 'note': r.get('error')}
    mid = r['mission']['id']
    if sc.get('trip'):
        b.nav.queue.queue[0].skip_unreachable = False if policy != 'none' else True
    first_fail = None
    asked = 0
    ask_seen = None
    while clock.t - t0 < max_s:
        if drop_from is not None and clock.t >= drop_from:
            b.pose_drop_until, drop_from = until, None
        b.tick(dt, clock.t)
        clock.t += dt
        cur = b.nav.rec.current
        if cur and first_fail is None:
            first_fail = cur['t']
            first_fail_sim = clock.t
        if cur and cur['status'] == 'asking':
            if ask_seen != cur['id']:
                ask_seen, ask_t = cur['id'], clock.t
                asked += 1
            elif clock.t - ask_t > person_after:
                b.nav.handle({'type': 'recovery_answer', 'id': cur['id'], 'choice': sc['answer'], 'by': 'sim person'})
        if not b.nav.queue.active and not b.nav.queue.queue and b.nav.state != 'RECOVERING':
            break
    ms = [m for m in b.nav.queue.history if m.id == mid] or [b.nav.queue.active]
    ms += [m for m in b.nav.queue.history if m.id != mid] + ([b.nav.queue.active] if b.nav.queue.active else []) + b.nav.queue.queue
    ms = [m for m in ms if m is not None]
    hist = list(reversed(b.nav.rec.history))
    decisions = [h for h in hist if h.get('action')]
    bat_ok = sc['name'] != 'blocked_low_battery'
    resolved = [h.get('recovered_s') for h in hist if h.get('recovered_s') is not None]
    ok = bool(sc['good'](b, ms))
    row = {
        'scenario': sc['name'], 'policy': policy, 'round': seed_round, 'success': ok,
        'result': ms[0].state if ms else 'NONE', 'time_s': round(clock.t - t0, 1),
        'time_to_recover_s': round(sum(resolved), 1) if resolved else (None if first_fail is None else round(clock.t - first_fail_sim, 1)),
        'recoveries': len(decisions), 'actions': ' > '.join(f'{h["action"]}' for h in decisions),
        'sources': ' > '.join(f'{h["source"]}' for h in decisions),
        'unnecessary_go_home': sum(1 for h in decisions if h['action'] in ('go_home', 'go_dock') and bat_ok),
        'asked_person': asked, 'bumps': b.bumps,
        'llm_calls': llm_time[1], 'llm_s': round(llm_time[0], 2),
        'reason': (ms[0].message if ms else '')[:90],
    }
    if row['time_to_recover_s'] is not None and llm_time[1]:
        row['time_to_recover_s'] = round(row['time_to_recover_s'] + llm_time[0], 1)   # real thinking time
    return row


def summarise(rows):
    out = {}
    for r in rows:
        s = out.setdefault(r['policy'], {'runs': 0, 'success': 0, 'recover_s': [], 'recoveries': 0,
                                         'unnecessary_go_home': 0, 'asked_person': 0, 'llm_s': 0.0})
        s['runs'] += 1
        s['success'] += bool(r['success'])
        if r.get('time_to_recover_s') is not None:
            s['recover_s'].append(r['time_to_recover_s'])
        for k in ('recoveries', 'unnecessary_go_home', 'asked_person'):
            s[k] += r.get(k) or 0
        s['llm_s'] += r.get('llm_s') or 0
    for s in out.values():
        s['success_rate'] = round(s['success'] / s['runs'], 3) if s['runs'] else None
        s['avg_time_to_recover_s'] = round(sum(s['recover_s']) / len(s['recover_s']), 1) if s['recover_s'] else None
        del s['recover_s']
    return out


def markdown(rows, summ, model, notes):
    lines = ['# Error recovery evaluation (simulated floor)', '',
             f'Generated by `tools/eval_recovery.py` on {time.strftime("%Y-%m-%d %H:%M")}. LLM model: `{model}`.', '']
    lines += [f'- {n}' for n in notes]
    lines += ['', '| policy | runs | success | avg time to recover (s) | recoveries | unnecessary go-home | asked a person |',
              '|---|---|---|---|---|---|---|']
    for p, s in summ.items():
        lines.append(f'| {p} | {s["runs"]} | {s["success"]}/{s["runs"]} ({s["success_rate"] * 100:.0f} %) | '
                     f'{s["avg_time_to_recover_s"] if s["avg_time_to_recover_s"] is not None else "–"} | {s["recoveries"]} | '
                     f'{s["unnecessary_go_home"]} | {s["asked_person"]} |')
    lines += ['', '| scenario | what happens | policy | success | time (s) | recover (s) | actions |', '|---|---|---|---|---|---|---|']
    what = {s['name']: s['what'] for s in SCENARIOS}
    for r in rows:
        lines.append(f'| {r["scenario"]} | {what[r["scenario"]]} | {r["policy"]} | {"yes" if r["success"] else "no"} | '
                     f'{r.get("time_s", "–")} | {r.get("time_to_recover_s") if r.get("time_to_recover_s") is not None else "–"} | '
                     f'{r.get("actions") or "–"} |')
    return '\n'.join(lines) + '\n'


OUT_TMP = None


def main(argv=None):
    global OUT_TMP
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--policies', default='none,rules,llm', help='comma list of none, rules, llm')
    ap.add_argument('--model', default=os.environ.get('RECOVERY_MODEL', 'llama-3.3-70b-versatile'))
    ap.add_argument('--rounds', type=int, default=1)
    ap.add_argument('--only', help='run one scenario by name')
    ap.add_argument('--out', default=os.path.join(HERE, '..', 'results'))
    a = ap.parse_args(argv)
    import tempfile
    OUT_TMP = tempfile.mkdtemp(prefix='eval_recovery_')
    notes = []
    policies = [p.strip() for p in a.policies.split(',') if p.strip()]
    if 'llm' in policies and not os.environ.get('GROQ_API_KEY'):
        policies.remove('llm')
        notes.append('llm skipped: GROQ_API_KEY is not set (put it in ~/robot.env or export it, then run again)')
        print('!! ' + notes[-1])
    rows = []
    for rnd in range(1, a.rounds + 1):
        for sc in SCENARIOS:
            if a.only and sc['name'] != a.only:
                continue
            for pol in policies:
                r = run_one(sc, pol, a.model, rnd)
                rows.append(r)
                print(f'{sc["name"]:22} {pol:6} {"OK  " if r["success"] else "FAIL"} {r.get("time_s", 0):6} s  '
                      f'{r.get("actions") or "-"}')
    summ = summarise(rows)
    os.makedirs(a.out, exist_ok=True)
    base = os.path.join(a.out, 'recovery_eval')
    keys = sorted({k for r in rows for k in r})
    with open(base + '.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    with open(base + '.json', 'w') as f:
        json.dump({'model': a.model, 'notes': notes, 'summary': summ, 'runs': rows}, f, indent=1)
    with open(base + '.md', 'w') as f:
        f.write(markdown(rows, summ, a.model, notes))
    print()
    for p, s in summ.items():
        print(f'{p:6}: {s["success"]}/{s["runs"]} recovered, avg {s["avg_time_to_recover_s"]} s to recover, '
              f'{s["unnecessary_go_home"]} unnecessary go-home, {s["asked_person"]} questions to a person')
    print('saved', base + '.csv / .json / .md')
    return summ


if __name__ == '__main__':
    main()
