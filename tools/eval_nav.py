#!/usr/bin/env python3
"""Navigation evaluation on the simulated CSE floor, faster than real time (no ROS, no hardware).

    python3 tools/eval_nav.py                 # body-shape checks ON vs OFF (old circle model)
    python3 tools/eval_nav.py --rounds 3 --csv results.csv

Drives the robot (rectangle body, axle near the front, like ours) through every saved place and
some hard starts (backed into a corner, parked against a wall facing the wrong way) and counts:
missions done / failed, wall contacts (bumps), time, distance. Numbers for the project report.
"""
import argparse
import csv
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dashboard_sim as sim  # noqa: E402

# (start pose, goal place): hard cases where the back has to swing near walls
HARD = [
    ((0.52, 3.85, 0.0), "HOD office"),          # corridor corner, back + side 5 cm off the walls
    ((13.45, 4.25, 0.0), 'Lab 3'),              # nose into the east end, must turn round
    ((2.2, 2.0, math.pi), 'Seminar hall'),      # between the lab benches facing the wrong way
    ((12.0, 3.1, math.pi / 2), 'Store'),        # just inside the store door
]


class SimClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def run(body_checks, rounds, max_s=240.0):
    clock = SimClock()
    b = sim.SimBackend(start_thread=False, clock=clock)
    b.use_body_checks = body_checks
    b.nav.body_checks = body_checks
    b.drain = 0.0                                   # battery out of the picture here
    rows = []
    dt = 0.02

    def mission(start, goal):
        if start:
            b.x, b.y, b.yaw = start
            b.wl.target = b.wr.target = 0.0
        bumps0 = b.bumps
        t0 = clock.t
        x0, y0 = b.x, b.y
        dist = 0.0
        r = b.nav.handle({'type': 'goto_place', 'name': goal}, (b.x, b.y, b.yaw))
        if not r.get('ok'):
            return {'goal': goal, 'result': 'REJECTED', 'reason': r.get('error')}
        lx, ly = b.x, b.y
        while clock.t - t0 < max_s:
            b.tick(dt, clock.t)
            clock.t += dt
            dist += math.hypot(b.x - lx, b.y - ly)
            lx, ly = b.x, b.y
            if not b.nav.queue.active and not b.nav.queue.queue:
                break
        mid = r['mission']['id']
        h = next((m for m in b.nav.queue.history if m.id == mid), None)
        res = h.state if h else 'TIMEOUT'
        if res == 'TIMEOUT':
            b.nav.handle({'type': 'cancel'})
        return {'goal': goal, 'result': res, 'reason': (h.message if h else '')[:80],
                'time_s': round(clock.t - t0, 1), 'distance_m': round(dist, 2),
                'straight_m': round(math.hypot(b.x - x0, b.y - y0), 2), 'bumps': b.bumps - bumps0,
                'manoeuvres': b.nav.follower.manoeuvres}

    names = [p[0] for p in sim.PLACES]
    for rnd in range(rounds):
        for g in names + ['HOD office']:
            rows.append({'case': f'tour {rnd + 1}', **mission(None, g)})
        for start, g in HARD:
            rows.append({'case': 'hard start', **mission(start, g)})
    return rows


def summary(rows):
    done = sum(r['result'] == 'DONE' for r in rows)
    return {'missions': len(rows), 'done': done, 'failed': len(rows) - done,
            'bumps': sum(r.get('bumps', 0) for r in rows),
            'missions_with_contact': sum(1 for r in rows if r.get('bumps', 0)),
            'time_s': round(sum(r.get('time_s', 0) for r in rows)),
            'distance_m': round(sum(r.get('distance_m', 0) for r in rows), 1)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rounds', type=int, default=1)
    ap.add_argument('--csv')
    ap.add_argument('--only', choices=['on', 'off'])
    args = ap.parse_args()
    out = []
    for mode in ([args.only] if args.only else ['off', 'on']):
        rows = run(mode == 'on', args.rounds)
        for r in rows:
            r['body_checks'] = mode
        out += rows
        s = summary(rows)
        print(f'body checks {mode.upper():3}: {s["done"]}/{s["missions"]} reached, {s["bumps"]} wall contacts '
              f'in {s["missions_with_contact"]} missions, {s["distance_m"]} m in {s["time_s"]} s')
        for r in rows:
            if r['result'] != 'DONE' or r.get('bumps'):
                print(f'    {r["case"]:10} -> {r["goal"]:13} {r["result"]:8} bumps {r.get("bumps", 0)}  {r.get("reason", "")}')
    if args.csv:
        keys = sorted({k for r in out for k in r})
        with open(args.csv, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(out)
        print('saved', args.csv)


if __name__ == '__main__':
    main()
