#!/usr/bin/env python3
"""Test the robot's command understanding (LLM vs offline rules) on a laptop. No robot, no ROS.

    python3 tools/eval_commands.py                         # offline rule parser only
    GROQ_API_KEY=gsk_... python3 tools/eval_commands.py    # also the LLM (needs internet)
    GROQ_API_KEY=... python3 tools/eval_commands.py --model openai/gpt-oss-120b
    python3 tools/eval_commands.py --tests my_tests.json --csv results.csv
    GROQ_API_KEY=... python3 tools/eval_commands.py --policies always,smart     # LLM-only vs the robot's mix

Runs: offline rules; LLM "always" (every command to the LLM, rules only on failure); and "smart"
(what the robot does: rules first, LLM only when the rules are unsure). Compare accuracy and latency.
Every LLM error is printed with its reason (retired model, bad key, blocked, timeout ...).

Uses the simulator's floor (places + rooms with contents, same as the app's demo) and the REAL
parser from the robot (ros2_ws/src/lidar_robot/lidar_robot/commands.py). Each test says what the
right answer is (intent, targets in order, and optionally priority, waits in seconds, confirms,
then_home); the script scores every field and times each answer.
To improve the LLM: edit the prompt in commands.py (_parse_llm), run this again, compare.
"""
import argparse
import csv
import json
import os
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'ros2_ws', 'src', 'lidar_robot'))
sys.path.insert(0, HERE)
from lidar_robot.commands import CommandParser  # noqa: E402
from lidar_robot.places import PlaceStore  # noqa: E402
from dashboard_sim import PLACES, ROOMS, HOME  # noqa: E402


def floor():
    s = PlaceStore(os.path.join(tempfile.mkdtemp(), 'places.json'))
    for name, x, y, yaw, aliases in PLACES:
        s.add(name, x, y, yaw, aliases)
    for name, poly, contents in ROOMS:
        s.add_room(name, poly, contents)
    s.set_home(*HOME)
    return s


def score(test, got):
    """-> (all_ok, list of 'field: wanted X got Y' problems)"""
    bad = []
    if got['intent'] != test['intent']:
        bad.append(f"intent: wanted {test['intent']}, got {got['intent']}")
    if 'targets' in test and got.get('targets') != test['targets']:
        bad.append(f"targets: wanted {test['targets']}, got {got.get('targets')}")
    if 'priority' in test and got.get('priority') != test['priority']:
        bad.append(f"priority: wanted {test['priority']}, got {got.get('priority')}")
    steps = got.get('steps') or []
    if 'waits' in test and [round(s.get('wait_s', 0)) for s in steps] != test['waits']:
        bad.append(f"waits: wanted {test['waits']}, got {[round(s.get('wait_s', 0)) for s in steps]}")
    if 'confirms' in test and [bool(s.get('confirm')) for s in steps] != test['confirms']:
        bad.append(f"confirms: wanted {test['confirms']}, got {[bool(s.get('confirm')) for s in steps]}")
    if 'then_home' in test and bool(got.get('then_home')) != test['then_home']:
        bad.append(f"then_home: wanted {test['then_home']}, got {got.get('then_home')}")
    return not bad, bad


def run(name, parser, tests, rows):
    ok = 0
    times = []
    print(f'\n=== {name} ===')
    for t in tests:
        t0 = time.time()
        got = parser.parse(t['text'])
        dt = time.time() - t0
        times.append(dt)
        good, bad = score(t, got)
        ok += good
        src = got.get('source')
        mark = 'OK ' if good else 'XX '
        print(f'{mark}{dt * 1000:6.0f} ms  [{src:5}] {t["text"]}')
        for b in bad:
            print(f'                    {b}')
        rows.append({'parser': name, 'text': t['text'], 'ok': good, 'ms': round(dt * 1000),
                     'source': src, 'intent': got['intent'], 'targets': '|'.join(got.get('targets') or []),
                     'problems': '; '.join(bad), 'notes': '; '.join(got.get('notes') or [])})
    times.sort()
    print(f'--> {name}: {ok}/{len(tests)} correct ({100 * ok / len(tests):.0f} %), '
          f'median {1000 * times[len(times) // 2]:.0f} ms, worst {1000 * times[-1]:.0f} ms')
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tests', default=os.path.join(HERE, 'commands_test.json'))
    ap.add_argument('--model', default='openai/gpt-oss-20b', help='Groq model name')
    ap.add_argument('--csv')
    ap.add_argument('--policies', default='always,smart', help='LLM runs: always, smart')
    args = ap.parse_args()
    tests = json.load(open(args.tests))
    places = floor()
    rows = []
    run('offline rules', CommandParser(places, policy='off'), tests, rows)
    key = os.environ.get('GROQ_API_KEY')
    if key:
        for pol in args.policies.split(','):
            p = CommandParser(places, groq_api_key=key, model=args.model, policy=pol.strip())
            run(f'LLM {args.model} ({pol.strip()})', p, tests, rows)
            errs = [c for c in p.log.items if c.get('outcome') == 'error']
            if errs:
                print(f'    {len(errs)} LLM errors, last: {errs[0]["kind"]} {errs[0].get("status") or ""} '
                      f'{errs[0].get("detail", "")[:120]}\n    -> {errs[0]["hint"]}')
        print('\nNote: lines marked [rules] in an LLM run mean the rules answered: in "smart" because they were '
              'sure, otherwise because the LLM failed (see the errors above).')
    else:
        print('\n(Set GROQ_API_KEY to also test the LLM.)')
    if args.csv:
        with open(args.csv, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
        print('saved', args.csv)


if __name__ == '__main__':
    main()
