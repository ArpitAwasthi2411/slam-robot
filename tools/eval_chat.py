#!/usr/bin/env python3
"""Measure the conversation agent (chat.py) on a small fixed notebook: is it grounded, does it admit
what it does not know, does it turn talk into the right trip, and how long does it take?

    GROQ_API_KEY=gsk_... python3 tools/eval_chat.py [--model openai/gpt-oss-120b] [--csv results/chat.csv]

Without a key it measures the offline path (notebook read-out). Nothing here touches the robot.
"""
import argparse
import csv
import os
import sys
import tempfile
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'ros2_ws', 'src', 'lidar_robot'))
from lidar_robot.chat import ChatAgent  # noqa: E402
from lidar_robot.commands import CommandParser  # noqa: E402
from lidar_robot.knowledge import Knowledge  # noqa: E402
from lidar_robot.llm import GroqClient, LLMLog, clean_key  # noqa: E402
from lidar_robot.places import PlaceStore  # noqa: E402

NOTEBOOK = [
    ('Head of the department', 'The head of the CSE department is Dr. Meera Rao. Her cabin is the HOD office.', ['hod']),
    ('Library timings', 'The library is open from 9 am to 5 pm on working days. It is closed on Sundays.', ['books']),
    ('Project lab rules', 'Final-year students may use the project lab until 6 pm after signing the register.', ['lab']),
    ('Courses', 'The department offers B.Tech in Computer Science with a specialisation in AI and Data Science.', ['branch']),
]
# (user, text, kind, expect): kind = grounded (reply must contain expect) | unknown (must admit it) |
# general (any answer, no trip) | trip (targets must equal expect) | notrip (must not move)
CASES = [
    ('a', 'who is the head of the department?', 'grounded', 'Meera Rao'),
    ('a', 'ok take me to her cabin', 'trip', ['HOD office']),
    ('b', 'library kab khulti hai?', 'grounded', '9'),
    ('b', 'is it open on sunday?', 'grounded', 'closed'),
    ('c', 'who is the principal of the college?', 'unknown', None),
    ('c', 'what is the phone number of the office?', 'unknown', None),
    ('d', 'what is SLAM in simple words?', 'general', None),
    ('d', 'till when can we use the project lab', 'grounded', '6'),
    ('e', 'which courses does the department offer', 'grounded', 'Data Science'),
    ('e', 'library timings?', 'notrip', None),
    ('f', 'i want to print something, where should i go', 'trip', ['HOD office']),
    ('f', 'take me to the swimming pool', 'notrip', None),
    ('g', 'hello', 'general', None),
    ('g', 'what is your battery level', 'general', None),
]
ADMITS = ("don't know", 'do not know', 'not sure', 'nahi pata', 'nahin pata', 'not in my', "don't have", 'do not have')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default='openai/gpt-oss-120b')
    ap.add_argument('--csv')
    a = ap.parse_args()
    tmp = tempfile.mkdtemp()
    places = PlaceStore(os.path.join(tmp, 'p.json'))
    for name, x, y, al in (('Library', 1, 1, []), ('HOD office', 3, 7, ['head of department']),
                           ('Lab 3', 2, 2, ['computer lab']), ('Canteen', 20, 3, [])):
        places.add(name, x, y, 0, al)
    places.add_room('HOD office', [[0, 5], [5, 5], [5, 9], [0, 9]], ['printer', 'files'])
    k = Knowledge(os.path.join(tmp, 'k.json'))
    for t, x, tags in NOTEBOOK:
        k.add(t, x, tags)
    key, log = clean_key(os.environ.get('GROQ_API_KEY')), LLMLog()
    agent = ChatAgent(GroqClient(key, model=a.model, timeout=10.0, log=log), k, places)
    parser = CommandParser(places, groq_api_key=key, log=log)
    ctx = {'robot': {'state': 'IDLE', 'doing': 'waiting at home', 'battery_pct': 76}, 'pose': [0.0, 0.0]}
    rows, good, times = [], 0, []
    print(f'model: {a.model if key else "none (offline: notebook only)"}\n')
    for user, text, kind, expect in CASES:
        t0 = time.monotonic()
        r = agent.handle(text, parser, user=user, ctx=ctx)
        ms = round((time.monotonic() - t0) * 1000)
        reply, low = r.get('reply', ''), r.get('reply', '').lower()
        if kind == 'grounded':
            ok = str(expect).lower() in low
        elif kind == 'unknown':
            ok = r['intent'] == 'chat' and any(w in low for w in ADMITS)
        elif kind == 'trip':
            ok = r['intent'] == 'navigate' and r['targets'] == expect
        elif kind == 'notrip':
            ok = r['intent'] not in ('navigate', 'return_home', 'dock')
        else:
            ok = r['intent'] == 'status' or (r['intent'] == 'chat' and bool(reply))
        good += ok
        times.append(ms)
        basis = (r.get('chat') or {}).get('basis') or r.get('source')
        err = ((r.get('llm') or {}).get('error') or {}).get('kind', '')
        print(f'{"PASS" if ok else "FAIL"} {kind:8} {ms:5} ms  {text!r}\n       -> [{r["intent"]}/{basis}] {reply[:110]} {r["targets"] or ""} {err}')
        rows.append([text, kind, ok, ms, r['intent'], basis, reply, err])
    times.sort()
    print(f'\n{good}/{len(CASES)} correct · median {times[len(times) // 2]} ms · worst {times[-1]} ms · '
          f'answers blocked by the guards: {agent.stats["guarded"]} · second lookups: {agent.stats["second_lookups"]}')
    if a.csv:
        os.makedirs(os.path.dirname(a.csv) or '.', exist_ok=True)
        with open(a.csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['text', 'kind', 'ok', 'ms', 'intent', 'basis', 'reply', 'error'])
            w.writerows(rows)


if __name__ == '__main__':
    main()
