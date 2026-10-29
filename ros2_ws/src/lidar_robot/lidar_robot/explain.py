"""'Why did you stop?' — a plain-language answer from the navigator's state and event log (offline).

    explain(core) -> {'answer': 'Something is in front of me, about 0.3 m away. ...',
                      'details': ['12:01:03 obstacle in the way: finding another route (1/3)', ...]}
"""
import time

FAIL_TEXT = {
    'NO_PATH': "I couldn't find a free route to {goal}: the way is blocked, or that part of the map "
               "isn't explored yet.",
    'GOAL_IN_OBSTACLE': "{goal} is inside or too close to a wall or furniture on the map, so I can't stand there. "
                        "Move that place a little into open floor.",
    'OUTSIDE_MAP': "{goal} is outside the map.",
    'START_BLOCKED': "I'm too close to something to plan a route. Push me a little into open space.",
    'BLOCKED': "The way to {goal} stayed blocked even after I tried other routes.",
    'POSE_LOST': "I lost track of where I am (no position from SLAM or wheel odometry for a while). "
                 "Check the LiDAR and ESP32 cables and the battery.",
    'NO_MAP': "I don't have a map yet.",
    'UNKNOWN_PLACE': "I don't know that place.",
    'DOCK_NO_CONTACT': "I reversed onto the dock but no charging current started. Check the dock is "
                       "powered and the contacts line up.",
    'BATTERY': "My battery is too low for new tasks. Let me charge first.",
}


def explain(core):
    st = core.state
    m = core.queue.active
    goal = m.current.label if (m and m.current) else 'the goal'
    f = core.follower
    bat = core.battery.info() if core.battery else {'level': 'unknown'}
    parts = []

    if core.estopped:
        parts.append('The emergency stop is pressed. Release it, then press Resume.')
    elif core.hold and (core.queue.active or core.queue.queue):
        parts.append('I was stopped by the e-stop earlier. My tasks are paused until someone presses Resume.')
    elif not core.localized:
        parts.append("I'm waiting for someone to check that my position on the map is right "
                     "(press Position OK, or Set position).")
    elif st == 'CONFIRM':
        parts.append(f"I'm at {goal} and waiting for someone to press OK before I continue.")
    elif st in ('DOCKING', 'UNDOCKING'):
        parts.append('I am moving onto the charging dock.' if st == 'DOCKING' else 'I am leaving the dock.')
    elif st == 'CHARGING':
        parts.append(f"I'm charging on the dock ({bat.get('pct')} %).")
    elif st == 'WAITING' and core._pose_lost_for > 1.0:
        parts.append("I lost my position for a moment and I'm waiting for it to come back "
                     f"(I give up after {core.pose_lost_fail_s:.0f} s).")
    elif m and f.blocked:
        why = f.why_blocked
        if why == 'turn':
            parts.append(f'I need to turn towards {goal}, but my back would swing into something beside or behind me. '
                         "I'll try another route in a moment.")
        elif why == 'body':
            parts.append(f'The next curve would bring my body too close to something. Waiting, then trying another way to {goal}.')
        else:
            parts.append(f'Something is in front of me on the way to {goal}. If it does not move, '
                         'I will look for another route.')
    elif st == 'DWELL' and m:
        left = max(0.0, core._dwell_until - core.clock())
        parts.append(f"I'm at {goal}, waiting {left:.0f} s more as asked." if left > 1 else f"I've just reached {goal}.")
    elif st in ('PLANNING', 'WAITING') and m:
        parts.append(f"I'm working out a route to {goal}: {core.message}.")
    elif st == 'DRIVING' and m:
        parts.append(f"I'm not stopped: I'm driving to {goal}.")
    elif core.last_failure and (time.time() - core.last_failure.get('t', 0) < 600):
        lf = core.last_failure
        g = (lf.get('mission') or {}).get('stops', [goal])
        g = g[min(len(g) - 1, (lf.get('mission') or {}).get('next_stop', 0))] if g else goal
        tpl = FAIL_TEXT.get(lf['code'])
        parts.append(tpl.format(goal=g) if tpl else lf['text'])    # raw text may contain braces
    else:
        parts.append("I'm not doing anything right now: no tasks are queued.")

    if bat['level'] in ('low', 'critical'):
        parts.append(f"My battery is {bat['level']} ({bat.get('pct')} %), so I'm heading to charge after this."
                     if bat['level'] == 'low' else f"My battery is critical ({bat.get('pct')} %).")
    details = [time.strftime('%H:%M:%S ', time.localtime(e['t'])) + e['text']
               for e in core.events[:8] if e.get('level') in ('warn', 'error')]
    return {'answer': ' '.join(parts), 'details': details, 'state': st}
