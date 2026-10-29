"""Error recovery: when a mission step fails, pick ONE safe action from a fixed menu (pure Python, tested).

    rec = Recovery(llm=RecoveryLLM(os.environ.get('GROQ_API_KEY'), model='llama-3.3-70b-versatile'))
    rec.start(context)            # robot is stopped; the LLM runs in a thread (rules answer at once)
    d = rec.poll()                # None while deciding, then {'action': 'wait_retry', 'params': {...}, ...}

The thesis idea: the LLM sees the failure in context (code, place, mission, queue, battery, time of day,
what happened at this place before) and chooses what to do, but it can only choose from the menu below,
its JSON is validated and every number is clamped. No key, no internet, a timeout or a bad answer ->
the rule-based policy decides instead, from the same menu. Both are logged with the reason.

Menu (MENU): wait_retry, replan_wide, alt_route, skip_stop, reorder, ask_person, go_home, go_dock.
"""
import json
import threading
import time

from lidar_robot.commands import GROQ_URL, _http_post_json

MENU = {
    'wait_retry': {'help': 'stop, wait wait_s seconds (a person or a trolley may move), then plan again',
                   'params': {'wait_s': (3.0, 120.0, 15.0)}},
    'replan_wide': {'help': 'plan again keeping margin_m more distance from walls and obstacles',
                    'params': {'margin_m': (0.03, 0.25, 0.10)}},
    'alt_route': {'help': 'drive via one of the listed alternative waypoints first, then to the stop',
                  'params': {'via': None}},
    'skip_stop': {'help': 'give up this stop only and continue with the next stop of the mission',
                  'params': {}},
    'reorder': {'help': 'do this stop later: let the other queued tasks (or the other stops) go first',
                'params': {}},
    'ask_person': {'help': 'stay here and ask a person in the app what to do (retry, skip, go home, cancel)',
                   'params': {'question': None}},
    'go_home': {'help': 'give up the mission and drive home', 'params': {}},
    'go_dock': {'help': 'give up the mission and go to the charging dock', 'params': {}},
}

WHAT = {
    'NO_PATH': "I couldn't find a free route to {place}.",
    'START_BLOCKED': "I'm too close to something to plan a route to {place}.",
    'BLOCKED': 'The way to {place} stayed blocked after several new routes.',
    'GOAL_IN_OBSTACLE': '{place} is inside or too close to an obstacle on the map.',
    'OUTSIDE_MAP': '{place} is outside the map.',
    'POSE_LOST': 'I lost my position on the map on the way to {place}.',
    'DOCK_NO_CONTACT': 'I reached the dock but the charging contacts did not connect.',
    'DOCK_NO_MARKER': "I couldn't see the dock marker in front of the dock.",
    'NO_MAP': "I don't have a map yet.",
}
RECOVERABLE = set(WHAT)

ASK_OPTIONS = [('retry', 'Try again', True), ('wait', 'Wait 1 min, then try'), ('skip', 'Skip this stop'),
               ('home', 'Go home'), ('dock', 'Go and charge'), ('cancel', 'Cancel the mission')]

SYSTEM = (
    'You are the error-recovery planner of a small indoor campus delivery robot. A mission step just failed. '
    'Choose exactly ONE action from this fixed menu (never invent another action):\n{menu}\n'
    'Only actions listed in "allowed" are possible right now. Prefer the cheapest action that is likely to '
    'work: people and trolleys in corridors usually move within a minute; a stop that failed many times at '
    'this place before is better skipped or asked about; go home / go to the dock only when the battery is low '
    'or nothing else can work. For alt_route, "via" must be one of the ids in "alternatives". '
    'Answer with JSON only: {{"action": "...", "params": {{...}}, "reason": "one short sentence for the log", '
    '"say": "one short friendly sentence the robot says"}}')


def _menu_text():
    lines = []
    for name, spec in MENU.items():
        ps = ', '.join(f'{k} {v[0]:g}..{v[1]:g}' if v else k for k, v in spec['params'].items())
        lines.append(f'- {name}({ps}): {spec["help"]}')
    return '\n'.join(lines)


def allowed_actions(ctx):
    """Which menu items make sense for this failure (the LLM and the rules only see these)."""
    m = ctx.get('mission') or {}
    stops, nxt = m.get('stops') or [], m.get('next_stop', 0)
    a = ['wait_retry', 'ask_person']
    if ctx['failure']['code'] in ('BLOCKED',):
        a.append('replan_wide')
    if ctx.get('alternatives'):
        a.append('alt_route')
    if len(stops) > 1:
        a.append('skip_stop')
    if ctx.get('queue') or len(stops) - nxt > 1:
        a.append('reorder')
    if ctx.get('home_set') and not ctx.get('going_home'):
        a.append('go_home')
    if ctx.get('dock_set') and not ctx['failure']['code'].startswith('DOCK_'):
        a.append('go_dock')
    return [x for x in MENU if x in a]


def validate(raw, ctx):
    """LLM (or rules) output -> clean decision. Raises ValueError for anything off the menu."""
    if not isinstance(raw, dict):
        raise ValueError('decision is not a JSON object')
    action = str(raw.get('action') or '').strip()
    if action not in MENU:
        raise ValueError(f'action {action!r} is not on the menu')
    if action not in ctx['allowed']:
        raise ValueError(f'action {action!r} is not possible now')
    given = raw.get('params') if isinstance(raw.get('params'), dict) else {}
    params = {}
    for k, lim in MENU[action]['params'].items():
        v = given.get(k)
        if lim is not None:
            try:
                v = float(v)
                if v != v:                                   # NaN
                    raise ValueError
            except (TypeError, ValueError):
                v = lim[2]
            params[k] = round(min(lim[1], max(lim[0], v)), 2)
        elif k == 'via':
            ids = {a['id'] for a in ctx.get('alternatives', [])}
            if str(v) not in ids:
                raise ValueError(f'via {v!r} is not one of the alternatives')
            params[k] = str(v)
        elif k == 'question':
            q = str(v or '').strip()[:160]
            params[k] = q or ctx_question(ctx)
    return {'action': action, 'params': params,
            'reason': str(raw.get('reason') or '').strip()[:200],
            'say': str(raw.get('say') or '').strip()[:160]}


def ctx_question(ctx):
    return f'I can\'t get to {ctx["failure"]["place"]}. What should I do?'


# rule-based policy: per failure code, an escalation ladder (n-th recovery at this stop -> n-th step)
LADDER = {
    'BLOCKED': [('wait_retry', {'wait_s': 20}), ('alt_route', {}), ('replan_wide', {'margin_m': 0.12}),
                ('reorder', {}), ('skip_stop', {}), ('ask_person', {})],
    'NO_PATH': [('wait_retry', {'wait_s': 15}), ('alt_route', {}), ('reorder', {}), ('skip_stop', {}),
                ('ask_person', {})],
    'START_BLOCKED': [('wait_retry', {'wait_s': 10}), ('ask_person', {})],
    'GOAL_IN_OBSTACLE': [('skip_stop', {}), ('ask_person', {})],
    'OUTSIDE_MAP': [('skip_stop', {}), ('ask_person', {})],
    'POSE_LOST': [('wait_retry', {'wait_s': 20}), ('ask_person', {})],
    'DOCK_NO_CONTACT': [('wait_retry', {'wait_s': 5}), ('wait_retry', {'wait_s': 10}), ('ask_person', {})],
    'DOCK_NO_MARKER': [('wait_retry', {'wait_s': 5}), ('ask_person', {})],
    'NO_MAP': [('wait_retry', {'wait_s': 20}), ('ask_person', {})],
}


def rule_decide(ctx, exclude=()):
    allowed = [a for a in ctx['allowed'] if a not in exclude]
    bat = (ctx.get('battery') or {}).get('level')
    if bat in ('low', 'critical'):
        for a in ('go_dock', 'go_home'):
            if a in allowed:
                return {'action': a, 'params': {}, 'reason': f'battery {bat}: charge before trying again',
                        'say': 'My battery is low, I am going to charge first.'}
    ladder = [(a, p) for a, p in LADDER.get(ctx['failure']['code'], LADDER['NO_PATH']) if a in allowed]
    n = ctx.get('tries_here', 0)
    if n >= len(ladder):
        return None                                   # nothing left to try: the mission fails
    a, p = ladder[n]
    if a == 'alt_route':
        p = {'via': ctx['alternatives'][0]['id']}
    d = validate({'action': a, 'params': p}, ctx)
    d['reason'] = f'rule {ctx["failure"]["code"]} step {n + 1}: {MENU[a]["help"]}'
    d['say'] = SAY.get(a, '')
    return d


SAY = {'wait_retry': 'Something is in my way. I will wait a moment and try again.',
       'replan_wide': 'I will try a wider route.', 'alt_route': 'I will try another way round.',
       'skip_stop': 'I cannot reach this stop. I will skip it and continue.',
       'reorder': 'I will come back to this stop later.',
       'ask_person': 'I need help. Please check the app.', 'go_home': 'I am going back home.',
       'go_dock': 'I am going to the charging dock.'}


def describe(action, params, ctx, alt_label=None):
    """Plain words for the app's "What I'm doing" line."""
    place = ctx['failure']['place']
    if action == 'wait_retry':
        return f'Waiting {params["wait_s"]:.0f} s, then trying {place} again.'
    if action == 'replan_wide':
        return f'Planning a wider route to {place} ({params["margin_m"] * 100:.0f} cm more room).'
    if action == 'alt_route':
        return f'Going via {alt_label or params.get("via")} first, then to {place}.'
    if action == 'skip_stop':
        return f'Skipping {place} and carrying on with the next stop.'
    if action == 'reorder':
        return f'Doing {place} later; the other tasks go first.'
    if action == 'ask_person':
        return 'Waiting for someone to choose in the app.'
    if action == 'go_home':
        return 'Giving up this mission and going home.'
    if action == 'go_dock':
        return 'Giving up this mission and going to charge.'
    return action


class RecoveryLLM:
    """Asks a Groq-hosted model for a decision. decide() raises on any problem (caller falls back)."""

    def __init__(self, key, model='llama-3.3-70b-versatile', timeout=6.0, http_post=None):
        self.key, self.model, self.timeout = key, model, timeout
        self._post = http_post or _http_post_json

    def decide(self, ctx):
        if not self.key:
            raise RuntimeError('no GROQ_API_KEY')
        public = {k: v for k, v in ctx.items() if k not in ('home_set', 'dock_set', 'going_home')}
        body = {'model': self.model, 'temperature': 0, 'max_tokens': 220,
                'response_format': {'type': 'json_object'},
                'messages': [{'role': 'system', 'content': SYSTEM.format(menu=_menu_text())},
                             {'role': 'user', 'content': json.dumps(public)}]}
        resp = self._post(GROQ_URL, body, {'Authorization': f'Bearer {self.key}'}, self.timeout)
        return json.loads(resp['choices'][0]['message']['content'])


class Recovery:
    """One decision at a time. mode: 'llm' (LLM first, rules as fallback), 'rules', or 'off'."""

    def __init__(self, llm=None, mode='llm', clock=time.monotonic, wall=time.time, threaded=True,
                 max_per_mission=4, ask_timeout_s=120.0):
        self.llm = llm
        self.mode = mode if mode in ('llm', 'rules', 'off') else 'llm'
        self.clock, self.wall = clock, wall
        self.threaded = threaded
        self.max_per_mission = max_per_mission
        self.ask_timeout_s = ask_timeout_s
        self.current = None              # record shown in the app (see record())
        self.history = []                # finished records, newest first (context for the next decisions)
        self._pending = None
        self._ids = 0

    @property
    def enabled(self):
        return self.mode != 'off'

    def start(self, ctx):
        """Begin deciding for this failure. The caller keeps the robot stopped until poll() answers."""
        self._ids += 1
        ctx['allowed'] = allowed_actions(ctx)
        f = ctx['failure']
        rec = {'id': self._ids, 't': round(self.wall(), 1), 'code': f['code'], 'place': f['place'],
               'mission': (ctx.get('mission') or {}).get('id'), 'what_happened': WHAT.get(f['code'], f['text']).format(place=f['place']),
               'doing': 'Deciding what to do…', 'why': '', 'source': None, 'status': 'deciding',
               'action': None, 'params': {}, 'note': '', 'outcome': None, 'left_s': None, 'ask': None,
               '_ctx': ctx, '_t0': self.clock(), '_until': None}
        self.current = rec
        use_llm = self.mode == 'llm' and self.llm is not None and getattr(self.llm, 'key', None)
        box = {'done': False, 'raw': None, 'err': None}
        self._pending = (rec, box, self.clock() + (getattr(self.llm, 'timeout', 6.0) + 2.0 if use_llm else 0))
        if not use_llm:
            box['done'] = True
            box['err'] = 'no GROQ_API_KEY' if self.mode == 'llm' else 'rules mode'
            return rec

        def work():
            try:
                box['raw'] = self.llm.decide(ctx)
            except Exception as e:                        # network, quota, JSON ...
                box['err'] = f'{type(e).__name__}: {e}'[:120]
            box['done'] = True
        if self.threaded:
            threading.Thread(target=work, daemon=True).start()
        else:
            work()
        return rec

    def poll(self):
        """The decision once it is ready (validated; rules if the LLM failed), else None."""
        if not self._pending:
            return None
        rec, box, deadline = self._pending
        if not box['done'] and self.clock() < deadline:
            return None
        self._pending = None
        ctx = rec['_ctx']
        d = None
        if box['done'] and box['raw'] is not None:
            try:
                d = validate(box['raw'], ctx)
                rec['source'] = 'llm'
            except ValueError as e:
                rec['note'] = f'LLM answer rejected ({e}); rules decided'
        elif box['err'] and self.mode == 'llm':
            rec['note'] = f'LLM unavailable ({box["err"]}); rules decided'
        elif not box['done']:
            rec['note'] = 'LLM too slow; rules decided'
        if d is None:
            d = rule_decide(ctx)
            rec['source'] = 'rules'
        if d is None:
            rec.update(status='failed', doing='Nothing left to try: the mission stops here.', outcome='gave_up')
            self._finish(rec)
            return {'action': None, 'params': {}, 'reason': 'all recovery steps used', 'say': ''}
        rec.update(action=d['action'], params=d['params'], why=d['reason'] or rec['note'])
        return d

    def asking(self, question, options):
        r = self.current
        if r:
            r['status'] = 'asking'
            r['ask'] = {'question': question,
                        'options': [{'id': o[0], 'label': o[1], 'primary': bool(o[2:] and o[2])} for o in options]}
            r['_until'] = self.clock() + self.ask_timeout_s

    def running(self, doing, until=None):
        r = self.current
        if r:
            r.update(status='running', doing=doing, ask=None)
            r['_until'] = until

    def resolve(self, outcome, status='done'):
        """The action worked (recovered / skipped / home ...) or failed again."""
        r = self.current
        if r and r['status'] not in ('done', 'failed'):
            r['outcome'] = outcome
            r['status'] = status
            r['recovered_s'] = round(self.clock() - r['_t0'], 1)
            self._finish(r)

    def cancel(self, why='canceled'):
        self._pending = None
        self.resolve(why, 'failed')

    def _finish(self, r):
        self.history.insert(0, self.public(r))
        del self.history[50:]

    def busy(self):
        return bool(self.current and self.current['status'] in ('deciding', 'running', 'asking'))

    def tries(self, mission_id, place=None):
        """Recoveries already used for this mission (and at this stop)."""
        rows = [h for h in self.history if h.get('mission') == mission_id]
        return len(rows), len([h for h in rows if place is None or h.get('place') == place])

    def place_history(self, place, now_wall=None, n=5):
        now_wall = now_wall or self.wall()
        return [{'code': h['code'], 'action': h['action'], 'outcome': h['outcome'],
                 'ago_min': round((now_wall - h['t']) / 60.0, 1)}
                for h in self.history if h.get('place') == place][:n]

    def public(self, r=None):
        r = r if r is not None else self.current
        if not r:
            return None
        out = {k: v for k, v in r.items() if not k.startswith('_')}
        if r.get('_until') is not None and r['status'] in ('running', 'asking'):
            out['left_s'] = round(max(0.0, r['_until'] - self.clock()))
        return out
