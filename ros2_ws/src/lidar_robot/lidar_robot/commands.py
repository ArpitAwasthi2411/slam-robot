"""Natural-language robot commands -> structured intent, with an offline fallback (pure Python, tested).

    parser = CommandParser(place_store, groq_api_key=os.environ.get('GROQ_API_KEY'))
    cmd = parser.parse("urgent: take this to the HOD office and then lab 3")
    -> {'intent': 'navigate', 'targets': ['HOD office', 'Lab 3'], 'priority': 3,
        'reply': '...', 'source': 'llm' | 'rules', 'notes': [...], 'llm': {...}}

How a command is understood (policy, default "smart"):
  * "stop" and friends never wait for the network: the rules answer at once (safety).
  * smart: the offline rules go first; when they are sure (a known place, home, dock, status, OK ...)
    that answer is used with zero latency. Only commands the rules are unsure about (unknown words,
    Hinglish, things instead of places) go to the LLM.  always: LLM first for everything.  off: rules only.
  * identical commands within 10 min (e.g. the app's preview, then the real send) reuse the LLM answer.

Hallucination / unwanted output guards:
  * strict JSON schema: intent is an enum and every target must be one of the map's place / room
    names (the model cannot even produce another name), priority 0-3, booleans are booleans;
  * the answer is validated again here (unknown places dropped and noted, values clamped);
  * an "unknown" from the LLM never overrides a rules answer that found something;
  * any LLM problem (no key, retired model, network, timeout, bad JSON) -> rules answer, with the exact
    error in result['llm'] and in the LLM log (app: Operate -> AI commands).
"""
import json
import re
import time

from lidar_robot.llm import CHAT_URL, DEFAULT_MODEL, GroqClient, LLMError, LLMLog, http_json
from lidar_robot.places import normalize

INTENTS = ('navigate', 'return_home', 'dock', 'stop', 'status', 'explain', 'find', 'confirm', 'unknown')
GROQ_URL = CHAT_URL
POLICIES = ('smart', 'always', 'off')
CACHE_S = 600.0

URGENT = re.compile(r'\b(urgent|urgently|asap|immediately|emergency|right now|quickly|hurry|priority|jaldi|turant|fatafat)\b')
LOW = re.compile(r'\b(low priority|when (you are |you\'re )?free|no rush|later|whenever)\b')
STOP = re.compile(r'\b(stop|halt|cancel|abort|freeze|emergency stop|wait there)\b')
HOME = re.compile(r'\b((go|come|return|head|get)\s+(back\s+)?(home|to base|to (the )?dock)|come back|go back|return|dock|go home)\b')
STATUS = re.compile(r'\b(where are you|status|what are you doing|are you busy|battery)\b')
NAV_VERB = re.compile(r'\b(go|goto|take|bring|deliver|drop|carry|navigate|move|drive|head|visit|reach|escort|guide|show)\b')
WAIT = re.compile(r'\b(?:wait|stay|pause|hold)\s+(?:there\s+)?(?:for\s+)?(\d+(?:\.\d+)?|a|an|one|two|three|four|five|six|seven|eight|nine|ten|fifteen|twenty|thirty|half an?)\s*'
                  r'(seconds?|secs?|s|minutes?|mins?|m|hours?|hrs?|h)\b')
NUM_WORDS = {'a': 1, 'an': 1, 'one': 1, 'two': 2, 'three': 3, 'four': 4, 'five': 5, 'six': 6, 'seven': 7,
             'eight': 8, 'nine': 9, 'ten': 10, 'fifteen': 15, 'twenty': 20, 'thirty': 30, 'half a': 0.5, 'half an': 0.5}
MAX_WAIT_S = 3600
EXPLAIN = re.compile(r'\b(why (did|have|are|do) you (stop|stopped|stopping|waiting|wait|stuck|not moving|standing)'
                     r'|why( did you)? stop|what happened|what went wrong|why (are|r) you stuck|explain yourself'
                     r'|why aren t you moving|why not moving|what s wrong)\b')
FIND = re.compile(r'\b(?:where is|where are|where s|where can i find|which room (?:has|contains|have)|'
                  r'in which room is|find)\s+(?:the |a |an |my |our )?(.+)$')
ROOM_WITH = re.compile(r'\b(?:room|place|lab|office)\s+(?:with|that has|which has|containing|where)\s+(?:the |a |an )?(.+?)'
                       r'(?:\s*(?:,|;|\bthen\b|\band\b|\bwait\b)|$)')
DOCK = re.compile(r'\b(go (and )?charge|charge (yourself|your battery|up)|go to (the )?(dock|charger|charging)|dock yourself|recharge)\b')
CONFIRM = re.compile(r'^\s*(ok|okay|done|confirm(ed)?|received|got it|you can go|go ahead)\s*$')
WAIT_OK = re.compile(r'\b(wait (for|until) (an? )?(ok|okay|confirmation|someone|me|them|him|her|signature)'
                     r'|until (someone|they|i|he|she) (press|presses|tap|taps|say|says|confirm|confirms)'
                     r'|get (it )?(signed|confirmed)|wait for (a )?reply)\b')
SPLIT = re.compile(r'\s*(?:,|;|\bthen\b|\band then\b|\bafter that\b|\bfollowed by\b|\band\b)\s*')


class CommandParser:
    def __init__(self, places, groq_api_key=None, model=DEFAULT_MODEL, timeout=6.0, http_post=None,
                 policy='smart', log=None, client=None, clock=time.monotonic):
        self.places = places
        self.log = log if log is not None else (client.log if client else LLMLog())
        self.client = client or GroqClient(groq_api_key, model=model, timeout=timeout, log=self.log,
                                           post=http_post)
        self.policy = policy if policy in POLICIES else 'smart'
        self.clock = clock
        self._cache = {}

    @property
    def key(self):
        return self.client.key

    @property
    def model(self):
        return self.client.model

    # ---------------------------------------------------------------- public
    def parse(self, text):
        text = (text or '').strip()
        if not text:
            return self._result('unknown', [], 1, 'Say where I should go.', 'rules', [])
        rules = self._parse_rules(text)
        if rules['intent'] == 'stop':                       # safety: never wait for the network
            return self._llm_info(rules, used=False, why='stop is always handled offline')
        if self.policy == 'off':
            return self._llm_info(rules, used=False, why='LLM turned off (llm_policy: off)')
        if not self.client.key:
            return self._llm_info(rules, used=False, why='no GROQ_API_KEY')
        if self.policy == 'smart' and self._rules_sure(rules, text):
            self.log.add(purpose='command', model=self.client.model, outcome='skipped', input=text[:160],
                         output=f'rules: {rules["intent"]} {rules["targets"]}', ms=0)
            return self._llm_info(rules, used=False, why='the offline rules were sure (no LLM call needed)')
        key = (normalize(text), self._places_sig())
        hit = self._cache.get(key)
        if hit and self.clock() - hit[0] < CACHE_S:
            self.log.add(purpose='command', model=self.client.model, outcome='cached', input=text[:160],
                         output=f'{hit[1]["intent"]} {hit[1]["targets"]}', ms=0)
            res = json.loads(json.dumps(hit[1]))
            res['llm'] = dict(res.get('llm') or {}, cached=True)
            return res
        t0 = self.clock()
        try:
            res = self._parse_llm(text)
        except LLMError as e:
            return self._fallback(rules, e)
        except Exception as e:                              # anything unexpected -> rules, never crash
            return self._fallback(rules, LLMError('error', f'{type(e).__name__}: {e}'))
        ms = round((self.clock() - t0) * 1000)
        if res['intent'] == 'unknown' and rules['intent'] != 'unknown':
            rules['notes'].append('LLM said "unknown"; kept the offline answer')
            return self._llm_info(rules, used=True, ms=ms, why='LLM was unsure, rules were not')
        res = self._llm_info(res, used=True, ms=ms)
        self._cache[key] = (self.clock(), json.loads(json.dumps(res)))
        if len(self._cache) > 200:
            self._cache.pop(next(iter(self._cache)))
        return res

    def _rules_sure(self, r, text=''):
        """The offline answer is complete and unambiguous: no reason to pay for an LLM round trip."""
        if r['intent'] in ('confirm', 'explain', 'status', 'dock', 'return_home'):
            return True
        if r['intent'] == 'find':
            return bool(r.get('query'))
        if r['intent'] == 'navigate':
            # every destination-looking part of the sentence must have become a target
            return bool(r['targets']) and not r['notes'] and self._destination_parts(text) <= len(r['targets'])
        return False

    @staticmethod
    def _destination_parts(text):
        """How many separate destinations the sentence seems to name (commas kept, unlike normalize())."""
        low = re.sub(r"[^a-z0-9,;' ]+", ' ', (text or '').lower())
        m = NAV_VERB.search(low)
        if not m:
            return 0
        n = 0
        for chunk in SPLIT.split(low[m.end():]):
            chunk = ' '.join((chunk or '').replace("'", ' ').split())
            for pat in (WAIT, WAIT_OK, HOME, URGENT, LOW):
                chunk = pat.sub(' ', chunk)
            chunk = re.sub(r'\b(?:me|us|this|these|them|it|to|into|in|at|towards?|the|a|an|my|our|please|'
                           r'there|here|and|then|for|of|go|come|back)\b', ' ', chunk)
            if chunk.strip():
                n += 1
        return n

    def _fallback(self, rules, err):
        rules['notes'].insert(0, f'LLM unavailable ({err.kind}); used offline parser')
        return self._llm_info(rules, used=True, error={'kind': err.kind, 'status': err.status,
                                                       'detail': err.detail[:200], 'hint': err.hint})

    def _llm_info(self, res, used, ms=None, why='', error=None):
        res['llm'] = {'used': used, 'model': self.client.model if used else None, 'ms': ms, 'why': why,
                      'error': error, 'policy': self.policy}
        return res

    def _places_sig(self):
        return getattr(self.places, 'version', None) or len(self.places.places) + len(self.places.rooms)

    # ---------------------------------------------------------------- rules
    def _parse_rules(self, text):
        t = normalize(text)
        prio = 3 if URGENT.search(t) else (0 if LOW.search(t) else 1)
        if CONFIRM.search(t):
            return self._result('confirm', [], 1, 'Thank you.', 'rules', [])
        if EXPLAIN.search(t):
            return self._result('explain', [], 1, '', 'rules', [])
        if STOP.search(t) and not WAIT_OK.search(t):
            return self._result('stop', [], 3, 'Stopping.', 'rules', [])
        if DOCK.search(t):
            return self._result('dock', [], prio, 'Going to charge.', 'rules', [])
        fm = FIND.search(t)
        if fm and not NAV_VERB.search(t[:fm.start()]) and not STATUS.search(t):
            res = self._result('find', [], 1, '', 'rules', [])
            res['query'] = fm.group(1).strip()
            return res
        if STATUS.search(t) and not NAV_VERB.search(t):
            return self._result('status', [], 1, '', 'rules', [])
        found = [(self._pos_in(t, f), f) for f in self.places.find_in_text(text)]
        notes = []
        for rm in ROOM_WITH.finditer(t):                 # "the room with the 3D printer"
            hits = self.places.find_thing(rm.group(1))
            if hits:
                tgt = self.places.room_target(hits[0][1])
                if tgt['name'] not in [f['name'] for _, f in found]:
                    found.append((rm.start(), tgt))
        if NAV_VERB.search(t):
            tail_at = NAV_VERB.search(t).end()
            tail = t[tail_at:]
            for chunk in [c for c in SPLIT.split(tail) if c and c.strip()]:
                at = t.find(chunk, tail_at)
                chunk = re.sub(r'^(?:(?:me|us|this|it|these)\s+)?(?:(?:to|into|in|at|towards?)\s+)?(?:(?:the|a|an|my|our)\s+)?', '', chunk.strip())
                if not chunk:
                    continue
                if any(at <= ps < at + len(chunk) for ps, _ in found) or self.places.find_in_text(chunk):
                    continue
                if ROOM_WITH.search(chunk) or WAIT.search(chunk) or WAIT_OK.search(chunk) or HOME.search(chunk):
                    continue
                hits = self.places.search(chunk, limit=1, min_score=0.6)
                if hits:                                  # "go to principl offce" (typo)
                    found.append((at, hits[0][1]))
                    continue
                things = self.places.find_thing(chunk, min_score=0.75)
                if things:                                # "take me to the printer" (a thing, not a place)
                    found.append((at, self.places.room_target(things[0][1])))
                    notes.append(f'"{chunk}" is in {things[0][1]["name"]}')
                elif len(chunk.split()) <= 5:
                    notes.append(f'unknown place: "{chunk}"')
        found.sort(key=lambda pf: pf[0])
        seen, uniq = set(), []
        for _, f in found:
            if f['name'] not in seen:
                seen.add(f['name'])
                uniq.append(f)
        found = uniq
        if found:
            names = [p['name'] for p in found]
            res = self._result('navigate', names, prio, 'Going to ' + ' then '.join(names) + '.', 'rules', notes)
            res.update(self._steps_from_text(text, found))
            if res['then_home']:
                res['reply'] = res['reply'][:-1] + ', then coming back.'
            return res
        if HOME.search(t):
            return self._result('return_home', [], prio, 'Returning home.', 'rules', [])
        hint = notes[0] if notes else 'I could not find a known place in that.'
        return self._result('unknown', [], prio, hint, 'rules', notes)

    # ---------------------------------------------------------------- llm
    def _schema(self, names):
        target = {'type': 'string', 'enum': names} if names else {'type': 'string'}
        return {'type': 'object', 'additionalProperties': False,
                'required': ['intent', 'query', 'targets', 'priority', 'reply', 'waits', 'then_home', 'confirms'],
                'properties': {
                    'intent': {'type': 'string', 'enum': list(INTENTS)},
                    'query': {'type': ['string', 'null']},
                    'targets': {'type': 'array', 'items': target},
                    'priority': {'type': 'integer', 'enum': [0, 1, 2, 3]},
                    'reply': {'type': 'string'},
                    'waits': {'type': 'array', 'items': {'type': 'number'}},
                    'then_home': {'type': 'boolean'},
                    'confirms': {'type': 'array', 'items': {'type': 'boolean'}}}}

    def _parse_llm(self, text):
        catalog = []
        for p in self.places.places:
            entry = {'name': p['name']}
            if p.get('aliases'):
                entry['aliases'] = p['aliases']
            room = self.places.get_room(p['name'])       # a room with the same name: pass on its contents
            if room and room.get('contents'):
                entry['room_contains'] = room['contents']
            catalog.append(entry)
        catalog += [{'name': r['name'], 'aliases': r.get('aliases', []), 'room_contains': r.get('contents', [])}
                    for r in self.places.rooms if not self.places.get(r['name'])]
        names = [c['name'] for c in catalog]
        system = (
            'You control an indoor campus robot. Convert the user command (English or Hinglish) into JSON with keys: '
            'intent (one of navigate, return_home, dock (go and charge), stop, status, '
            'explain (user asks why the robot stopped / what went wrong), '
            'find (user asks where something is), confirm (user says ok/done), unknown), '
            'query (for find: the thing asked about, else null), '
            'targets (ordered list of place names, ONLY copied exactly from the provided list), '
            'priority (0 low, 1 normal, 2 high, 3 urgent), '
            'reply (one short friendly sentence to say back), '
            'waits (list of seconds to wait at each target, same order as targets, 0 if none), '
            'then_home (true if the user wants the robot to come back / return afterwards), '
            'confirms (list of true/false per target: true if the robot must wait there until someone presses OK). '
            'A target can be a room; if the user names a thing, pick the room whose room_contains has it. '
            'Never invent places. If a requested place is not in the list, leave it out and say so in reply. '
            'If the command is not about the robot, use intent unknown. '
            'Known places: ' + json.dumps(catalog, separators=(',', ':')))
        d = self.client.chat_json('command', [{'role': 'system', 'content': system},
                                              {'role': 'user', 'content': text}],
                                  schema=self._schema(names), max_tokens=500, input_text=text)
        return self.build(d, text, 'llm')

    def build(self, d, text, source='llm'):
        """Model output -> a checked command. Everything a model says passes through here (also the
        conversation agent's actions): unknown places dropped, numbers clamped, urgent only with urgent words."""
        intent = d.get('intent', 'unknown')
        if intent not in INTENTS:
            intent = 'unknown'
        notes, names = [], []
        if intent in ('explain', 'confirm', 'dock'):
            return self._result(intent, [], 1, str(d.get('reply', ''))[:200], source, [])
        if intent == 'find':
            res = self._result('find', [], 1, '', source, [])
            res['query'] = str(d.get('query') or '')[:80]
            return res
        for raw in d.get('targets', []) or []:
            p = self.places.target(str(raw))
            if p is None:
                hits = self.places.search(str(raw), limit=1, min_score=0.85)
                p = hits[0][1] if hits else None
            if p is None:
                notes.append(f'ignored place not on the map: "{raw}"')     # hallucination guard
            elif p['name'] not in names:
                names.append(p['name'])
        if intent == 'navigate' and not names:
            intent = 'unknown'
        try:
            prio = max(0, min(3, int(d.get('priority', 1))))
        except (TypeError, ValueError):
            prio = 1
        if prio == 3 and not URGENT.search(normalize(text)):   # urgent pre-empts other people's trips:
            prio = 2                                            # only when the words say so
            notes.append('LLM asked for urgent priority without an urgent word; used high instead')
        reply = str(d.get('reply', ''))[:200]
        res = self._result(intent, names, prio, reply, source, notes)
        if intent == 'navigate':
            from_text = self._steps_from_text(text, [self.places.target(n) for n in names])
            waits = d.get('waits') if isinstance(d.get('waits'), list) else []
            confirms = d.get('confirms') if isinstance(d.get('confirms'), list) else []
            for i, st in enumerate(from_text['steps']):
                try:
                    w = float(waits[i]) if i < len(waits) else 0.0
                except (TypeError, ValueError):
                    w = 0.0
                st['wait_s'] = max(st['wait_s'], min(MAX_WAIT_S, max(0.0, w)))
                st['confirm'] = bool(st.get('confirm')) or (i < len(confirms) and confirms[i] is True)
            res['steps'] = from_text['steps']
            res['then_home'] = bool(d.get('then_home')) or from_text['then_home']
        return res

    # ---------------------------------------------------------------- multi-step plans
    def _steps_from_text(self, text, places):
        """'take this to Lab 3, wait 2 mins, then come back' -> steps with waits + then_home."""
        t = normalize(text)
        pos = []
        for p in places:
            best = None
            for label in [p['name']] + list(p.get('aliases', [])):
                m = re.search(r'\b' + re.escape(normalize(label)) + r'\b', t) if normalize(label) else None
                if m and (best is None or m.start() < best):
                    best = m.start()
            pos.append(best)
        steps = [{'place': p['name'], 'wait_s': 0.0, 'confirm': False} for p in places]
        for m in WAIT.finditer(t):
            n = m.group(1)
            val = NUM_WORDS.get(n, None) if not n[0].isdigit() else float(n)
            unit = m.group(2)
            secs = val * (60 if unit[0] == 'm' else 3600 if unit[0] == 'h' else 1)
            before = [i for i, ps in enumerate(pos) if ps is not None and ps < m.start()]
            i = before[-1] if before else 0
            if steps:
                steps[i]['wait_s'] = float(min(MAX_WAIT_S, secs))
        for m in WAIT_OK.finditer(t):
            before = [i for i, ps in enumerate(pos) if ps is not None and ps < m.start()]
            if steps:
                steps[before[-1] if before else 0]['confirm'] = True
        first = min([ps for ps in pos if ps is not None], default=0)
        home = [m for m in HOME.finditer(t) if m.start() > first]
        return {'steps': steps, 'then_home': bool(home and steps)}

    @staticmethod
    def _pos_in(t, p):
        best = len(t)
        for label in [p['name']] + list(p.get('aliases', [])):
            n = normalize(label)
            m = re.search(r'\b' + re.escape(n) + r'\b', t) if n else None
            if m:
                best = min(best, m.start())
        return best

    @staticmethod
    def _result(intent, targets, priority, reply, source, notes):
        return {'intent': intent, 'targets': targets, 'priority': priority, 'reply': reply,
                'source': source, 'notes': notes,
                'steps': [{'place': n, 'wait_s': 0.0, 'confirm': False} for n in targets], 'then_home': False}


def _http_post_json(url, body, headers, timeout):
    """Kept for old callers: JSON POST with the robot's User-Agent (see llm.py for why it matters)."""
    return http_json(url, body, headers, timeout)
