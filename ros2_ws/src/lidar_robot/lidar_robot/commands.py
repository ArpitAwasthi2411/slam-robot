"""Natural-language robot commands -> structured intent, with an offline fallback (pure Python, tested).

    parser = CommandParser(place_store, groq_api_key=os.environ.get('GROQ_API_KEY'))
    cmd = parser.parse("urgent: take this to the HOD office and then lab 3")
    -> {'intent': 'navigate', 'targets': ['HOD office', 'Lab 3'], 'priority': 3,
        'reply': '...', 'source': 'llm' | 'rules', 'notes': [...]}

Fault tolerance (the gap in cloud-only systems like ELLMER):
  * no API key / no internet / timeout / bad JSON  -> rule-based parser, still works offline
  * LLM names a place that is not on the map        -> dropped (hallucination guard), noted
"""
import json
import re
import urllib.error
import urllib.request

from lidar_robot.places import normalize

INTENTS = ('navigate', 'return_home', 'dock', 'stop', 'status', 'explain', 'find', 'confirm', 'unknown')
GROQ_URL = 'https://api.groq.com/openai/v1/chat/completions'

URGENT = re.compile(r'\b(urgent|urgently|asap|immediately|emergency|right now|quickly|hurry|priority)\b')
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
    def __init__(self, places, groq_api_key=None, model='llama-3.1-8b-instant', timeout=4.0,
                 http_post=None):
        self.places = places
        self.key = groq_api_key
        self.model = model
        self.timeout = timeout
        self._post = http_post or _http_post_json

    # ---------------------------------------------------------------- public
    def parse(self, text):
        text = (text or '').strip()
        if not text:
            return self._result('unknown', [], 1, 'Say where I should go.', 'rules', [])
        if self.key:
            try:
                res = self._parse_llm(text)
                if res is not None:
                    return res
            except Exception as e:                      # network, quota, JSON ... -> fall back
                fallback_note = f'LLM unavailable ({type(e).__name__}); used offline parser'
                res = self._parse_rules(text)
                res['notes'].insert(0, fallback_note)
                return res
        return self._parse_rules(text)

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
    def _parse_llm(self, text):
        catalog = []
        for p in self.places.places:
            entry = {'name': p['name'], 'aliases': p.get('aliases', [])}
            room = self.places.get_room(p['name'])       # a room with the same name: pass on its contents
            if room and room.get('contents'):
                entry['room_contains'] = room['contents']
            catalog.append(entry)
        catalog += [{'name': r['name'], 'aliases': r.get('aliases', []), 'room_contains': r.get('contents', [])}
                    for r in self.places.rooms if not self.places.get(r['name'])]
        system = (
            'You control an indoor campus robot. Convert the user command into JSON with keys: '
            'intent (one of navigate, return_home, dock (go and charge), stop, status, '
            'explain (user asks why the robot stopped / what went wrong), '
            'find (user asks where something is), confirm (user says ok/done), unknown), '
            'query (for find: the thing asked about), '
            'targets (ordered list of place names, ONLY copied exactly from the provided list), '
            'priority (0 low, 1 normal, 2 high, 3 urgent), '
            'reply (one short friendly sentence to say back), '
            'waits (list of seconds to wait at each target, same order as targets, 0 if none), '
            'then_home (true if the user wants the robot to come back / return afterwards), '
            'confirms (list of true/false per target: true if the robot must wait there until someone presses OK). '
            'A target can be a room; if the user names a thing, pick the room whose room_contains has it. '
            'Never invent places. If a requested place is not in the list, leave it out and say so in reply. '
            'Known places: ' + json.dumps(catalog))
        body = {'model': self.model, 'temperature': 0, 'max_tokens': 200,
                'response_format': {'type': 'json_object'},
                'messages': [{'role': 'system', 'content': system},
                             {'role': 'user', 'content': text}]}
        resp = self._post(GROQ_URL, body, {'Authorization': f'Bearer {self.key}'}, self.timeout)
        content = resp['choices'][0]['message']['content']
        d = json.loads(content)
        intent = d.get('intent', 'unknown')
        if intent not in INTENTS:
            intent = 'unknown'
        notes, names = [], []
        if intent in ('explain', 'confirm', 'dock'):
            return self._result(intent, [], 1, str(d.get('reply', ''))[:200], 'llm', [])
        if intent == 'find':
            res = self._result('find', [], 1, '', 'llm', [])
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
        reply = str(d.get('reply', ''))[:200]
        res = self._result(intent, names, prio, reply, 'llm', notes)
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
    req = urllib.request.Request(url, data=json.dumps(body).encode(), method='POST',
                                 headers={'Content-Type': 'application/json', **headers})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode())
