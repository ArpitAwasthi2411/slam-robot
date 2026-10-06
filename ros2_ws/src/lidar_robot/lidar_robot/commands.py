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

INTENTS = ('navigate', 'return_home', 'stop', 'status', 'unknown')
GROQ_URL = 'https://api.groq.com/openai/v1/chat/completions'

URGENT = re.compile(r'\b(urgent|urgently|asap|immediately|emergency|right now|quickly|hurry|priority)\b')
LOW = re.compile(r'\b(low priority|when (you are |you\'re )?free|no rush|later|whenever)\b')
STOP = re.compile(r'\b(stop|halt|cancel|abort|freeze|emergency stop|wait there)\b')
HOME = re.compile(r'\b((go|come|return|head|get)\s+(back\s+)?(home|to base|to (the )?dock)|come back|go back|return|dock|go home)\b')
STATUS = re.compile(r'\b(where are you|status|what are you doing|are you busy|battery)\b')
NAV_VERB = re.compile(r'\b(go|goto|take|bring|deliver|drop|carry|navigate|move|drive|head|visit|reach|escort|guide|show)\b')
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
        if STOP.search(t):
            return self._result('stop', [], 3, 'Stopping.', 'rules', [])
        if STATUS.search(t) and not NAV_VERB.search(t):
            return self._result('status', [], 1, '', 'rules', [])
        found = self.places.find_in_text(text)
        notes = []
        if not found and NAV_VERB.search(t):
            # "go to principl offce" -> fuzzy match each chunk after the verb
            tail = NAV_VERB.split(t, maxsplit=1)[-1]
            for chunk in [c for c in SPLIT.split(tail) if c and c.strip()]:
                hits = self.places.search(chunk, limit=1, min_score=0.6)
                if hits:
                    found.append(hits[0][1])
                elif len(chunk.split()) <= 5:
                    notes.append(f'unknown place: "{chunk}"')
        if found:
            names = [p['name'] for p in found]
            return self._result('navigate', names, prio, 'Going to ' + ' then '.join(names) + '.',
                                'rules', notes)
        if HOME.search(t):
            return self._result('return_home', [], prio, 'Returning home.', 'rules', [])
        hint = notes[0] if notes else 'I could not find a known place in that.'
        return self._result('unknown', [], prio, hint, 'rules', notes)

    # ---------------------------------------------------------------- llm
    def _parse_llm(self, text):
        catalog = [{'name': p['name'], 'aliases': p.get('aliases', [])} for p in self.places.places]
        system = (
            'You control an indoor campus robot. Convert the user command into JSON with keys: '
            'intent (one of navigate, return_home, stop, status, unknown), '
            'targets (ordered list of place names, ONLY copied exactly from the provided list), '
            'priority (0 low, 1 normal, 2 high, 3 urgent), '
            'reply (one short friendly sentence to say back). '
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
        for raw in d.get('targets', []) or []:
            p = self.places.get(str(raw))
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
        return self._result(intent, names, prio, reply, 'llm', notes)

    @staticmethod
    def _result(intent, targets, priority, reply, source, notes):
        return {'intent': intent, 'targets': targets, 'priority': priority, 'reply': reply,
                'source': source, 'notes': notes}


def _http_post_json(url, body, headers, timeout):
    req = urllib.request.Request(url, data=json.dumps(body).encode(), method='POST',
                                 headers={'Content-Type': 'application/json', **headers})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode())
