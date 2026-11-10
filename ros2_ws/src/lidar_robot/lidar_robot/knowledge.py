"""What the robot knows about the college, the department and itself (pure Python, unit-tested).

A small notebook of facts the team writes in the app (Operate -> Knowledge): each entry is a title, a
few sentences and optional tags. The conversation agent (chat.py) looks up the entries that fit a
question and answers ONLY from them when the question is about the college, so it cannot make up a
teacher's name or a timing. Without internet the best entry is read out as it is.

Stored in ~/maps/knowledge.json (next to places.json). Search is plain word matching with weights:
no model, no internet, a few milliseconds on the Pi.
"""
import difflib
import json
import math
import os
import re
import time

MAX_ENTRIES = 300
MAX_TITLE = 80
MAX_TEXT = 1200
MAX_TAGS = 8

# Facts about the robot itself: always there, cannot be deleted (ids start with "pathik-").
BUILT_IN = [
    {'id': 'pathik-who', 'title': 'Who is Pathik',
     'text': 'I am Pathik, an indoor campus service robot. I was built by final-year engineering students as '
             'their major project. I find my way with a laser scanner (LiDAR) and a map of the building.',
     'tags': ['name', 'robot', 'who are you', 'kaun ho', 'introduce']},
    {'id': 'pathik-can', 'title': 'What Pathik can do',
     'text': 'I can guide you to a room, carry small things between rooms, visit several places in one trip, wait '
             'at a place until someone presses OK, tell you where a thing is kept, explain why I stopped, and go '
             'back to my charging dock by myself. You can talk to me in English or Hinglish, or use the Pathik app.',
     'tags': ['help', 'features', 'abilities', 'kya kar sakte ho', 'what can you do', 'use']},
    {'id': 'pathik-safety', 'title': 'Is Pathik safe',
     'text': 'I move at walking pace or slower. I stop for people and obstacles using my laser scanner, ultrasonic '
             'sensors and camera. There is a red emergency stop button on me, and STOP in the app always works.',
     'tags': ['safe', 'safety', 'stop', 'emergency', 'hit', 'collision']},
]

STOP_WORDS = set('''a an the is are was were be been am i you he she it we they me my your our their this that these
those of to in on at for from with by and or but if then so as do does did have has had can could will would shall
should may might please tell about what which who whom whose when where why how there here any some no not yes
kya hai hain ka ki ke ko se me mein par aur ya to bhi ye yeh wo woh kaun kab kahan kaise kyu kyun kitna kitne
mujhe hume batao bata do na ji hi tha thi the ho hu hoon'''.split())


def _tokens(text):
    return [t for t in re.sub(r"[^a-z0-9ऀ-ॿ ]+", ' ', (text or '').lower()).split()
            if t not in STOP_WORDS and len(t) > 1]


def _clean(s, limit):
    return ' '.join(str(s or '').split())[:limit]


class Knowledge:
    def __init__(self, path=None):
        self.path = os.path.expanduser(path) if path else None
        self.entries = []
        self.version = 0
        self._index = None
        self.load()

    # ---------------------------------------------------------------- storage
    def load(self):
        self.entries = []
        if self.path and os.path.exists(self.path):
            try:
                with open(self.path) as f:
                    raw = json.load(f)
                for e in raw.get('entries', []) if isinstance(raw, dict) else []:
                    rec = self._record(e.get('title'), e.get('text'), e.get('tags'))
                    if rec and not str(e.get('id', '')).startswith('pathik-'):
                        rec['id'] = str(e.get('id') or self._new_id())
                        rec['t'] = e.get('t')
                        self.entries.append(rec)
            except (OSError, ValueError, AttributeError):
                self.entries = []
        self._changed(save=False)

    def save(self):
        if not self.path:
            return
        os.makedirs(os.path.dirname(self.path) or '.', exist_ok=True)
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump({'entries': self.entries}, f, indent=1, ensure_ascii=False)
        os.replace(tmp, self.path)

    def _changed(self, save=True):
        self.version += 1
        self._index = None
        if save:
            self.save()

    def _new_id(self):
        used = {e['id'] for e in self.entries}
        n = len(self.entries) + 1
        while f'k{n}' in used:
            n += 1
        return f'k{n}'

    @staticmethod
    def _record(title, text, tags):
        title, text = _clean(title, MAX_TITLE), _clean(text, MAX_TEXT)
        if not title or not text:
            return None
        if isinstance(tags, str):
            tags = tags.split(',')
        tags = [_clean(t, 30).lower() for t in (tags or []) if _clean(t, 30)][:MAX_TAGS]
        return {'title': title, 'text': text, 'tags': tags}

    # ---------------------------------------------------------------- editing (admin, from the app)
    def all(self):
        return BUILT_IN + self.entries

    def get(self, entry_id):
        return next((e for e in self.all() if e['id'] == entry_id), None)

    def add(self, title, text, tags=()):
        rec = self._record(title, text, tags)
        if rec is None:
            raise ValueError('a title and some text are needed')
        if len(self.entries) >= MAX_ENTRIES:
            raise ValueError(f'the notebook is full ({MAX_ENTRIES} entries): delete some first')
        rec['id'] = self._new_id()
        rec['t'] = round(time.time())
        self.entries.append(rec)
        self._changed()
        return rec

    def update(self, entry_id, title=None, text=None, tags=None):
        e = next((x for x in self.entries if x['id'] == entry_id), None)
        if e is None:
            raise ValueError('built-in entries cannot be changed' if str(entry_id).startswith('pathik-')
                             else 'no such entry')
        rec = self._record(e['title'] if title is None else title, e['text'] if text is None else text,
                           e['tags'] if tags is None else tags)
        if rec is None:
            raise ValueError('a title and some text are needed')
        e.update(rec, t=round(time.time()))
        self._changed()
        return e

    def remove(self, entry_id):
        n = len(self.entries)
        self.entries = [e for e in self.entries if e['id'] != entry_id]
        if len(self.entries) == n:
            raise ValueError('built-in entries cannot be deleted' if str(entry_id).startswith('pathik-')
                             else 'no such entry')
        self._changed()

    # ---------------------------------------------------------------- search
    def _build(self):
        docs, df = [], {}
        for e in self.all():
            w = {}
            for tok in _tokens(e['title']) + [t for tag in e['tags'] for t in _tokens(tag)]:
                w[tok] = w.get(tok, 0.0) + 3.0                 # a word in the title or a tag counts 3x
            for tok in _tokens(e['text']):
                w[tok] = w.get(tok, 0.0) + 1.0
            docs.append((e, w))
            for tok in w:
                df[tok] = df.get(tok, 0) + 1
        n = max(1, len(docs))
        idf = {tok: math.log(1.0 + n / c) for tok, c in df.items()}
        self._index = (docs, idf)

    def search(self, query, limit=4, min_score=0.18):
        """-> [(score 0..1, entry)], best first. Score = how much of the question the entry covers."""
        q = _tokens(query)
        if not q:
            return []
        if self._index is None:
            self._build()
        docs, idf = self._index
        vocab = list(idf)
        terms = []
        for tok in dict.fromkeys(q):
            if tok in idf:
                terms.append((tok, tok, 1.0))
            else:                                              # a typo or another word form: nearest known word
                near = difflib.get_close_matches(tok, vocab, n=1, cutoff=0.82)
                terms.append((tok, near[0], 0.8) if near else (tok, None, 0.0))
        total = sum((idf.get(known, 1.5) if known else 1.5) for _, known, _ in terms) or 1.0
        out = []
        for e, w in docs:
            got = sum(idf[known] * f * min(1.0, 0.5 + w[known] / 6.0)
                      for _, known, f in terms if known and known in w)
            score = got / total
            if score >= min_score:
                out.append((round(score, 3), e))
        out.sort(key=lambda se: -se[0])
        return out[:limit]

    def summary(self):
        return {'count': len(self.entries), 'built_in': len(BUILT_IN), 'version': self.version,
                'entries': [dict(e, built_in=e['id'].startswith('pathik-')) for e in self.all()]}
