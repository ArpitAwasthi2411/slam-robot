"""Groq LLM client used by the command parser and the error recovery (pure Python, tested).

Why this exists (Nov 2026): the LLM never worked on the robot, even with a valid key, because
  1. the models we asked for were retired by Groq on 16 Aug 2026 (llama-3.1-8b-instant,
     llama-3.3-70b-versatile): every call failed with model_decommissioned / not found;
  2. Groq sits behind Cloudflare, which rejects Python's default "Python-urllib/3.x" User-Agent
     (403, "error code: 1010"), so even a good key and model were refused;
  3. the error was swallowed: the app only ever saw "LLM unavailable (HTTPError)".

This module fixes all three: an explicit User-Agent, current models with automatic fallback when a
model is retired, and every call recorded in an LLMLog (what was sent, how long it took, what came
back, the exact error and what to do about it) that the app shows under Operate -> AI commands.

    log = LLMLog()
    client = GroqClient(key, model='openai/gpt-oss-20b', log=log)
    data = client.chat_json('command', messages, schema=SCHEMA, input_text=text)   # dict, or raises LLMError
    client.diagnose()            # key / network / model check for the app's "Test" button
"""
import collections
import json
import re
import socket
import time
import urllib.error
import urllib.request

API = 'https://api.groq.com/openai/v1'
CHAT_URL = API + '/chat/completions'
MODELS_URL = API + '/models'
USER_AGENT = 'pathik-robot/1.0 (+https://github.com/ArpitAwasthi2411/slam-robot)'

DEFAULT_MODEL = 'openai/gpt-oss-20b'          # commands: fast (~1 s), strict JSON schema
RECOVERY_MODEL = 'openai/gpt-oss-120b'        # error recovery: bigger, not time critical
FALLBACK_MODELS = ('openai/gpt-oss-20b', 'openai/gpt-oss-120b')
RETIRED = {'llama-3.1-8b-instant': 'openai/gpt-oss-20b', 'llama-3.3-70b-versatile': 'openai/gpt-oss-120b',
           'llama3-8b-8192': 'openai/gpt-oss-20b', 'llama3-70b-8192': 'openai/gpt-oss-120b',
           'gemma2-9b-it': 'openai/gpt-oss-20b', 'qwen/qwen3-32b': 'openai/gpt-oss-120b'}
STRICT_SCHEMA_MODELS = ('openai/gpt-oss-', 'qwen/qwen3.8')

HINTS = {
    'no_key': 'No GROQ_API_KEY on the robot. Put the line GROQ_API_KEY=gsk_... in ~/robot.env '
              '(no quotes, no "export"), then: sudo systemctl restart robot',
    'bad_key_format': 'GROQ_API_KEY does not look like a Groq key (gsk_...). Copy it again from console.groq.com.',
    'auth': 'Groq rejected the key (401). It was deleted or mistyped: create a new key at console.groq.com, '
            'put it in ~/robot.env and restart the robot.',
    'blocked': 'Groq\'s firewall (Cloudflare) refused the request (403). Usually the User-Agent or the network\'s '
               'IP: try another network (phone hotspot).',
    'forbidden': 'Groq refused the request (403): the key may not have access to this model or the account is '
                 'limited. Check console.groq.com.',
    'model': 'The model is retired or unknown on Groq. Set llm_model / recovery_llm_model in robot_params.yaml '
             '(see console.groq.com/docs/models). The robot tried the fallback models.',
    'rate_limit': 'Groq rate limit (429): too many requests on the free tier. Wait a minute.',
    'server': 'Groq server error (5xx): try again in a minute.',
    'timeout': 'Groq did not answer in time: slow internet. The offline parser was used.',
    'network': 'No connection to api.groq.com: the robot has no internet or no DNS.',
    'bad_output': 'The model answered with something that is not the JSON we asked for. The offline parser was used.',
    'bad_request': 'Groq said the request was invalid (400).',
    'error': 'Unexpected error talking to Groq.',
}


class LLMError(Exception):
    """kind: one of HINTS; status: HTTP status or None; detail: server message (key masked)."""

    def __init__(self, kind, detail='', status=None):
        super().__init__(f'{kind}: {detail}' if detail else kind)
        self.kind, self.detail, self.status = kind, detail, status

    @property
    def hint(self):
        return HINTS.get(self.kind, HINTS['error'])


def clean_key(raw):
    """Key from the environment, forgiving common ~/robot.env mistakes (quotes, 'export', 'KEY=')."""
    if not raw:
        return None
    k = str(raw).strip()
    k = re.sub(r'^export\s+', '', k)
    k = re.sub(r'^GROQ_API_KEY\s*=\s*', '', k)
    k = k.strip().strip('"\'').strip()
    return k or None


def mask(text, key):
    """Never let the key reach a log line or the app."""
    s = str(text)
    if key:
        s = s.replace(key, key[:4] + '…' + key[-4:] if len(key) > 10 else '…')
    return re.sub(r'gsk_[A-Za-z0-9]{8,}', 'gsk_…', s)


def classify_http(status, body):
    """(kind, detail) for an HTTP error from Groq."""
    text = body.decode('utf-8', 'replace') if isinstance(body, bytes) else str(body or '')
    msg, code = text.strip()[:300], ''
    try:
        err = json.loads(text).get('error') or {}
        msg = str(err.get('message') or msg)[:300]
        code = str(err.get('code') or err.get('type') or '')
    except (ValueError, AttributeError):
        pass
    low = (msg + ' ' + code).lower()
    if status == 401 or 'invalid_api_key' in low or 'invalid api key' in low:
        return 'auth', msg
    if status == 403:
        return ('blocked' if '1010' in low or 'cloudflare' in low or 'access denied' in low else 'forbidden'), msg
    if status == 404 or 'decommission' in low or 'model_not_found' in low or 'does not exist' in low \
            or ('model' in low and ('not found' in low or 'not supported' in low or 'deprecated' in low)):
        return 'model', msg
    if status == 429:
        return 'rate_limit', msg
    if status == 400 and ('json_validate_failed' in low or 'failed to generate json' in low
                          or 'failed to validate json' in low):
        return 'bad_output', msg
    if status and status >= 500:
        return 'server', msg
    if status == 400:
        return 'bad_request', msg
    return 'error', msg


def http_json(url, body=None, headers=None, timeout=6.0, method=None):
    """POST (or GET when body is None) JSON. Returns the parsed reply; raises LLMError."""
    h = {'User-Agent': USER_AGENT, 'Accept': 'application/json', **(headers or {})}
    data = None
    if body is not None:
        data = json.dumps(body).encode()
        h['Content-Type'] = 'application/json'
    req = urllib.request.Request(url, data=data, method=method or ('POST' if body is not None else 'GET'),
                                 headers=h)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            raw = r.read().decode()
    except urllib.error.HTTPError as e:
        try:
            payload = e.read()
        except Exception:      # noqa: BLE001
            payload = b''
        kind, detail = classify_http(e.code, payload)
        raise LLMError(kind, detail, e.code) from None
    except (socket.timeout, TimeoutError) as e:
        raise LLMError('timeout', str(e)) from None
    except urllib.error.URLError as e:
        if isinstance(e.reason, (socket.timeout, TimeoutError)):
            raise LLMError('timeout', str(e.reason)) from None
        raise LLMError('network', str(e.reason)) from None
    except OSError as e:
        raise LLMError('network', str(e)) from None
    try:
        return json.loads(raw)
    except ValueError:
        raise LLMError('bad_output', 'reply is not JSON: ' + raw[:120]) from None


class LLMLog:
    """The last `size` LLM calls, newest first, plus running totals. Safe to show in the app."""

    def __init__(self, size=60, wall=time.time):
        self.items = collections.deque(maxlen=size)
        self.wall = wall
        self.totals = {'calls': 0, 'ok': 0, 'failed': 0, 'skipped': 0, 'cached': 0, 'ms_sum': 0.0}
        self.last_error = None
        self.last_test = None
        self._id = 0

    def add(self, **entry):
        self._id += 1
        e = {'id': self._id, 't': round(self.wall(), 1), **entry}
        self.items.appendleft(e)
        outcome = e.get('outcome')
        if outcome == 'ok':
            self.totals['calls'] += 1
            self.totals['ok'] += 1
            self.totals['ms_sum'] += float(e.get('ms') or 0)
        elif outcome == 'error':
            self.totals['calls'] += 1
            self.totals['failed'] += 1
            self.last_error = {k: e.get(k) for k in ('t', 'purpose', 'model', 'kind', 'status', 'detail', 'hint')}
        elif outcome == 'cached':
            self.totals['cached'] += 1
        elif outcome == 'skipped':
            self.totals['skipped'] += 1
        return e

    def snapshot(self, limit=30):
        t = dict(self.totals)
        t['avg_ms'] = round(t.pop('ms_sum') / t['ok']) if t['ok'] else None
        return {'totals': t, 'last_error': self.last_error, 'last_test': self.last_test,
                'calls': list(self.items)[:limit]}


class GroqClient:
    """chat_json() -> dict. Tries `model`, then the fallbacks when a model is retired/unknown.
    `post(url, body, headers, timeout)` and `get(url, headers, timeout)` can be injected for tests."""

    def __init__(self, key, model=DEFAULT_MODEL, fallbacks=FALLBACK_MODELS, timeout=6.0, log=None,
                 post=None, get=None, clock=time.monotonic):
        self.key = clean_key(key)
        model = (model or DEFAULT_MODEL).strip()
        self.requested_model = model
        self.model = RETIRED.get(model, model)
        self.fallbacks = [m for m in fallbacks if m != self.model]
        self.timeout = timeout
        self.log = log if log is not None else LLMLog()
        self._post = post or (lambda url, body, headers, timeout: http_json(url, body, headers, timeout))
        self._get = get or (lambda url, headers, timeout: http_json(url, None, headers, timeout))
        self.clock = clock
        self.dead_models = set()        # retired models found at run time: skipped from then on

    # ------------------------------------------------------------------ config
    def key_problem(self):
        if not self.key:
            return 'no_key'
        if not self.key.startswith('gsk_') or len(self.key) < 20 or ' ' in self.key:
            return 'bad_key_format'
        return None

    def config(self):
        k = self.key
        return {'key': (k[:4] + '…' + k[-4:]) if k and len(k) > 10 else ('set' if k else None),
                'key_problem': self.key_problem(), 'model': self.model, 'requested_model': self.requested_model,
                'fallbacks': self.fallbacks, 'timeout_s': self.timeout,
                'retired_model_replaced': self.requested_model != self.model}

    # ------------------------------------------------------------------ calls
    def _body(self, model, messages, schema, max_tokens, schema_ok):
        body = {'model': model, 'messages': messages, 'temperature': 0, 'max_completion_tokens': max_tokens}
        if 'gpt-oss' in model:
            body['reasoning_effort'] = 'low'        # least thinking -> lowest latency; enough for parsing
            body['include_reasoning'] = False       # don't send the reasoning back (fewer bytes, no leaks)
        if schema and schema_ok and model.startswith(STRICT_SCHEMA_MODELS):
            body['response_format'] = {'type': 'json_schema',
                                       'json_schema': {'name': 'robot_reply', 'strict': True, 'schema': schema}}
        else:
            body['response_format'] = {'type': 'json_object'}
        return body

    def chat_json(self, purpose, messages, schema=None, max_tokens=500, input_text=''):
        problem = self.key_problem()
        if problem == 'no_key':                   # a key that merely looks odd is still tried (Groq decides)
            err = LLMError(problem)
            self.log.add(purpose=purpose, model=self.model, outcome='error', kind=problem, status=None,
                         detail='', hint=err.hint, input=input_text[:160], ms=0)
            raise err
        models = [m for m in [self.model] + self.fallbacks if m not in self.dead_models] or [self.model]
        last = None
        for model in models:
            schema_ok = True
            for _attempt in range(2):            # second try only without the strict schema
                t0 = self.clock()
                try:
                    resp = self._post(CHAT_URL, self._body(model, messages, schema, max_tokens, schema_ok),
                                      {'Authorization': f'Bearer {self.key}', 'User-Agent': USER_AGENT},
                                      self.timeout)
                    content = self._content(resp)
                    data = json.loads(content)
                    if not isinstance(data, dict):
                        raise LLMError('bad_output', 'JSON is not an object')
                except LLMError as e:
                    last = e
                except (socket.timeout, TimeoutError) as e:       # injected posts may raise these
                    last = LLMError('timeout', str(e))
                except urllib.error.URLError as e:
                    last = LLMError('network', str(e.reason))
                except ValueError as e:
                    last = LLMError('bad_output', f'not valid JSON: {e}')
                except (KeyError, IndexError, TypeError) as e:
                    last = LLMError('bad_output', f'unexpected reply shape: {e}')
                except Exception as e:                            # noqa: BLE001
                    last = LLMError('error', f'{type(e).__name__}: {e}')
                else:
                    ms = round((self.clock() - t0) * 1000)
                    usage = resp.get('usage') or {}
                    self.log.add(purpose=purpose, model=model, outcome='ok', ms=ms, input=input_text[:160],
                                 output=mask(content, self.key)[:600], tokens=usage.get('total_tokens'),
                                 fallback=model != self.model, strict=bool(schema and schema_ok))
                    return data
                ms = round((self.clock() - t0) * 1000)
                self.log.add(purpose=purpose, model=model, outcome='error', kind=last.kind, status=last.status,
                             detail=mask(last.detail, self.key)[:300], hint=last.hint, input=input_text[:160], ms=ms)
                if last.kind == 'bad_output' and schema and schema_ok and last.status == 400:
                    schema_ok = False             # strict schema refused: retry once in plain JSON mode
                    continue
                break
            if last.kind == 'model':
                self.dead_models.add(model)       # retired: never ask for it again this run
                continue
            break                                 # auth / network / timeout ...: the next model won't help
        raise last

    @staticmethod
    def _content(resp):
        msg = resp['choices'][0]['message']
        content = msg.get('content')
        if not content or not str(content).strip():
            fin = resp['choices'][0].get('finish_reason')
            raise LLMError('bad_output', f'empty answer (finish_reason={fin})')
        return str(content)

    # ------------------------------------------------------------------ the app's "Test" button
    def diagnose(self):
        """Checks key format, the network, the key itself and which of our models exist. Never raises."""
        cfg = self.config()
        out = {'ok': False, 'config': cfg, 'steps': []}

        def step(name, ok, text):
            out['steps'].append({'name': name, 'ok': ok, 'text': text})
        problem = self.key_problem()
        if problem:
            step('key', False, HINTS[problem])
            out['summary'] = HINTS[problem]
            return out
        step('key', True, f'key {cfg["key"]} found in the environment')
        if cfg['retired_model_replaced']:
            step('model setting', False, f'{self.requested_model} is retired on Groq: using {self.model} instead '
                                         '(update robot_params.yaml)')
        t0 = self.clock()
        try:
            resp = self._get(MODELS_URL, {'Authorization': f'Bearer {self.key}', 'User-Agent': USER_AGENT},
                             self.timeout)
        except LLMError as e:
            step('reach Groq', False, f'{e.kind} ({e.status or "-"}): {mask(e.detail, self.key)[:160]}')
            out['summary'] = e.hint
            return out
        except Exception as e:                             # noqa: BLE001
            step('reach Groq', False, f'{type(e).__name__}: {e}')
            out['summary'] = HINTS['network']
            return out
        ms = round((self.clock() - t0) * 1000)
        step('reach Groq', True, f'api.groq.com answered in {ms} ms, key accepted')
        ids = {m.get('id') for m in (resp.get('data') or []) if isinstance(m, dict)}
        if ids:
            usable = [m for m in [self.model] + self.fallbacks if m in ids]
            missing = [m for m in [self.model] + self.fallbacks if m not in ids]
            step('models', bool(usable),
                 ('available: ' + ', '.join(usable) if usable else 'none of our models is available')
                 + (f' · not available: {", ".join(missing)}' if missing else ''))
            if self.model not in ids and usable:
                self.model = usable[0]
                step('model switched', True, f'using {self.model}')
            if not usable:
                out['summary'] = HINTS['model']
                return out
        out['ok'] = True
        out['summary'] = f'Groq works with {self.model}.'
        return out
