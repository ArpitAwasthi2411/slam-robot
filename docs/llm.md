# The language model on Pathik: why it never worked, and how it works now

Code: `lidar_robot/llm.py` (Groq client + log), `lidar_robot/commands.py` (command understanding),
`lidar_robot/recovery.py` (error recovery), app: **Operate → AI commands**. Tests: `test/test_llm.py`.

## 1. Why the LLM never worked, even with a correct key

Three separate faults, each one enough to break every call:

| # | Fault | What the robot did |
|---|---|---|
| 1 | **Retired models.** Groq shut down `llama-3.1-8b-instant` (commands) and `llama-3.3-70b-versatile` (recovery) on **16 Aug 2026**. Replacements: `openai/gpt-oss-20b` / `openai/gpt-oss-120b` ([Groq deprecations](https://console.groq.com/docs/deprecations)). | every call: HTTP 400/404 `model_decommissioned` |
| 2 | **Blocked User-Agent.** api.groq.com is behind Cloudflare, which rejects Python's default `Python-urllib/3.x` User-Agent with 403 "error code: 1010" ([Groq community](https://community.groq.com/t/cloudflare-blocking-urllib-request-without-user-agent/860)). Commands, recovery *and Whisper voice* all used plain `urllib`. | every call: HTTP 403, even with a good key and model |
| 3 | **Hidden error.** The parser caught the exception and kept only its class name, so the app showed "LLM unavailable (HTTPError)". No status, no message, nothing in the log. | silently used the offline rules |

Two more problems made it look broken even when Groq answered:

- **Timeouts in the chain.** The app gave `/api/nav` 4 s, and the dashboard waited 3 s for the navigator. An LLM
  command that took longer showed "robot did not answer in time", even though it was working.
- **The LLM call ran inside the navigator's control loop**, under the core lock. While Groq was thinking, the 20 Hz
  loop (path following, obstacle stop) was frozen for up to 4 s.

## 2. What changed

**Working calls**
- Every request sends `User-Agent: pathik-robot/1.0`. This is in `llm.http_json` and in the Whisper upload in `voice.py`.
- Default models are `openai/gpt-oss-20b` for commands and `openai/gpt-oss-120b` for recovery.
- An old model name in `robot_params.yaml` is mapped to its replacement automatically, with a warning in the log.
- If Groq retires a model at run time, the client tries the fallback model and never asks for the dead one again.
- The key is cleaned of common `~/robot.env` mistakes: quotes, `export`, `GROQ_API_KEY=` pasted twice.
  systemd's `EnvironmentFile` does **not** accept `export ...` lines, so write the line as `GROQ_API_KEY=gsk_...`.

**Lower latency**
- **`llm_policy: smart`** (default). The offline rules answer first. If they are sure, the LLM is not called at
  all, so that command has no network delay. "Sure" means a known place, home, dock, status, OK or
  "why did you stop", *and* every destination-looking part of the sentence became a target. Only unsure commands
  (Hinglish, things instead of places, unknown words, several destinations) go to the LLM. `always` sends
  everything to the LLM; `off` never calls it.
- **"Stop" never waits for the network** in any policy.
- **Cache.** The app previews a command (dry run) and then sends it. The second parse reuses the first answer
  (10 min, cleared when the places change), so there is one LLM call per command instead of two.
- **Small, fast requests.** `reasoning_effort: low` and `include_reasoning: false` (gpt-oss), `temperature 0`,
  a compact place catalog and `max_completion_tokens 500`.
- **Off the control loop.** The node parses commands and the AI test in a worker thread; the core lock is held
  only to act on the result. The app now waits 20 s for a command and the dashboard waits 16 s.

**Hallucination and unwanted output**
1. **Constrained output.** A strict JSON schema (`response_format: json_schema, strict: true`): `intent` is an
   enum, `targets` items are an **enum of the map's place and room names**, `priority` is one of 0–3 and booleans
   are booleans. The model cannot produce a place that is not on the map
   ([Groq structured outputs](https://console.groq.com/docs/structured-outputs)).
2. **Re-validation.** Every answer is checked again in `commands.py`:
   - unknown places are dropped and noted;
   - values are clamped;
   - waits are capped at 1 h;
   - "navigate" with no valid place becomes "unknown".
3. **Urgent needs urgent words.** Priority 3 pre-empts other people's trips, so the LLM may only set it when the
   sentence has an urgent word ("urgent", "asap", "jaldi", "turant" …). Otherwise it is lowered to 2, with a note.
4. **Unsure LLM never overrides confident rules.** An LLM "unknown" never replaces a rules answer that found something.
5. **Recovery stays menu-only.** The LLM picks one of 8 fixed actions; numbers are clamped and the rules decide
   when the LLM is late or wrong (`recovery.py`, unchanged).
6. **Fallback on any failure.** No key, retired model, network, timeout, refused schema or empty answer → the
   offline parser, with the reason attached to the result. A refused schema (`json_validate_failed`) is retried
   once in plain JSON mode first.

**Debugging from the app** (Operate → **AI commands**, robot team only)
- **Status card:**
  - AI ready or not;
  - the key (masked: `gsk_…abcd`), its problem if any;
  - models for commands and recovery, and a note when a retired model was replaced;
  - the policy;
  - the **last error** with its HTTP status, Groq's message and what to do.
- **Counters:** LLM calls, worked, failed, average time, "rules were sure" (calls saved), "remembered" (cache hits).
- **Test AI button:**
  - checks the key format;
  - reaches api.groq.com (`/models`), which proves network, firewall and key in one call;
  - checks that our models exist;
  - runs one sample command through the LLM (your own sentence, or a default).

  Every step shows ✓ or ✗ with a reason.
- **Every call**, newest first: what was sent, model, time, tokens, the raw JSON answer, or the error (kind, status,
  message, fix). The key never appears.
- **On the Send screen:** "Understood by the language model in 0.8 s", "(remembered)", or
  "Understood offline (AI failed)" plus a toast with the reason.
- **Robot log:** `journalctl -u robot | grep -i llm` shows the start-up line (model, policy) and every failure with
  its fix.

## 3. Error kinds you can see

| kind | Means | Fix |
|---|---|---|
| `no_key` | no `GROQ_API_KEY` in the robot's environment | `GROQ_API_KEY=gsk_...` in `~/robot.env` (no quotes, no `export`), `sudo systemctl restart robot` |
| `auth` (401) | key deleted or mistyped | new key at console.groq.com |
| `blocked` (403, 1010) | Cloudflare refused (User-Agent or the network's IP) | fixed for the User-Agent; if it persists, try a phone hotspot |
| `forbidden` (403) | account or model access | check console.groq.com |
| `model` (400/404) | model retired or unknown | `llm_model` / `recovery_llm_model` in `robot_params.yaml`; the fallback model was tried |
| `rate_limit` (429) | free-tier limit | wait a minute |
| `timeout` / `network` | slow or no internet / DNS | the offline parser handles it |
| `bad_output` | not the JSON asked for | logged with the raw answer; offline parser used |

## 4. Measuring it (for the report)

```bash
python3 tools/eval_commands.py                                  # offline rules only
GROQ_API_KEY=gsk_... python3 tools/eval_commands.py              # + LLM "always" and "smart"
GROQ_API_KEY=gsk_... python3 tools/eval_commands.py --csv results/commands.csv
```

For each run it prints accuracy per field (intent, targets in order, priority, waits, confirms, then_home), median
and worst latency, and every LLM error with its reason. Compare three things:
- **rules vs always:** what the LLM adds in accuracy, and what it costs in latency;
- **always vs smart:** how much latency smart saves for the same accuracy;
- **hallucination rate:** targets not on the map before the schema (old logs) vs after (0 by construction).

Recovery: `GROQ_API_KEY=... python3 tools/eval_recovery.py --policies llm,rules --model openai/gpt-oss-120b`.

## 5. Settings (`config/robot_params.yaml`, navigator)

```yaml
llm_model: openai/gpt-oss-20b
llm_policy: smart          # smart | always | off
llm_timeout: 6.0
recovery_mode: llm
recovery_llm_model: openai/gpt-oss-120b
```

Model names change. Check [console.groq.com/docs/models](https://console.groq.com/docs/models) and the
deprecations page when a model starts failing; the app's AI page will already say `model` and name it.
