# Error recovery (LLM + rules): what the robot does when a step fails

Code: `lidar_robot/recovery.py` (menu, validation, rules, LLM client), `navigator_core.py` (`_fail` →
`_start_recovery` → `_recovery_tick` → `_apply_recovery`), app card on Home, `tools/eval_recovery.py`.

## The loop

1. A mission step fails with a recoverable code: `NO_PATH`, `START_BLOCKED`, `BLOCKED`, `GOAL_IN_OBSTACLE`,
   `OUTSIDE_MAP`, `POSE_LOST`, `DOCK_NO_CONTACT`, `NO_MAP`. (E-stop, exploration goals and patrol routes keep
   their old behaviour.)
2. The robot **stops** (state `RECOVERING`, speed 0) and builds the context:
   failure code + text, the stop, the mission (all stops, which one is next, priority, who asked), the queue,
   battery (% and level), time of day, robot position and room, **history of this place** (earlier failures
   there and what worked), tries already used, detour candidates (saved places from which the stop is
   reachable, with route lengths), and the list of actions that are possible right now.
3. **Decide** (`recovery_mode: llm`): the LLM (Groq, `recovery_llm_model`, default `llama-3.3-70b-versatile`)
   gets the context and must answer JSON `{"action", "params", "reason", "say"}`. It runs in a thread, so the
   control loop keeps running (robot stopped). If there is no key, no internet, a timeout (6 s + 2 s), an
   action that is not on the menu or not possible now, or bad JSON, the **rules** decide instead and the
   card says why (`note`). `recovery_mode: rules` skips the LLM; `off` = the old "mission FAILED" behaviour.
4. **Validate and clamp**: unknown actions are rejected (never executed), numbers are clamped
   (`wait_s` 3–120, `margin_m` 0.03–0.25), `via` must be one of the listed detours, questions ≤ 160 chars.
5. **Act**, log it in the mission log (`recoveries` per mission, with source and reason), say it on the
   speaker, show it in the app ("What happened / What I'm doing / Why", source: language model, safety
   rules, or a person).
6. **Outcome**: reaching the stop = `recovered`; skip / postpone / give up are recorded as such; another
   failure = `failed_again` and the next decision knows it (tries, place history). At most
   `recovery_max_per_mission` (4) decisions per mission, then the mission fails as before.

## The menu (the only things the robot can choose)

| action | params | what the robot does | possible when |
|---|---|---|---|
| `wait_retry` | `wait_s` 3–120 | stays stopped, then plans again | always |
| `replan_wide` | `margin_m` 0.03–0.25 | plans again keeping more distance from obstacles | BLOCKED |
| `alt_route` | `via` (one of the listed detours) | drives via a saved place first, then to the stop | detours exist |
| `skip_stop` | – | gives up this stop, continues with the next | trip has more stops |
| `reorder` | – | lets the queued tasks go first (or moves this stop to the end of the trip) | queue or more stops |
| `ask_person` | `question` | waits; the app shows the question with Try again / Wait 1 min / Skip / Go home / Go and charge / Cancel | always |
| `go_home` | – | ends the mission, drives home | home is set |
| `go_dock` | – | ends the mission, goes to charge | dock is set |

"Ask a person": any logged-in role (staff too) can answer from the Home card, the classic API
(`{"type": "recovery_answer", "id": N, "choice": "retry|wait|skip|home|dock|cancel"}`) or the MCP tool
`answer_robot`. Nobody answers within `recovery_ask_timeout_s` (120 s) → the rules decide without asking.

## Rule-based policy (fallback)

Battery low/critical → go to the dock (or home). Otherwise an escalation ladder per failure code; the n-th
recovery at the same stop takes the n-th possible step:

- BLOCKED: wait 20 s → detour → wider route → reorder → skip → ask
- NO_PATH: wait 15 s → detour → reorder → skip → ask
- GOAL_IN_OBSTACLE / OUTSIDE_MAP: skip → ask
- POSE_LOST: wait 20 s → ask · START_BLOCKED, NO_MAP: wait → ask · DOCK_NO_CONTACT: wait 5 s → wait 10 s → ask

## Evaluation

```bash
python3 tools/eval_recovery.py                     # none vs rules (vs llm when GROQ_API_KEY is set)
GROQ_API_KEY=... python3 tools/eval_recovery.py --rounds 3 --model llama-3.3-70b-versatile
```
Six scripted failures on the simulated floor (person in the corridor 25 s, trolley parked in a door, door
blocked 100 s, a saved place inside furniture, position lost 40 s, blocked corridor with a low battery).
Metrics: success, time to recover (LLM thinking time added), recoveries used, unnecessary go-home/go-dock
(battery fine), questions to a person. Output: `results/recovery_eval.{md,csv,json}`.

First run (rules only, no key in the cloud session): **no recovery 1/6, rules 6/6**, 45.9 s average time to
recover, 0 unnecessary go-homes, 1 question to a person. Run it again with the key on the laptop or the Pi to
fill the LLM column (Arpit, before the report / ROSCon numbers).

## Settings (`config/robot_params.yaml`, navigator)

```yaml
recovery_mode: llm                 # llm | rules | off
recovery_llm_model: llama-3.3-70b-versatile
recovery_max_per_mission: 4
recovery_ask_timeout_s: 120.0
```
Key: `GROQ_API_KEY` in `~/robot.env` (the same key the command parser uses). The node log says at start-up
which mode is active (`error recovery llm (rules: no GROQ_API_KEY)` when the key is missing).
