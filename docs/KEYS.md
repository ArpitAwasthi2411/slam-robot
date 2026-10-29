# Keys, accounts and where they go

Nothing secret is ever in Git (this repo is public). Secrets live in **`~/robot.env`** on the Pi
(`chmod 600`), which `scripts/robot.service` loads (`EnvironmentFile=`) and the scripts read. For Docker the
same file is `~/pathik/robot.env` (`deploy/pi/`).

```bash
nano ~/robot.env
```
```bash
chmod 600 ~/robot.env
```
```bash
sudo systemctl restart robot
```

## `~/robot.env`

| Line | What it is for | Where to get it | Without it |
|---|---|---|---|
| `GROQ_API_KEY=gsk_...` | LLM command parsing (`llm_model`), **LLM error recovery** (`recovery_llm_model`), **Whisper speech-to-text** (voice), the laptop vision script | console.groq.com → API Keys → Create (free tier). **Create a new one**: the old key was visible in a screenshot | offline rules parser, rule-based recovery, Vosk offline speech (all still work; the app shows "Understood offline" / "Decided by the safety rules") |
| `TS_AUTHKEY=tskey-auth-...` | headless Tailscale login (`scripts/setup_tailscale.sh`) | login.tailscale.com → Settings → Keys → Generate auth key (pre-approved, 90 days) | the script prints a login URL to open once |
| `ROBOT_ARGS="..."` | extra launch args for the service, e.g. `slam_mode:=localization map:=/home/arpit/maps/cse.pbstream voice:=true` | – | defaults |
| `ROBOT_USER=...` / `ROBOT_PASSWORD=...` | the **MCP server** logs in to the robot with this account (`scripts/mcp_server.py`); put them in the MCP client's config on the laptop, not on the Pi | a Pathik account (staff is enough to send/ask; operator to drive) | the MCP tools report "login needed" |
| `PATHIK_PORT=8080` (optional) | dashboard port used by the scripts | – | 8080 |

Model names are settings, not secrets: `llm_model` (default `llama-3.1-8b-instant`), `recovery_llm_model`
(`llama-3.3-70b-versatile`) in `config/robot_params.yaml`, `stt_model` (`whisper-large-v3-turbo`) in the voice
section; `RECOVERY_MODEL` / `RECOVERY_MODE` env vars for `tools/eval_recovery.py` and the simulator.

## Accounts (not keys)

| Account | Used for | Where |
|---|---|---|
| **Pathik admin** (first sign-up on the robot) | approves people, roles | app → Operate → People. Create it on the robot's own network **before** turning on the public Funnel link (the script refuses otherwise) |
| Pathik operator / staff accounts | robot team / campus users | sign up in the app, approved by the admin; stored hashed in `~/maps/users.json` |
| Tailscale account | private remote access; Funnel | tailscale.com (Google/GitHub login); same account on the Pi and on the controlling phone |
| GitHub (ArpitAwasthi2411) | CI, APK releases, Docker image (ghcr.io) | make the GHCR package public before `scripts/pathik.sh install` on the Pi |
| Android signing keystore (optional) | stable APK updates | GitHub repo secrets `PATHIK_KEYSTORE_B64`, `PATHIK_KEYSTORE_PASSWORD`, `PATHIK_KEY_ALIAS` (without them CI signs with a debug key) |

## Downloads that need internet once (no keys)
`scripts/setup_voice.sh` (Vosk models from alphacephei.com, Piper + voices from GitHub / Hugging Face),
`scripts/setup_vision.sh` (YOLO11n export), `scripts/setup_pi_camera.sh` (libcamera + imx708 driver),
`scripts/setup_tailscale.sh` (Tailscale). Run them while the Pi is on a hotspot with internet.
