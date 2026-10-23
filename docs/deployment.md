# Deploying Pathik on the campus

How the robot is set up so that **staff anywhere on the floor** can call it, **only approved people** can
use it, and **new software reaches the robot safely** (tested, versioned, with one-command rollback).

```
 Teacher's phone (college Wi-Fi or mobile data)         GitHub
        │ Pathik app, logged in as "staff"                 │ push → tests → Docker image (arm64)
        ▼                                                  ▼
   Tailscale (private network over the internet)     ghcr.io/arpitawasthi2411/slam-robot:latest
        ▲                                                  │ pathik.sh update (pull, health check,
        │                                                  ▼  auto-rollback)
   Robot: Pi ── Wi-Fi ── phone hotspot riding on the robot (4G) ── internet
          └─ container "pathik": LiDAR, ESP32 bridge, Cartographer, navigator, app server :8080
```

## 1. Network: one robot, a large floor, no single Wi-Fi

The robot does its jobs **on the Pi**, so a dropped link never stops a mission. It only needs a network
to *receive* orders and *report* status.

1. **Internet on the robot:** an old Android phone with a SIM rides on the robot as a hotspot; the Pi
   joins it (`sudo bash scripts/setup_wifi.sh hotspot "RobotNet" "password"`), with the lab Wi-Fi as a
   backup (`setup_wifi.sh extra ...`). Step by step: [remote_access.md](remote_access.md).
2. **Reach it from anywhere:** Tailscale on the Pi (`sudo bash scripts/setup_tailscale.sh`). The robot
   is then `slam-robot` on your private network, whatever Wi-Fi or mobile data anyone is on.
3. **Teachers' phones:** each teacher installs the Tailscale app once. Share the robot with them from
   the Tailscale admin page (*Machines → slam-robot → Share*): they accept the link with their own
   Google account and see only the robot, nothing else of yours. In Pathik they type `slam-robot`.

The app login (section 2) is the second lock: being on the network is not enough to drive the robot.

> **Optional, wider access:** `tailscale funnel 8080` gives the robot a public `https://…ts.net`
> address that opens in any browser without the Tailscale app (and HTTPS lets browser voice work).
> Only do this with accounts on, strong admin passwords, and turn it off (`tailscale funnel off`)
> when not demonstrating.

## 2. Accounts and roles

| Role | Who | Can |
|---|---|---|
| **staff** | teachers, office | send the robot to a place/room, voice and typed commands, "come back", "why did you stop?", charge, **press STOP**, see the map |
| **operator** | the robot team | everything staff can, plus drive with the joystick, map, edit places/rooms/home/dock, schedules, Tuning Lab, **release STOP** |
| **admin** | you | everything, plus approve sign-ups, change roles, disable, reset passwords, remove people |

- **First start:** the app shows *Set up this robot*. The first account becomes the **admin**.
- **Everyone else:** *Request access* in the app. The request waits; admins see a badge on **Me** and
  approve it under **Me → People**, choosing the role.
- Passwords are stored as salted PBKDF2 hashes in `~/maps/users.json` on the robot; logins expire
  after 30 days; 5 wrong passwords in a minute lock that login for a minute.
- **Forgot the admin password:** on the Pi
  `bash ~/pathik/pathik.sh users reset-admin arpit 'new-password'`
  (without Docker: `python3 -m lidar_robot.auth ~/maps/users.json reset-admin arpit 'new-password'`).
- The robot remembers **who** sent every mission (mission log column `by`).
- Laptop simulator with ready-made accounts: `python3 tools/dashboard_sim.py --demo-users`
  (admin/admin123, operator/operator123, staff/staff123). The demo inside the app has the same three.
- The MCP server (AI assistant) logs in with `ROBOT_USER` / `ROBOT_PASSWORD` ([mcp.md](mcp.md)).

## 3. The robot as a container (Docker)

Why: the same tested image runs on every robot, updates are one command, a bad update rolls back by
itself, and the Pi's own system stays clean (no `colcon build` on the robot).

**Install (once, Pi with internet):**
```bash
cd ~/slam-robot && bash scripts/pathik.sh install     # installs Docker, creates ~/pathik
# (log out and in once if it asks, then run it again)
nano ~/pathik/robot.env                               # ports, GROQ_API_KEY
bash ~/pathik/pathik.sh update                        # first start
```
The old `robot.service` is switched off by `install`; Docker now starts the robot at boot
(`restart: unless-stopped`).

| Command (on the Pi) | Does |
|---|---|
| `bash ~/pathik/pathik.sh update` | pulls the newest image, restarts, waits for the health check; **if unhealthy it rolls back by itself** |
| `bash ~/pathik/pathik.sh rollback` | back to the version before the last update |
| `bash ~/pathik/pathik.sh use sha-1a2b3c4` | run one exact version (every commit has its own tag) |
| `bash ~/pathik/pathik.sh status` / `logs` | which version runs, live logs |
| `bash ~/pathik/pathik.sh users list` | accounts |

What the compose file does (`deploy/pi/docker-compose.yml`):
- `network_mode: host`: ROS 2 discovery and the app on port 8080 behave exactly as without Docker.
- `/dev` mounted with **device cgroup rules** (`ttyUSB`, `ttyACM`, sound, camera) instead of a fixed
  device list: if a USB serial port drops and comes back (the motor-board reconnects), the container
  still sees it. `privileged` is not needed.
- `~/maps` mounted at `/root/maps`: maps, places, rooms, users, tuning and mission log survive updates.
- log rotation (3 × 10 MB) so the SD card doesn't fill up; `SIGINT` on stop so the motors stop cleanly.

**First pull of a private package:** GitHub creates the image package as private. Make it public once
(GitHub → your profile → *Packages* → `slam-robot` → *Package settings* → *Change visibility*), or on
the Pi `docker login ghcr.io` with a token that has `read:packages`.

## 4. The CI/CD pipeline

```
git push ──► test (pytest, ~1 min) ──► image: tests again INSIDE the Docker image
                                        ─► smoke test (image starts, packages installed)
                                        ─► build arm64 (Pi) + amd64 ─► push ghcr.io …:latest, :sha-xxxxxxx
pull request ──► same tests and build, nothing pushed (main always has a working image)
git tag v1.0.0 && git push --tags ──► also :1.0.0 / :v1.0.0 (a release you can always go back to)
Pi: pathik.sh update ──► pull ─► health check ─► keep, or roll back automatically
```

- Workflow: `.github/workflows/robot-image.yml`. The phone app has its own (`android.yml`, APK release).
- Build cache (GitHub Actions cache): ROS and the LiDAR driver are cached layers, so a code change
  only rebuilds the last layers.
- See a run: GitHub → *Actions* → *Robot image*. The summary shows the pushed tag.

**Words for the report / viva:** *continuous integration* (every change is tested automatically),
*continuous delivery* (every passing change becomes a versioned, deployable image), *immutable
deployments* (the robot runs an exact image tag, not a hand-built folder), *health-checked rollout with
automatic rollback*, *least-privilege containers* (device cgroup rules instead of `--privileged`),
*secrets outside the image* (`robot.env` on the robot, never in Git).

**Kubernetes?** Not on the robot: a Pi 4 needs its CPU and memory for SLAM. Kubernetes (e.g. k3s) fits
the **fleet side**: a server that collects status from many robots. One robot does not need it.
