# Control Pathik from a phone on another network (mobile data)

The set-up for the next run: the **robot** rides on **phone A's hotspot** (that phone stays with the robot)
and **phone B** controls it from anywhere, on mobile data or another Wi-Fi. Two ways to do it:

| | Private: Tailscale on phone B (recommended) | Public link: Tailscale Funnel |
|---|---|---|
| Phone B needs | the Tailscale app, logged into your Tailscale account | nothing, only the Pathik app (or a browser) |
| Address in Pathik | `slam-robot` (or `100.x.y.z`) | `https://slam-robot.<tailnet>.ts.net` |
| Who can reach it | only devices in your Tailscale account | anyone with the link (a login is still required) |
| Turn on | once (step 2) | per demo: `funnel on`, then `funnel off` |

Either way the robot keeps its own safety: the e-stop, the RC override, the 0.5 s command watchdog in the
bridge and the ESP32, and the app pauses the joystick when the link is slow (see "Link delay" below).

---

## 1. Robot on phone A's hotspot (once)

Phone A: turn on its hotspot (name + password). Pi (ssh over the ethernet cable, `ssh arpit@169.254.1.2`):
```bash
sudo bash ~/slam-robot/scripts/setup_wifi.sh hotspot "HotspotName" "password"
```
Optional, a network for when the hotspot is off (e.g. the lab Wi-Fi). The robot always prefers the hotspot
and switches back to it within ~30 s when it comes back:
```bash
sudo bash ~/slam-robot/scripts/setup_wifi.sh fallback "LabWiFi" "password"
```
Check:
```bash
sudo bash ~/slam-robot/scripts/setup_wifi.sh status
```
`robot-hotspot` priority 20 is active, `hotspot-first timer: on`.

## 2. Tailscale on the robot (once)

You need a free Tailscale account (tailscale.com, log in with Google/GitHub/Microsoft).

**a) In the Tailscale admin page** (login.tailscale.com/admin):
- **DNS** → turn on **MagicDNS** (so `slam-robot` works as a name).
- Headless set-up (no browser on the Pi): **Settings → Keys → Generate auth key**: reusable off,
  pre-approved on, expiry 90 days. Copy the `tskey-auth-…` key.

**b) On the Pi** (needs internet, which the hotspot gives it):
```bash
nano ~/robot.env
```
Add the line `TS_AUTHKEY=tskey-auth-...` (this file never goes to GitHub), save, then:
```bash
sudo bash ~/slam-robot/scripts/setup_tailscale.sh
```
It installs Tailscale, logs in with the key (or prints a login URL if there is no key), starts it on every
boot and prints the robot's name and 100.x address.

**Check:**
```bash
sudo bash ~/slam-robot/scripts/setup_tailscale.sh status
```
`state: Running`, `starts at boot: enabled`, `Pathik app answers: yes`.

## 3a. Phone B with Tailscale (private, recommended)

1. Install **Tailscale** from the Play Store, log in with the **same** account, switch it on.
2. Open **Pathik** → type `slam-robot` in the address box → **Connect**. (No MagicDNS? Type the 100.x address
   from step 2.)
3. Log in with your Pathik account. The top bar says **Internet (Tailscale) · NNN ms**.

## 3b. Phone B without Tailscale (public link, for a demo)

Only when phone B can't install Tailscale (e.g. a visitor's phone).

1. First, on the robot's own network, make sure an **admin account exists** in Pathik (the script refuses
   otherwise, so a stranger can't become admin) and give the visitor a **staff** account (Me → People).
2. Allow Funnel once in the Tailscale admin page: **DNS → HTTPS Certificates: Enable**, and in
   **Access controls** add the `funnel` node attribute (Tailscale shows a one-click link the first time
   `funnel on` runs).
3. Pi:
   ```bash
   sudo bash ~/slam-robot/scripts/setup_tailscale.sh funnel on
   ```
   It prints `https://slam-robot.<tailnet>.ts.net`.
4. Phone B: Pathik → type that full `https://…` address → **Connect** → log in. (A browser works too:
   open the same address.) The top bar shows a red **PUBLIC** tag and Me shows a red warning.
5. After the demo:
   ```bash
   sudo bash ~/slam-robot/scripts/setup_tailscale.sh funnel off
   ```

What protects the robot on the public link: logins are mandatory (if `require_login` were false the API
refuses public requests), HTTPS end to end, sessions made over the public link last 7 days instead of 30,
wrong passwords are rate-limited per address (30 per 15 min) and per account over the public link
(20 per 15 min, the owner on the robot's Wi-Fi is never locked out), sign-ups limited to 5 per hour per
address, and the first (admin) account can't be created over the public link.

## Link delay and the joystick

The app shows the link in the top bar (`Internet (Tailscale) · 180 ms`, Lab → Health → Link delay).
Driving by hand needs a fresh, fast link:

- above **600 ms** delay, or **1.5 s** without an answer from the robot, the joystick greys out with
  "Driving by hand paused" and the app sends one stop;
- the app never queues joystick commands (a late command is worse than none);
- the robot itself stops 0.5 s after the last command anyway (bridge + ESP32 watchdogs).

On a slow link, send the robot to places instead (Go tab, or type/say a command): missions run on the robot
and don't need the link.

## Problems

| What you see | Do |
|---|---|
| Pathik: "No robot answered at slam-robot" | Tailscale on in the phone? Same account? MagicDNS on? Try the 100.x address. |
| `setup_tailscale.sh status`: state `NeedsLogin` | the auth key expired or was single-use: new key in `~/robot.env`, run the script again |
| Funnel: "Funnel not available" / link to the admin page | do step 3b.2 (HTTPS certificates + funnel attribute), then `funnel on` again |
| Robot drops off the hotspot | phone A's hotspot must stay on and near the robot; `setup_wifi.sh status`; the fallback network takes over and the robot returns to the hotspot when it's back |
| Joystick "paused" all the time | the link is slower than 600 ms: use missions, or move phone A closer / to better signal |
