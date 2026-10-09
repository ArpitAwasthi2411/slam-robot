# Staying connected when the robot goes to another lab

The problem: the phone and the robot share one Wi-Fi. When the robot drives to another lab or floor
it leaves that Wi-Fi's range and you lose the app.

Two things fix it.

## 1. The robot doesn't need you to be connected

Missions (Go to Lab 3, Take this to HOD then come back) and Map by itself run **on the Pi**. The
phone only sends the order and watches. If the link drops:
- the robot **keeps going** and finishes the mission or the exploration,
- the app shows "Link to the robot lost. It carries on with its job by itself; reconnecting…" and
  reconnects by itself when the link is back,
- only joystick driving stops (by design: the motors stop 0.5 s after the last joystick message).

Safety without the app: the **RC remote** always overrides, and the **mushroom e-stop** on the robot
cuts the motors in hardware. Someone should walk with the robot (or be at the destination) whenever
it drives through public corridors.

## 2. Give the robot its own internet: a phone riding on the robot

Put a spare Android phone with a SIM (mobile data) on the robot and turn on its **hotspot**. The Pi
joins it and gets internet wherever the phone has signal. Then use **Tailscale** (free, a private
network between your devices) to reach the Pi from anywhere: your own phone on mobile data,
the college Wi-Fi, home.

```
 your phone (any network) ──Tailscale──► internet ◄── 4G ── robot phone hotspot ◄── Wi-Fi ── Pi
```

Bonus: with internet on the robot, typed commands use the LLM instead of the offline parser.

### What you need
- A spare Android phone with mobile data (any old phone works; keep it charged, or power it from
  the robot's 5 V buck converter through a USB cable).
- A free Tailscale account (sign in with Google): https://tailscale.com

### Setup (once)

**Robot phone:** Settings → Hotspot → name e.g. `RobotNet`, a WPA2 password, *turn off "switch off
hotspot automatically"* (Android turns the hotspot off when no device is connected for a while).
Turn the hotspot on.

**Pi** (over ethernet from the laptop, `ssh arpit@169.254.1.2`):
```bash
cd ~/slam-robot
sudo bash scripts/setup_wifi.sh hotspot "RobotNet" "hotspot-password"   # joins the robot phone
sudo bash scripts/setup_wifi.sh extra "CSE-Lab-WiFi" "lab-password"      # optional: lab Wi-Fi as backup
sudo bash scripts/setup_tailscale.sh                                     # prints a login link
```
Open the printed link on any device and log in with your Tailscale account. The script then shows
the robot's Tailscale name **slam-robot** and its address `100.x.y.z`.

**Start on boot** (so the robot is reachable after power-on without SSH):
```bash
sudo cp ~/slam-robot/scripts/robot.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now robot
```

**Your phone (the one with Pathik):** install the **Tailscale** app from the Play Store, log in with
the same account, switch it on. In Tailscale's admin page (login.tailscale.com → DNS) make sure
**MagicDNS** is on.

**Pathik:** Find robot (it tries `slam-robot` by itself), or type `slam-robot` (or the 100.x
address) in "Or type its address".

The laptop can join too (install Tailscale, same account): `ssh arpit@slam-robot` works from
anywhere, and RViz can stay on the ethernet cable when you're next to the robot.

### Check it
1. Turn off your phone's Wi-Fi so it uses only mobile data.
2. Pathik → type `slam-robot` → Connect. You should see the live map.
3. Status tab: the link line shows the round-trip time (over 4G, 80–300 ms is normal; the app is
   built for that, joystick driving included, but drive gently over 4G).

### Costs and limits
- The robot sends the map and scan to the app while it's open: roughly 20–60 MB per hour of
  watching. Nothing is sent when the app is closed.
- Lifts and basements may have no 4G: the robot keeps working there, the app reconnects later.
- If the robot phone's hotspot turns off, the Pi falls back to any other network you added with
  `setup_wifi.sh extra`.

## Which setup when

| Situation | Use |
|---|---|
| Demo in one room | Robot's own Wi-Fi (`setup_wifi.sh ap`, join SLAM-Robot) or your phone's hotspot |
| Robot goes between labs, you walk with it | Your phone's hotspot (phone travels with you and the robot) |
| Robot goes alone to another lab/floor | Phone on the robot as hotspot + Tailscale (this page) |
| Lab has good Wi-Fi everywhere | `setup_wifi.sh extra` for the lab Wi-Fi + Tailscale |
