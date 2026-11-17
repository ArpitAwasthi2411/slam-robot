# Building the map on a laptop riding on the robot

Mapping a whole department on the Pi 4 alone runs it at 100 % CPU: scans are matched late, the map tears and
saving times out (17 Nov). A laptop strapped to the robot and joined to the Pi by an **Ethernet cable** does
the heavy part. The Pi keeps everything that has to stay on the robot.

| | Pi | Laptop (on the robot, Ethernet) |
|---|---|---|
| LiDAR, ESP32 bridge (odometry 50 Hz), safety, e-stop | yes | |
| Navigator, app, voice, camera | yes | |
| Cartographer + map picture while **mapping** | no (`slam_host:=laptop`) | yes, finer settings (`cartographer_laptop.lua`) |
| Recording of the drive (`~/bags/laptop_<time>`) | | yes |
| RViz | | yes |
| Localization on a frozen map | yes (light) | not needed: the laptop can come off |

Only one Cartographer may run (both would publish `map -> odom`), so the Pi starts none while the laptop maps.

## One-time setup

On the laptop (Ubuntu 22.04, ROS 2 Humble):

```bash
sudo apt install ros-humble-cartographer-ros ros-humble-rviz2 rsync
git clone https://github.com/ArpitAwasthi2411/slam-robot.git ~/slam-robot   # or: cd ~/slam-robot && git pull
bash ~/slam-robot/scripts/setup_laptop_link.sh     # laptop = 169.254.1.1 on the cable, Pi = 169.254.1.2
ssh-copy-id arpit@169.254.1.2                      # maps are copied with rsync, without a password
echo 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0' >> ~/.bashrc
```

On the Pi, add `slam_host:=laptop` to `ROBOT_ARGS` in `~/robot.env`, then `sudo systemctl restart robot`.
(Remove it again to map on the Pi alone. Localization on a frozen map always runs on the Pi.)

## Every mapping drive

```bash
python3 ~/slam-robot/scripts/laptop_slam.py --check   # cable, SSH, ROS, Cartographer: "all good"
python3 ~/slam-robot/scripts/laptop_slam.py           # leave it running; RViz opens
```

Then use the app as usual: Map tab → New map / Continue mapping → drive → **Freeze map**. The script follows
the app by itself:

* starts Cartographer when the app is mapping, restarts it when the Pi restarts or another map is continued,
  stops it when the map is frozen;
* records the drive; rebuild it at full quality any time with
  `bash ~/slam-robot/scripts/map_offline.sh build laptop_<time>` (a few minutes on the laptop);
* every map the app saves (Freeze, the 3-minute autosave) is written on the laptop, copied to the Pi's
  `~/maps` and kept in `~/pathik_maps`. Freeze waits for the copy before the Pi switches to the frozen map.

## If something goes wrong

* **Cable out / laptop asleep:** the app's Map card says the laptop's map stopped. The robot drives on wheel
  odometry for up to `map_tf_timeout` (10 s) and then stops ("position lost"). Plug the cable back in: a short
  break continues the same map; after 15 s of silence the laptop stops, and the recording still has the drive.
* **"SSH needs a password":** `ssh-copy-id arpit@169.254.1.2` once.
* **Clocks:** the map only uses the Pi's timestamps, so a clock difference does not break it; the script
  prints it once a minute. To sync: both online, `sudo timedatectl set-ntp true` on both.
* **Wi-Fi:** if the laptop is also on the hotspot, robot data may go over Wi-Fi; `nmcli radio wifi off` keeps it
  on the cable while mapping.
* **See Cartographer's own messages:** `python3 scripts/laptop_slam.py --verbose`.
* **Compare with the Pi's settings:** `--quality pi`.

A laptop GPU does not help Cartographer (it is CPU only); any laptop with 4 cores is plenty.
