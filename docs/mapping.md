# Mapping a whole floor accurately (tens of rooms, long corridors)

The saved map is the ground truth for everything after it: places, rooms, routes, docking. A corridor that
comes out 3 % short or slightly bent puts every place at its far end in the wrong spot. This is the method
for a map that measures within ~1 % of the real building.

## Why maps distort (so the steps make sense)

| What you see | Real cause | Fixed by |
|---|---|---|
| a long corridor too short / too long | two parallel walls say nothing about *how far along* the corridor the robot is; only the wheels do | good wheel distance (step 1), Cartographer told to trust it (config) |
| a corridor bends / rooms rotate | heading drift: gyro or wheel turn error, fast turns | gyro_scale (step 1), slow turns |
| double walls, smeared walls | turning faster than the 7–10 Hz LiDAR can follow, or late data from an overloaded Pi | slow driving, `throttled=0x0`, localization afterwards |
| the end of the run doesn't meet the start | no loop closure | drive back through places you've already mapped |
| one part of the map folded onto another | a *wrong* loop closure (corridors with identical doors) | stricter loop-closure score (config), offline rebuild |

Live mapping on the Pi is a compromise: two cores, real time. **The accurate map is rebuilt afterwards from a
recording** with all cores and far more optimisation (`scripts/map_offline.sh`). The live map is only for
watching your progress.

## Step 1. Calibrate the odometry (30 min, once, and after changing wheels/tyres)

Nothing else matters if this is wrong.

1. **Encoders:** every GND connected. One wheel turn by hand ≈ 4970 ticks on that side.
2. **Wheel speed loop:** reset the tuning (`{"op": "defaults"}`), then on blocks
   `serial_probe.py --vel 200 200` must read 180–220 mm/s on both wheels.
3. **Distance (ticks per wheel turn), over 5 m, 3 times:**
   ```bash
   sudo systemctl stop robot
   python3 ~/slam-robot/tools/serial_probe.py --port /dev/ttyAMA0 --push --metres 5
   ```
   Tape 5.000 m along a straight wall, push the robot slowly, keep it parallel to the wall. Average the 3
   `ticks_per_rev_left/right` and put them in `config/robot_params.yaml`. They should agree within 0.5 %.
4. **IMU:** `?` on the ESP32 must say `imu=ok`; app Health → "Heading from: gyro + wheels".
5. **Gyro scale:** app → Operate → Drive tests → **Spin**, 0.4 rad/s, at least 3 turns, in a room with walls
   close by. With the gyro working the result is a suggested **gyro_scale**: put it under `esp32_bridge` in
   `robot_params.yaml`. Repeat until it says the gyro counts right (within 1 %).
6. **Straight line:** Drive tests → Straight, 0.15 m/s: under 3 cm sideways drift per metre.
7. Rebuild + restart (`colcon build …`, `sudo systemctl restart robot`).

## Step 2. Prepare the floor and the robot

- `vcgencmd get_throttled` = `0x0` under load, Pi below 70 °C (`tools/cpu_report.py`). An overloaded or
  under-powered Pi delivers late data, and late data smears the map.
- LiDAR rigid and level, nothing of the robot's body in its 360° view.
- Battery charged (voltage sag slows the motors unevenly late in the run).
- Doors of the rooms you want **open**; few people around (early morning or after hours).
- Plan the route on paper: the corridor loop first, then rooms.

## Step 3. Record the mapping drive

Start the robot in **mapping** mode (the normal `ROBOT_ARGS`, no `slam_mode`), then in a second ssh window:

```bash
bash ~/slam-robot/scripts/map_offline.sh record cse_floor
```

Drive with the app joystick:

- **Slow:** ≤ 0.15 m/s straight, turns ≤ 0.3 rad/s. Stop for a second before and after each turn.
- **Start somewhere distinctive** (a corner, not the middle of a corridor) and stand still 5 s at the start.
- **Corridors first, as loops.** Drive the whole corridor system and come back to the start along a different
  way where you can. Arriving back at a place already mapped is what lets Cartographer correct the drift.
- **Rooms:** drive in through the door, go to the middle, turn slowly once all the way round, drive out
  the same door. Room by room.
- **Long straight corridors:** keep to the middle, at a steady speed; don't stop and turn on the spot there.
- **End where you started**, facing the same way, and stand still 5 s. Then Ctrl+C the recording.
- Too many people walked through? Do that part again later as a separate recording.

Watch the live map in the app while driving: if it's already badly wrong, stop and fix the cause (step 1/2)
instead of finishing the run.

## Step 4. Build the accurate map

```bash
sudo systemctl stop robot
bash ~/slam-robot/scripts/map_offline.sh build cse_floor
```

This runs Cartographer offline with `config/cartographer_mapping_hq.lua` (all 4 cores, twice as many
loop-closure checks, a long final whole-map optimisation) and writes `~/maps/cse_floor.pbstream`
(for the robot) and `~/maps/cse_floor.pgm/.yaml` (a picture for the report). A 15-minute recording takes
roughly 10–30 minutes on the Pi. Nothing is lost if it's slow: the recording stays in `~/bags/`, and you can
rebuild it any time with different settings.

## Step 5. Run on it (frozen)

Put the line the build printed into `~/robot.env`:

```
ROBOT_ARGS=esp32_port:=/dev/ttyAMA0 lidar_port:=/dev/ttyUSB0 slam_mode:=localization map:=/home/arpit/maps/cse_floor.pbstream
```

`sudo systemctl restart robot`, drive a little until the orange dots sit on the walls, **Map → Position OK**.
Add places and rooms **now**, on this map (places from a different map are in a different frame).

## Step 6. Check the accuracy (write the numbers in the report)

With a tape or laser measure, measure three real distances, as long as possible:

1. the longest corridor, end wall to end wall;
2. across the widest room;
3. between two doors far apart on the floor.

Measure the same three in the app (Map → **Measure**). Error = |app − real| ÷ real.

| Error | Verdict |
|---|---|
| < 1 % | good: freeze it |
| 1–3 % | usable; redo step 1 (distance + gyro scale) before the next map |
| > 3 %, or bent corridors | odometry or Pi problem: fix, then record again |

Also look at the picture: straight walls straight, corridors parallel, room walls single (not doubled).

## Settings behind this (for the report)

`config/cartographer_odom.lua` (live) and `cartographer_mapping_hq.lua` (offline, includes the live one):

- `use_online_correlative_scan_matching = false`: the real-time correlative search picks the best-looking
  match anywhere in its window; in a corridor that can be several cm further along, every scan.
- Ceres scan matcher `translation_weight = 20`, `rotation_weight = 40`: stay close to the odometry
  prediction unless the walls clearly say otherwise (the old values 2 / 10 were for broken odometry).
- Pose graph `odometry_translation_weight = odometry_rotation_weight = 1e5`: between nodes, trust the wheels
  and the gyro (the old `1e3` for rotation let corridors bend).
- `submaps.num_range_data = 90`, `max_range = 10`: more wall in each submap, the corridor's end wall included.
- Loop closures `min_score 0.65`, global `0.70`: a missed closure leaves a little drift, a wrong one folds the
  map, so be strict.
- Offline only: 4 threads, `optimize_every_n_nodes 20`, `constraint_builder.sampling_ratio 0.5`,
  `max_num_final_iterations 400`.
