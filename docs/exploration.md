# Map by itself (autonomous exploration)

Driving fast with the RC remote makes smeared maps: the LiDAR sees each wall only briefly, often at
speed and while turning. With **Map by itself** the robot builds the map on its own:

1. It looks at the map for **open edges** (frontiers): places where mapped floor meets unmapped space.
2. It picks the next edge that's worth the trip: close by and big. Tiny slivers behind furniture are ignored.
3. It drives there **slowly** (0.12–0.25 m/s, you choose), along a planned route that keeps clear of walls.
4. At each spot it **turns once on the spot** (optional), so the LiDAR sees every wall from there.
5. It repeats until no reachable edge is left, then **drives back** to Home (or where it started),
   which also gives Cartographer a loop closure, and **saves the map** as `~/maps/explore_<date>_<time>`.

On the simulated CSE floor (15 × 10 m, 6 rooms + corridor) it maps every room in about 17 minutes at
0.18 m/s with the turns, or 12 minutes without them. What's left unmapped is only space the robot
can't fit into (gaps behind desks narrower than the robot).

## How to use it

1. Start the robot in **mapping** mode as usual (`robot_up`). Wait until the map shows the first room.
2. Clear the floor (bags, chair legs pushed in), **open the doors** of the rooms you want mapped.
   Closed doors are walls to the robot.
3. Pathik → **Drive** tab → scroll down to **Map by itself**:
   - **Careful 0.12 m/s**: narrow labs, many chairs, or the first run.
   - **Normal 0.18 m/s**: the default.
   - **Quick 0.25 m/s**: open corridors.
   - **Turn once at every spot**: on for the cleanest map. Off saves about 30 % of the time.
4. **Start mapping** → confirm. The card shows m² mapped, areas visited, open edges left and time.
   On the map, **magenta dots** are the open edges still to visit; the **dashed magenta ring** is
   where it's heading now.
5. When it's done you get a toast "Map saved as explore_…" and the robot is back where it started.
   Use that file for localization like any saved map.

**Stop mapping** stops at once and keeps the map so far (save it with *Save map* if you want it).
The big **STOP** still works as always. The RC remote always overrides the robot: grab the sticks
and the robot obeys you; let go and exploration carries on.

## What it does when something goes wrong

| Situation | What happens |
|---|---|
| Person or bag blocks the way | Waits, then plans a different route (up to the replan limit) |
| An edge can't be reached (too narrow, door closed) | Skips it and remembers not to try that spot again |
| A goal takes more than 2 minutes | Gives it up and picks another |
| Phone loses the Wi-Fi link | **Keeps exploring** (it runs on the Pi, not on the phone). Reconnect to watch again |
| Nothing reachable left | Finishes, returns, saves the map |

## Tips for clean maps

- Do one manual loop of the room with the remote first only if Cartographer struggles to start;
  otherwise let exploration do everything.
- Mirrors, glass doors and shiny cupboards confuse every LiDAR. Cover them with paper for the
  mapping run, or the map gets ghost walls.
- Avoid people walking right next to the robot during mapping: they become "walls" for a moment.
- After the run, open the map in the app: if a room is missing, its door was probably closed.
  Open it and press **Start mapping** again: it continues from the existing map.

## Settings (for the report / tuning)

In `lidar_robot/explorer.py`, `ExploreParams` (defaults chosen with the simulator benchmark):

| Setting | Default | Meaning |
|---|---|---|
| `min_frontier_cells` | 15 | ignore open edges shorter than about 0.75 m |
| `gain_per_cell` | 0.03 m | how much a bigger edge is worth compared with a longer drive |
| `standoff` | 0.05 m | extra clearance beyond the robot radius for each goal point |
| `search_radius` | 1.2 m | how far from the edge a safe goal point may be |
| `blacklist_radius` | 0.6 m | visited or failed goals block this area |
| `max_goal_time` | 120 s | give up on a goal after this |
| `scan_speed` | 0.4 rad/s | turn speed at each spot (about 16 s per turn) |

Request API (`/api/nav`, also from the command line):
```json
{"type": "explore_start", "speed": 0.18, "turn_speed": 0.45, "scan_spin": true}
{"type": "explore_stop"}
```
Status is in `/api/state` → `nav.explore` (`active`, `state` running/done/stopped, `goals_done`,
`skipped`, `frontiers` [[x, y, cells]…], `target`, `area` m², `elapsed_s`) and `autosave`
(`state` saving/saved/failed, `name`).

Algorithm: frontier cells are free cells next to unknown cells; they're grouped into clusters
(8-connected), each cluster gets the nearest point where the robot fits (distance transform of the
planner's map), candidates are ranked by straight-line distance minus a size bonus, and the best
few get a full A\* plan to pick the cheapest real route. Tests: `test/test_explorer.py` (maps a
two-room floor through a doorway, returns to start, turns a full circle at each stop, stop/cancel,
auto-save).
