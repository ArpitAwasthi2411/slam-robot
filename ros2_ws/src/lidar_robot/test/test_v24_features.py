"""v2.4: body-shape collision checks, battery, dock, schedules, wait-for-OK, mission log, rooms, explain."""
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from lidar_robot.battery import BatteryMonitor, percent  # noqa: E402
from lidar_robot.commands import CommandParser  # noqa: E402
from lidar_robot.footprint import Footprint, FootprintGuard, check_path  # noqa: E402
from lidar_robot.kinematics import wrap_angle  # noqa: E402
from lidar_robot.navigator_core import NavigatorCore  # noqa: E402
from lidar_robot.places import PlaceStore  # noqa: E402
from lidar_robot.planner import GridMap, Planner, PlannerParams  # noqa: E402
from lidar_robot.schedules import ScheduleStore  # noqa: E402

RES = 0.05
FP = Footprint(front=0.07, rear=0.37, half_width=0.21)


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def floor():
    """8 x 6 m: wall at x=4 with a 0.9 m door (y 2.6..3.5), a table in the left room."""
    w, h = int(8 / RES), int(6 / RES)
    g = np.zeros((h, w), dtype=np.int16)

    def box(x0, y0, x1, y1):
        g[int(y0 / RES):int(y1 / RES) + 1, int(x0 / RES):int(x1 / RES) + 1] = 100
    box(0, 0, 8, 0.05)
    box(0, 5.95, 8, 6)
    box(0, 0, 0.05, 6)
    box(7.95, 0, 8, 6)
    box(4.0, 0, 4.05, 2.6)
    box(4.0, 3.5, 4.05, 6)
    box(1.5, 1.0, 2.5, 2.0)
    return GridMap.from_occupancy(w, h, RES, 0.0, 0.0, g.ravel().tolist()), g


def occupied_points(g):
    ys, xs = np.nonzero(g >= 65)
    return np.stack([(xs + 0.5) * RES, (ys + 0.5) * RES], axis=1)


def scan_base(occ, pose, r=1.2):
    x, y, yaw = pose
    d = occ - np.array([x, y])
    near = d[(np.abs(d[:, 0]) < r) & (np.abs(d[:, 1]) < r)]
    c, s = math.cos(yaw), math.sin(yaw)
    return [(c * dx + s * dy, -s * dx + c * dy) for dx, dy in near]


def body_hits(occ, pose, fp=FP):
    return any(fp.signed_distance(px, py) < 0 for px, py in scan_base(occ, pose, 0.6))


# ------------------------------------------------------------------ footprint geometry
def test_signed_distance_and_outline():
    assert FP.signed_distance(0.0, 0.0) < 0
    assert abs(FP.signed_distance(0.17, 0.0) - 0.10) < 1e-9          # 10 cm in front of the bumper
    assert abs(FP.signed_distance(-0.47, 0.0) - 0.10) < 1e-9         # 10 cm behind the back
    assert abs(FP.circumradius - math.hypot(0.37, 0.21)) < 1e-9
    assert all(abs(FP.signed_distance(x, y)) < 1e-9 for x, y in FP.outline())


def test_turn_on_the_spot_is_refused_when_the_back_would_hit():
    # wall 15 cm to the robot's right, beside the back half: turning left swings the back into it
    wall = [(x, -0.36) for x in np.arange(-0.45, 0.1, 0.03)]
    g = FootprintGuard(FP, wall)
    assert not g.rotation_clear(math.pi / 2)            # left turn: back swings right, into the wall
    assert g.rotation_clear(-0.3)                       # small right turn swings the back away
    assert g.motion_clear(0.1, 0.0)                     # driving straight along it is fine


def test_check_path_flags_a_route_hugging_a_wall():
    grid, _ = floor()
    p = Planner(PlannerParams(robot_radius=0.27))
    p.set_map(grid)
    path = [(x, 0.40) for x in np.arange(0.6, 3.5, 0.1)]               # side 0.14 m off the south wall
    assert check_path(p, path, FP, start_yaw=0.0) == []                  # straight along it: body fits
    turn = [(0.8, 0.40)] + [(0.8, 0.40 + k * 0.1) for k in range(1, 15)]
    assert check_path(p, turn, FP, start_yaw=0.0)                        # 90° turn there: back hits


# ------------------------------------------------------------------ closed loop: no wall contact
def drive(core, occ, pose, goal_name, seconds=90.0, guard=True):
    dt = 0.05
    clock = core.clock
    core.handle({'type': 'goto_place', 'name': goal_name}, pose)
    hits = 0
    for _ in range(int(seconds / dt)):
        if guard:
            core.update_scan_base(scan_base(occ, pose))
        v, w = core.step(pose, 5.0, dt)
        x, y, yaw = pose
        nxt = (x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt, wrap_angle(yaw + w * dt))
        if body_hits(occ, nxt):
            hits += 1                                   # contact: the body would be inside a wall
        else:
            pose = nxt
        clock.t += dt
        if not core.queue.active and not core.queue.queue:
            break
    return pose, hits


def make_core(tmp_path, places_xy, **kw):
    store = PlaceStore(str(tmp_path / 'places.json'))
    for name, x, y, yaw in places_xy:
        store.add(name, x, y, yaw)
    clock = Clock()
    core = NavigatorCore(store, PlannerParams(robot_radius=0.27), clock=clock,
                         tuning_path=str(tmp_path / 'tuning.json'), footprint=FP, dwell_s=0.0, **kw)
    grid, g = floor()
    core.set_map(grid)
    return core, occupied_points(g)


def test_robot_backed_against_a_wall_turns_around_without_touching_it(tmp_path):
    # robot starts facing east with its back 6 cm from the west wall's corner region and a wall
    # close on its right; the goal is behind it, so it has to turn round
    core, occ = make_core(tmp_path, [('Door', 3.5, 3.05, 0.0), ('Back', 2.0, 4.5, math.pi)])
    start = (0.48, 0.32, 0.0)
    assert not body_hits(occ, start)
    pose, hits = drive(core, occ, start, 'Back')
    assert hits == 0
    assert math.hypot(pose[0] - 2.0, pose[1] - 4.5) < 0.2


def test_through_the_door_without_contact(tmp_path):
    core, occ = make_core(tmp_path, [('Right room', 6.0, 3.0, 0.0)])
    pose, hits = drive(core, occ, (1.0, 4.5, -math.pi / 2), 'Right room')
    assert hits == 0
    assert math.hypot(pose[0] - 6.0, pose[1] - 3.0) < 0.2
    rec = core.mlog.recent(1)[0]
    assert rec['result'] == 'DONE' and rec['distance_m'] > 4.0 and rec['planned_m'] > 4.0
    assert rec['min_gap_m'] is not None and rec['min_gap_m'] > 0.0


# ------------------------------------------------------------------ battery + dock
def test_battery_percent_and_levels():
    assert percent(12.6) == 100 and percent(9.9) == 0 and 30 < percent(11.1) < 45
    clock = Clock()
    b = BatteryMonitor(clock=clock, confirm_s=10)
    b.update(11.7, 1.0)
    assert b.current_level() == 'ok'
    for _ in range(200):                   # sag to 10.4 V (~6 %) for a while
        clock.t += 1
        b.update(10.4, 1.0)
    assert b.current_level() == 'critical'
    b.update(12.0, -1.5)
    assert b.current_level() == 'charging' and b.info()['charging']
    clock.t += 30
    assert b.current_level() == 'unknown'           # no reading for 10 s


def test_low_battery_sends_robot_to_dock_and_it_reverses_on(tmp_path):
    core, occ = make_core(tmp_path, [('Lab', 6.5, 3.0, 0.0)])
    core.battery.confirm_s = 0.0
    core.places.set_dock(1.0, 1.0, math.pi / 2)      # dock against the south area, robot faces north on it
    pose = (2.0, 4.5, 0.0)
    dt = 0.05
    core.set_battery(10.8, 1.0)                       # ~20 % -> low
    docked = False
    for _ in range(int(120 / dt)):
        on = math.hypot(pose[0] - 1.0, pose[1] - 1.0) < 0.02
        core.set_battery(12.2 if on else 10.8, -1.2 if on else 1.0)   # contacts give current on the dock
        core.update_scan_base(scan_base(occ, pose))
        v, w = core.step(pose, 5.0, dt)
        x, y, yaw = pose
        pose = (x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt, wrap_angle(yaw + w * dt))
        core.clock.t += dt
        if core.docked:
            docked = True
            break
    assert docked and core.state == 'CHARGING'
    assert math.hypot(pose[0] - 1.0, pose[1] - 1.0) < 0.05
    # leaving the dock for a new task drives straight off first
    core.set_battery(12.2, -1.2)
    core.handle({'type': 'goto_place', 'name': 'Lab'}, pose)
    v, w = core.step(pose, 5.0, dt)
    assert v > 0 and w == 0.0 and core.state == 'UNDOCKING'


def test_critical_battery_refuses_new_missions(tmp_path):
    core, _ = make_core(tmp_path, [('Lab', 6.5, 3.0, 0.0)])
    core.battery.confirm_s = 0.0
    core.set_battery(10.5, 1.0)
    core.set_battery(10.5, 1.0)
    r = core.handle({'type': 'goto_place', 'name': 'Lab'}, (1, 4, 0))
    assert not r['ok'] and 'battery' in r['error']


# ------------------------------------------------------------------ wait for OK, schedules, explain
def test_wait_for_ok_stop(tmp_path):
    core, occ = make_core(tmp_path, [('Desk', 2.0, 4.5, 0.0), ('Door', 3.0, 3.05, 0.0)],
                          confirm_timeout_s=60.0)
    c = core.parser.parse('take this file to Desk and wait for a signature, then Door')
    assert [s['confirm'] for s in c['steps']] == [True, False]
    core.handle({'type': 'command', 'text': 'take this file to Desk and wait for a signature, then Door'}, (1, 4.5, 0))
    pose, dt = (1.0, 4.5, 0.0), 0.05
    for _ in range(int(40 / dt)):
        v, w = core.step(pose, 5.0, dt)
        x, y, yaw = pose
        pose = (x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt, wrap_angle(yaw + w * dt))
        core.clock.t += dt
        if core.state == 'CONFIRM':
            break
    assert core.state == 'CONFIRM' and core.status()['confirm']['label'] == 'Desk'
    assert 'press OK' in core.handle({'type': 'explain'})['answer']
    core.clock.t += 30
    core.step(pose, 5.0, dt)
    assert core.state == 'CONFIRM'                  # still waiting
    assert core.handle({'type': 'command', 'text': 'ok'})['ok']
    core.step(pose, 5.0, dt)
    assert core.queue.active.current.label == 'Door'


def test_schedule_fires_once_in_its_minute(tmp_path):
    now = [time.struct_time((2026, 10, 13, 9, 59, 0, 1, 286, 0))]     # Tuesday
    st = ScheduleStore(str(tmp_path / 's.json'), localtime=lambda: now[0])
    st.add('Patrol', '10:00', {'type': 'go_home'}, days=[0, 1, 2, 3, 4])
    assert st.due() == []
    now[0] = time.struct_time((2026, 10, 13, 10, 1, 0, 1, 286, 0))
    assert [s['name'] for s in st.due()] == ['Patrol']
    assert st.due() == []                                              # not twice
    now[0] = time.struct_time((2026, 10, 18, 10, 0, 0, 6, 291, 0))     # Sunday: not a weekday
    assert st.due() == []
    assert ScheduleStore(str(tmp_path / 's.json')).items[0]['name'] == 'Patrol'   # persisted


def test_core_runs_schedule_and_explains_failure(tmp_path):
    now = [time.struct_time((2026, 10, 13, 10, 0, 0, 1, 286, 0))]
    core, _ = make_core(tmp_path, [('Lab', 6.5, 3.0, 0.0), ('Inside table', 2.0, 1.5, 0.0)],
                        localtime=lambda: now[0])
    core.handle({'type': 'schedule_add', 'name': 'To lab', 'at': '10:00',
                 'action': {'type': 'goto_place', 'name': 'Lab'}})
    core.step((1.0, 4.5, 0.0), 5.0, 0.05)
    assert core.queue.active and core.queue.active.stops[0].label == 'Lab'
    core.handle({'type': 'cancel'})
    core.handle({'type': 'goto_place', 'name': 'Inside table'}, (1.0, 4.5, 0.0))
    for _ in range(5):
        core.step((1.0, 4.5, 0.0), 5.0, 0.05)
        core.clock.t += 3
    ans = core.handle({'type': 'explain'})['answer']
    assert 'Inside table' in ans and ('wall' in ans or 'route' in ans)
    stats = core.handle({'type': 'mission_log'})['stats']
    assert stats['failed'] == 1 and stats['canceled'] == 1


# ------------------------------------------------------------------ rooms
def test_rooms_find_and_go(tmp_path):
    core, _ = make_core(tmp_path, [('Lab 3 door', 3.5, 3.0, 0.0)])
    r = core.handle({'type': 'add_room', 'name': 'Workshop', 'polygon': [[4.2, 0.2], [7.8, 0.2], [7.8, 5.8], [4.2, 5.8]],
                     'contents': '3D printer, soldering station, oscilloscope'})
    assert r['ok'] and r['room']['contents'] == ['3D printer', 'soldering station', 'oscilloscope']
    assert 'Workshop' in core.handle({'type': 'find', 'q': 'where is the 3d printer'.split('the ')[-1]})['answer']
    p = CommandParser(core.places)
    c = p.parse('where is the oscilloscope?')
    assert c['intent'] == 'find' and c['query'].startswith('oscilloscope')
    c = p.parse('go to the room with the soldering station then Lab 3 door')
    assert c['intent'] == 'navigate' and c['targets'] == ['Workshop', 'Lab 3 door']
    c = p.parse('take me to the 3d printer')
    assert c['targets'] == ['Workshop']
    res = core.handle({'type': 'command', 'text': 'take me to the 3d printer'}, (1, 4, 0))
    assert res['ok'] and core.queue.queue[0].stops[0].label == 'Workshop'
    assert abs(core.queue.queue[0].stops[0].x - 6.0) < 0.01
    assert core.places.room_at(5, 3)['name'] == 'Workshop' and core.places.room_at(1, 1) is None
    assert p.parse('why did you stop?')['intent'] == 'explain'
    assert p.parse('go and charge')['intent'] == 'dock'


def test_bat_and_bump_lines():
    from lidar_robot.protocol import parse_line, BatPacket, BumpPacket
    p = parse_line('BAT,11850,-1520,1')
    assert isinstance(p, BatPacket) and p.volts == 11.85 and p.amps == -1.52 and p.src == 'ina226'
    p = parse_line('BAT,11200,0,2')
    assert p.amps is None and p.src == 'adc'
    assert parse_line('BAT,11200,0,7') is None and parse_line('BAT,x,0,1') is None
    assert parse_line('BMP,1') == BumpPacket(True) and parse_line('BMP,2') is None
    b = BatteryMonitor()
    b.update(5.4, None, 'adc')                     # floating ADC pin: ignored
    assert b.current_level() == 'unknown'
