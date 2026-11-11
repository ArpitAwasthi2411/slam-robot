"""Nav2 behind the follower interface: goals, results, and Pathik's safety checks on Nav2's commands."""
import math

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))
from lidar_robot.follower import FollowerParams  # noqa: E402
from lidar_robot.nav2_follower import Nav2Follower  # noqa: E402
from test_navigator_core import make_setup  # noqa: E402


class Rig:
    def __init__(self, mode='navigate'):
        self.t, self.sent, self.cancelled = 0.0, [], []
        self.f = Nav2Follower(FollowerParams(max_linear=0.2, max_angular=0.5, stop_distance=0.4, slow_distance=0.8,
                                             blocked_after=2.0),
                              lambda *a: self.sent.append(a), self.cancelled.append, lambda: self.t, mode=mode)

    def step(self, front=5.0, guard=None, dt=0.05, pose=(0.0, 0.0, 0.0)):
        self.t += dt
        return self.f.step(pose, front, dt, guard)


def test_goal_is_sent_with_the_route_end_and_heading():
    r = Rig()
    r.f.set_path([(0, 0), (1, 0), (1, 2)], None)
    seq, mode, pts, goal = r.sent[0]
    assert (seq, mode) == (1, 'navigate') and goal[:2] == (1, 2) and abs(goal[2] - math.pi / 2) < 1e-9
    r.f.set_path([(0, 0), (3, 0)], 1.0)                      # a new goal cancels the old one
    assert r.cancelled == [1] and r.sent[1][3] == (3, 0, 1.0)


def test_nav2_command_is_driven_within_pathik_limits():
    r = Rig()
    r.f.set_path([(0, 0), (3, 0)])
    assert r.step() == (0.0, 0.0, None)                      # Nav2 has said nothing yet
    r.f.on_cmd(0.15, 0.2)
    assert r.step() == (0.15, 0.2, None)
    r.f.on_cmd(0.9, -3.0)                                    # more than the app allows
    assert r.step() == (0.2, -0.5, None)
    r.t += 1.0                                               # Nav2 went quiet (planning): stand still
    assert r.step() == (0.0, 0.0, None)
    r.f.on_cmd(float('nan'), 0.0)
    assert r.step()[:2] == (0.0, 0.0)


def test_front_obstacle_stops_the_robot_whatever_nav2_says():
    r = Rig()
    r.f.set_path([(0, 0), (3, 0)])
    r.f.on_cmd(0.2, 0.0)
    v, _, _ = r.step(front=0.6)
    assert 0.05 <= v < 0.2                                   # slowing down
    out = None
    for _ in range(60):
        r.f.on_cmd(0.2, 0.0)
        out = r.step(front=0.3)
        if out[2]:
            break
        assert out[:2] == (0.0, 0.0) and r.f.blocked and r.f.why_blocked == 'front'
    assert out[2] == 'BLOCKED' and r.cancelled == [1]        # told the core, cancelled Nav2's goal
    r.f.on_cmd(-0.1, 0.0)                                    # backing away is never blocked by the front check
    r.f.set_path([(0, 0), (3, 0)])
    r.f.on_cmd(-0.1, 0.0)
    assert r.step(front=0.3)[0] == -0.1


def test_body_check_gates_a_turn():
    class Guard:
        min_gap = 0.05

        def motion_clear(self, v, w):
            return abs(w) < 0.1 and v <= 0.05
    r = Rig()
    r.f.set_path([(0, 0), (3, 0)])
    r.f.on_cmd(0.2, 0.5)
    assert r.step(guard=Guard()) == (0.05, 0.0, None)        # creep straight instead of swinging the back out


def test_results_and_stale_answers():
    r = Rig()
    r.f.set_path([(0, 0), (3, 0)])
    r.f.set_path([(0, 0), (5, 0)])
    r.f.on_result(1, True)                                   # answer to the goal we replaced: ignored
    assert r.step()[2] is None
    r.f.on_result(2, True)
    assert r.step()[2] == 'ARRIVED' and r.step() == (0.0, 0.0, None)
    r.f.set_path([(0, 0), (5, 0)])
    r.f.on_result(3, False, 'Nav2 gave up: no route or stuck')
    assert r.step()[2] == 'BLOCKED' and r.f.why_blocked == 'nav2' and 'gave up' in r.f.nav2_note
    assert r.f.stats == {'goals': 3, 'arrived': 1, 'failed': 1, 'gated': 0}


def test_nav2_not_running_is_reported_not_waited_for_forever():
    r = Rig()
    r.f.set_path([(0, 0), (3, 0)])
    ev = None
    for _ in range(300):
        ev = r.step()[2]
        if ev:
            break
    assert ev == 'BLOCKED' and 'did not answer' in r.f.nav2_note and 9.5 < r.t < 11


def test_follow_mode_hands_over_pathiks_route_and_keeps_it():
    r = Rig('follow')
    r.f.set_path([(0, 0), (1, 0), (1, 2)])
    assert r.sent[0][1] == 'follow' and r.sent[0][2] == [(0, 0), (1, 0), (1, 2)]
    r.f.on_plan([(0, 0), (9, 9)])
    assert r.f.path[-1] == (1, 2)
    n = Rig()
    n.f.set_path([(0, 0), (1, 2)])
    n.f.on_plan([(0, 0), (0.5, 1.0), (1, 2)])                # navigate mode: the app draws Nav2's route
    assert len(n.f.path) == 3


def test_core_runs_a_mission_through_nav2(tmp_path):
    core, clk = make_setup(tmp_path)
    sent = []
    fol = Nav2Follower(core.follower.p, lambda *a: sent.append(a), lambda s: None, clk)
    core.follower = fol
    pose = (1.0, 1.0, 0.0)
    assert core.handle({'type': 'goto_place', 'name': 'Lab 3'}, pose)['ok']
    for _ in range(40):
        clk.t += 0.05
        core.step(pose, 5.0, 0.05)
        if sent:
            break
    assert sent and sent[0][3][:2] == (6.0, 1.0)             # the core planned, then handed the goal to Nav2
    fol.on_cmd(0.15, 0.0)
    clk.t += 0.05
    assert core.step(pose, 5.0, 0.05) == (0.15, 0.0)
    fol.on_result(fol.seq, True)
    clk.t += 0.05
    core.step((6.0, 1.0, 0.0), 5.0, 0.05)
    assert any('arrived at Lab 3' in str(e) for e in core.events)
    # Nav2 gives up -> the core replans and sends a new goal
    assert core.handle({'type': 'goto_place', 'name': 'Store'}, (6.0, 1.0, 0.0))['ok']
    for _ in range(200):
        clk.t += 0.05
        core.step((6.0, 1.0, 0.0), 5.0, 0.05)
        if len(sent) >= 2:
            break
    n = len(sent)
    fol.on_result(fol.seq, False, 'Nav2 gave up: no route or stuck')
    for _ in range(100):
        clk.t += 0.05
        core.step((6.0, 1.0, 0.0), 5.0, 0.05)
    assert len(sent) > n and any('Nav2 gave up' in str(e) for e in core.events)


def test_nav2_params_match_the_robot():
    import ast
    import yaml
    cfg = os.path.join(os.path.dirname(__file__), '..', 'config')
    n2 = yaml.safe_load(open(os.path.join(cfg, 'nav2_params.yaml')))
    rp = yaml.safe_load(open(os.path.join(cfg, 'robot_params.yaml')))['navigator']['ros__parameters']
    front, rear = rp['footprint_front'] + rp['footprint_margin'], rp['footprint_rear'] + rp['footprint_margin']
    half = rp['footprint_half_width'] + rp['footprint_margin']
    want = [[front, half], [front, -half], [-rear, -half], [-rear, half]]
    for cm in ('local_costmap', 'global_costmap'):
        got = ast.literal_eval(n2[cm][cm]['ros__parameters']['footprint'])
        assert all(abs(a - b) < 1e-6 for p, q in zip(got, want) for a, b in zip(p, q)), cm
    rpp = n2['controller_server']['ros__parameters']['FollowPath']
    assert rpp['desired_linear_vel'] <= rp['max_linear'] and rpp['rotate_to_heading_angular_vel'] <= rp['max_angular']
    assert n2['behavior_server']['ros__parameters']['max_rotational_vel'] <= rp['max_angular']
    launch = open(os.path.join(cfg, '..', 'launch', 'bringup.launch.py')).read()
    assert launch.count("('cmd_vel', 'cmd_vel_nav2')") == 1 and launch.count('remappings=to_pathik') == 2
