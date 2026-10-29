"""The voice button on the robot (no login) and what the robot does when the network goes away."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))
from test_navigator_core import make_setup, run  # noqa: E402
from lidar_robot.navigator_core import NavigatorCore  # noqa: E402
from lidar_robot.netcheck import default_gateways, network_ok  # noqa: E402

HOME = (1.0, 1.0, 0.0)
AWAY = (6.0, 1.0, 0.0)          # Lab 3


def button(nav, text, pose):
    return nav.handle({'type': 'command', 'text': text, 'by': 'robot button', 'source': 'button'}, pose)


def reply(r):
    return r['interpretation']['reply']


# ------------------------------------------------------------------ the button
def test_button_trip_from_home_is_accepted_at_normal_priority(tmp_path):
    nav, clk = make_setup(tmp_path)
    nav.step(HOME, float('inf'), 0.05)
    r = button(nav, 'urgent go to hod', HOME)
    assert r['ok'] and not r.get('refused') and r.get('mission')
    m = nav.queue.active or nav.queue.queue[0]
    assert m.priority <= 1                      # "urgent" from the button never pre-empts anyone
    assert m.by == 'robot button' and m.source == 'button'
    x, y, _ = run(nav, clk, HOME, 200)
    assert math.hypot(x - 6.5, y - 4.5) < 0.25


def test_button_trip_away_from_home_is_refused(tmp_path):
    nav, clk = make_setup(tmp_path)
    nav.step(AWAY, float('inf'), 0.05)
    r = button(nav, 'go to hod', AWAY)
    assert r.get('refused') and 'home spot' in reply(r)
    assert nav.queue.active is None and not nav.queue.queue
    assert nav.status()['button']['at_home'] is False


def test_button_stop_works_anywhere_and_questions_are_answered(tmp_path):
    nav, clk = make_setup(tmp_path)
    nav.handle({'type': 'command', 'text': 'go to lab 3', 'by': 'arpit'}, HOME)
    run(nav, clk, HOME, 3)                      # moving, away from home
    assert nav.queue.active is not None
    r = button(nav, 'stop', nav._pose)
    assert r['ok'] and not r.get('refused')
    assert nav.queue.active is None and nav.state == 'IDLE'
    r = button(nav, 'where are you', nav._pose)   # status: answered, nothing moves
    assert not r.get('refused')


def test_button_cannot_queue_behind_a_running_app_mission(tmp_path):
    nav, clk = make_setup(tmp_path)
    nav.handle({'type': 'command', 'text': 'go to store', 'by': 'arpit'}, HOME)
    nav.step(HOME, float('inf'), 0.05)          # mission starts, robot still at home
    r = button(nav, 'go to hod', HOME)
    assert r.get('refused') and 'busy' in reply(r)
    assert [m.by for m in ([nav.queue.active] + nav.queue.queue) if m] == ['arpit']


def test_button_ok_only_when_something_waits(tmp_path):
    nav, clk = make_setup(tmp_path)
    nav.step(AWAY, float('inf'), 0.05)
    r = button(nav, 'ok done', AWAY)
    assert reply(r) in ('Nothing is waiting for an OK.',) or r['interpretation']['intent'] != 'confirm'


def test_button_one_trip_per_gap(tmp_path):
    nav, clk = make_setup(tmp_path, button_gap_s=30.0)
    nav.step(HOME, float('inf'), 0.05)
    assert button(nav, 'go to lab 3', HOME).get('mission')
    nav.handle({'type': 'cancel'})              # someone cancels it in the app, robot still at home
    nav.step(HOME, float('inf'), 0.05)
    clk.t += 5
    r = button(nav, 'go to store', HOME)
    assert r.get('refused') and 'wait' in reply(r).lower()
    clk.t += 30
    assert button(nav, 'go to store', HOME).get('mission')


def test_button_switch_is_remembered(tmp_path):
    nav, clk = make_setup(tmp_path, data_dir=str(tmp_path))
    nav.step(HOME, float('inf'), 0.05)
    r = nav.handle({'type': 'button_settings', 'enabled': False, 'by': 'arpit'}, HOME)
    assert r['ok'] and r['button']['enabled'] is False
    r = button(nav, 'go to hod', HOME)
    assert r.get('refused') and 'turned off' in reply(r)
    nav2, _ = make_setup(tmp_path, data_dir=str(tmp_path))
    assert nav2.button_enabled is False         # survives a restart


def test_on_the_dock_counts_as_home(tmp_path):
    nav, clk = make_setup(tmp_path)
    nav.docked = True
    nav.step(AWAY, float('inf'), 0.05)
    assert nav._at_home(AWAY)


# ------------------------------------------------------------------ network loss
def idle_steps(nav, clk, pose, seconds, dt=0.5):
    for _ in range(int(seconds / dt)):
        nav.step(pose, float('inf'), dt)
        clk.t += dt


def test_no_network_and_idle_away_from_home_goes_home_once(tmp_path):
    nav, clk = make_setup(tmp_path, offline_home_s=120.0)
    nav.set_network(True)
    nav.set_network(False)
    idle_steps(nav, clk, AWAY, 100)
    assert nav.queue.active is None             # not yet: 2 minutes
    idle_steps(nav, clk, AWAY, 30)
    m = nav.queue.active or nav.queue.queue[0]
    assert m.source == 'offline-home' and m.by == ''
    assert any(k == 'offline_home' for k, _ in nav.say_queue)
    x, y, yaw = run(nav, clk, AWAY, 200)
    assert math.hypot(x - 1.0, y - 1.0) < 0.25
    idle_steps(nav, clk, (x, y, yaw), 200)      # at home: stays, no second trip
    assert nav.queue.active is None and not nav.queue.queue
    assert nav.status()['button']['at_home']


def test_network_loss_never_cancels_accepted_trips(tmp_path):
    nav, clk = make_setup(tmp_path, offline_home_s=60.0)
    nav.set_network(True)
    nav.handle({'type': 'command', 'text': 'go to lab 3 then store', 'by': 'arpit'}, HOME)
    nav.set_network(False)                      # the hotspot dies right after the order
    x, y, yaw = run(nav, clk, HOME, 300)
    assert [m.state for m in nav.queue.history][:1] == ['DONE']
    assert math.hypot(x - 1.0, y - 4.5) < 0.25  # it finished the whole trip (Store)
    idle_steps(nav, clk, (x, y, yaw), 70)       # then nothing to do and still offline: home
    m = nav.queue.active or nav.queue.queue[0]
    assert m.source == 'offline-home'


def test_network_back_logs_what_happened_offline(tmp_path):
    nav, clk = make_setup(tmp_path, offline_home_s=10.0)
    nav.set_network(True)
    nav.set_network(False)
    idle_steps(nav, clk, AWAY, 12)
    nav.set_network(True)
    assert nav.status()['network']['ok'] is True
    assert any('connection back' in e['text'] and 'drove home' in e['text'] for e in nav.events)


def test_lab_cable_without_network_never_triggers(tmp_path):
    nav, clk = make_setup(tmp_path, offline_home_s=10.0)
    nav.set_network(False)                      # never been up since start (laptop cable only)
    idle_steps(nav, clk, AWAY, 60)
    assert nav.queue.active is None and not nav.queue.queue


def test_offline_home_can_be_switched_off(tmp_path):
    nav, clk = make_setup(tmp_path, offline_home_s=0.0)
    nav.set_network(True)
    nav.set_network(False)
    idle_steps(nav, clk, AWAY, 300)
    assert nav.queue.active is None and not nav.queue.queue


# ------------------------------------------------------------------ netcheck
ROUTES = """Iface\tDestination\tGateway \tFlags\tRefCnt\tUse\tMetric\tMask\t\tMTU\tWindow\tIRTT
wlan0\t00000000\t012BA8C0\t0003\t0\t0\t600\t00000000\t0\t0\t0
wlan0\t002BA8C0\t00000000\t0001\t0\t0\t600\t00FFFFFF\t0\t0\t0
eth0\t0000FEA9\t00000000\t0001\t0\t0\t1000\t0000FFFF\t0\t0\t0
"""


def test_default_gateway_parsing():
    assert default_gateways(ROUTES) == [('wlan0', '192.168.43.1')]
    only_cable = ROUTES.splitlines()[0] + '\n' + ROUTES.splitlines()[3] + '\n'
    assert default_gateways(only_cable) == []


def test_network_ok_pings_the_gateway():
    seen = []
    assert network_ok(ROUTES, ping_fn=lambda h: seen.append(h) or True) and seen == ['192.168.43.1']
    assert not network_ok(ROUTES, ping_fn=lambda h: False)
    assert not network_ok(ROUTES.splitlines()[0] + '\n', ping_fn=lambda h: True)


def test_core_without_network_reports_none():
    from lidar_robot.places import PlaceStore
    nav = NavigatorCore(PlaceStore(None))
    st = nav.status()
    assert st['network']['ok'] is None and st['button']['enabled'] is True
