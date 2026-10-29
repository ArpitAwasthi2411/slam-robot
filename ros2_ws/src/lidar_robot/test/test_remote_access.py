"""Phase 1: robot reachable over Tailscale and (optionally) Tailscale Funnel = public internet."""
import json
import os
import sys
import urllib.error
import urllib.request

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from lidar_robot import auth  # noqa: E402
from lidar_robot.auth import AuthError, UserStore  # noqa: E402
from lidar_robot.dashboard_server import DashboardServer, link_kind  # noqa: E402


class Backend:
    def get_state(self):
        return {'pose': None}

    def get_map(self, since):
        return None

    def ping(self):
        return {'ok': True, 'api': 2, 'robot': 'slam-robot'}

    def command(self, v, w):
        pass


def req(base, path, body=None, headers=None, token=None):
    h = dict(headers or {})
    if token:
        h['Authorization'] = 'Bearer ' + token
    r = urllib.request.Request(base + path, data=None if body is None else json.dumps(body).encode(),
                               method='GET' if body is None else 'POST', headers=h)
    try:
        with urllib.request.urlopen(r) as resp:
            return resp.status, json.load(resp), dict(resp.headers)
    except urllib.error.HTTPError as e:
        return e.code, json.load(e), dict(e.headers)


FUNNEL = {'X-Forwarded-For': '203.0.113.7', 'X-Forwarded-Proto': 'https'}       # as tailscaled sends it


@pytest.fixture
def server(tmp_path):
    flag = tmp_path / '.public_url'
    users = UserStore(str(tmp_path / 'users.json'))
    srv = DashboardServer(Backend(), host='127.0.0.1', port=0, users=users, public_flag=str(flag))
    srv.start()
    yield f'http://127.0.0.1:{srv.httpd.server_address[1]}', users, flag
    srv.stop()


def test_link_kind():
    assert link_kind('100.101.2.3') == 'tailscale' and link_kind('fd7a:115c:a1e0::1') == 'tailscale'
    assert link_kind('192.168.43.20') == 'lan' and link_kind('127.0.0.1') == 'local'
    assert link_kind('garbage') == 'lan'


def test_net_info_and_public_flag(server):
    base, users, flag = server
    code, p, h = req(base, '/api/ping')
    assert code == 200 and p['net'] == {'via': 'local', 'public': False, 'public_url': None, 'https': False}
    assert h['X-Content-Type-Options'] == 'nosniff' and h['X-Frame-Options'] == 'DENY'
    assert 'Strict-Transport-Security' not in h
    flag.write_text('https://slam-robot.tail1234.ts.net\n')
    import time
    time.sleep(3.1)                                       # flag file is re-read every 3 s
    code, p, h = req(base, '/api/ping', headers=FUNNEL)
    assert p['net']['via'] == 'public' and p['net']['public'] and p['net']['https']
    assert p['net']['public_url'] == 'https://slam-robot.tail1234.ts.net'
    assert h['Strict-Transport-Security'].startswith('max-age=')
    # a phone on the tailnet going through `tailscale serve`: proxied too, but not public
    code, p, _ = req(base, '/api/ping', headers={'X-Forwarded-For': '100.90.1.2'})
    assert p['net']['via'] == 'tailscale'
    # the Funnel header alone marks a request public even before the flag file is read
    code, p, _ = req(base, '/api/ping', headers={'X-Forwarded-For': '198.51.100.1', 'Tailscale-Funnel-Request': '?1'})
    assert p['net']['via'] == 'public'


def test_first_admin_never_over_the_public_link(server):
    base, users, flag = server
    code, r, _ = req(base, '/api/auth/signup', {'username': 'eve', 'password': 'secret1'},
                     headers=dict(FUNNEL, **{'Tailscale-Funnel-Request': '?1'}))
    assert code == 403 and 'admin' in r['error'] and users.setup_needed()
    code, r, _ = req(base, '/api/auth/signup', {'username': 'arpit', 'password': 'secret1'})   # on the robot's network
    assert code == 200 and r['token']


def test_public_sessions_are_shorter(tmp_path):
    t = [1000.0]
    u = UserStore(str(tmp_path / 'u.json'), clock=lambda: t[0])
    u.signup('arpit', 'secret1')
    lan = u.login('arpit', 'secret1', '192.168.1.5')['token']
    pub = u.login('arpit', 'secret1', '203.0.113.7', public=True)['token']
    t[0] += (auth.SESSION_DAYS_PUBLIC + 1) * 86400
    assert u.verify(lan) and u.verify(pub) is None


def test_rate_limits(tmp_path):
    u = UserStore(str(tmp_path / 'u.json'))
    u.signup('arpit', 'secret1')
    # 1) one address guessing many usernames
    for i in range(auth.ADDR_MAX_FAILS):
        with pytest.raises(AuthError):
            u.login(f'user{i}', 'x', '203.0.113.9', public=True)
    with pytest.raises(AuthError) as e:
        u.login('arpit', 'secret1', '203.0.113.9', public=True)
    assert e.value.code == 429
    # 2) many addresses guessing one username over the public link -> locked for the public link only
    for i in range(auth.PUBLIC_USER_MAX_FAILS):
        with pytest.raises(AuthError):
            u.login('arpit', 'wrong', f'198.51.100.{i}', public=True)
    with pytest.raises(AuthError) as e:
        u.login('arpit', 'secret1', '198.51.100.250', public=True)
    assert e.value.code == 429 and "own network" in str(e.value)
    assert u.login('arpit', 'secret1', '192.168.43.5')['token']            # the owner on the robot's Wi-Fi
    # 3) sign-up spam
    for i in range(auth.SIGNUPS_PER_ADDR):
        u.signup(f'spam{i}', 'secret1', addr='203.0.113.50', public=True)
    with pytest.raises(AuthError) as e:
        u.signup('spam99', 'secret1', addr='203.0.113.50', public=True)
    assert e.value.code == 429
    # 4) tables don't grow forever
    u._prune(u.clock() + 10 * 3600)
    assert not u.fails and not u.addr_fails and not u.public_fails and not u.signups


def test_no_login_mode_refuses_public_requests(tmp_path):
    flag = tmp_path / '.public_url'
    flag.write_text('https://x.ts.net')
    srv = DashboardServer(Backend(), host='127.0.0.1', port=0, users=None, public_flag=str(flag))
    srv.start()
    base = f'http://127.0.0.1:{srv.httpd.server_address[1]}'
    try:
        assert req(base, '/api/state')[0] == 200                          # robot's own network: old behaviour
        code, r, _ = req(base, '/api/state', headers=FUNNEL)
        assert code == 403 and 'logins are required' in r['error']
        assert req(base, '/api/cmd', {'v': 0.2, 'w': 0}, headers=FUNNEL)[0] == 403
        assert req(base, '/api/ping', headers=FUNNEL)[0] == 200            # discovery still answers
    finally:
        srv.stop()
