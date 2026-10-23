"""Login, roles and permissions: the store on its own, then over real HTTP."""
import json
import os
import sys
import time
import urllib.error
import urllib.request

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from lidar_robot import auth  # noqa: E402
from lidar_robot.auth import AuthError, UserStore, required_role  # noqa: E402


@pytest.fixture(autouse=True)
def fast_hash(monkeypatch):
    monkeypatch.setattr(auth, 'PBKDF2_ROUNDS', 1000)       # tests don't need slow hashing


def test_first_user_is_admin_then_signups_wait(tmp_path):
    s = UserStore(str(tmp_path / 'users.json'))
    assert s.setup_needed()
    r = s.signup('Arpit', 'secret1', 'Arpit A')
    assert r['token'] and r['user']['role'] == 'admin' and r['user']['username'] == 'arpit'
    assert not s.setup_needed()
    r2 = s.signup('teacher1', 'pass123')
    assert r2['token'] is None and r2['user']['status'] == 'pending'
    with pytest.raises(AuthError) as e:
        s.login('teacher1', 'pass123')
    assert e.value.code == 403 and 'approve' in str(e.value)
    s.admin_update('arpit', 'teacher1', role='staff', status='active')
    tok = s.login('teacher1', 'pass123')['token']
    assert s.verify(tok)['role'] == 'staff'
    # stored on disk without the password, survives a restart, session too
    raw = open(tmp_path / 'users.json').read()
    assert 'pass123' not in raw and tok not in raw
    s2 = UserStore(str(tmp_path / 'users.json'))
    assert s2.verify(tok)['username'] == 'teacher1'


def test_validation_and_duplicates(tmp_path):
    s = UserStore(str(tmp_path / 'u.json'))
    for bad in ('ab', 'has space', 'x' * 40, ''):
        with pytest.raises(AuthError):
            s.signup(bad, 'secret1')
    with pytest.raises(AuthError):
        s.signup('good', '123')                      # too short
    s.signup('good', 'secret1')
    with pytest.raises(AuthError) as e:
        s.signup('GOOD', 'secret1')                  # usernames are case-insensitive
    assert e.value.code == 409


def test_wrong_password_rate_limit(tmp_path):
    t = [1000.0]
    s = UserStore(str(tmp_path / 'u.json'), clock=lambda: t[0])
    s.signup('admin', 'secret1')
    for _ in range(auth.MAX_FAILS):
        with pytest.raises(AuthError) as e:
            s.login('admin', 'nope', '1.2.3.4')
        assert e.value.code == 401
    with pytest.raises(AuthError) as e:               # even the right password is refused now
        s.login('admin', 'secret1', '1.2.3.4')
    assert e.value.code == 429
    t[0] += auth.FAIL_WINDOW_S + 1
    assert s.login('admin', 'secret1', '1.2.3.4')['token']


def test_sessions_expire_and_disable_logs_out(tmp_path):
    t = [0.0]
    s = UserStore(str(tmp_path / 'u.json'), clock=lambda: t[0])
    s.signup('admin', 'secret1')
    s.signup('oper', 'secret1')
    s.admin_update('admin', 'oper', role='operator', status='active')
    tok = s.login('oper', 'secret1')['token']
    assert s.verify(tok)
    s.admin_update('admin', 'oper', status='disabled')
    assert s.verify(tok) is None
    s.admin_update('admin', 'oper', status='active')
    tok = s.login('oper', 'secret1')['token']
    t[0] += auth.SESSION_DAYS * 86400 + 1
    assert s.verify(tok) is None
    assert s.verify('made-up-token') is None and s.verify('') is None


def test_never_lose_the_last_admin(tmp_path):
    s = UserStore(str(tmp_path / 'u.json'))
    s.signup('boss', 'secret1')
    for kw in ({'role': 'staff'}, {'status': 'disabled'}):
        with pytest.raises(AuthError):
            s.admin_update('boss', 'boss', **kw)
    with pytest.raises(AuthError):
        s.admin_delete('boss', 'boss')
    s.signup('two', 'secret1')
    s.admin_update('boss', 'two', role='admin', status='active')
    s.admin_update('two', 'boss', role='operator')   # fine now: 'two' is still admin
    assert [u['role'] for u in s.list_users() if u['username'] == 'boss'] == ['operator']


def test_password_change_and_reset_end_sessions(tmp_path):
    s = UserStore(str(tmp_path / 'u.json'))
    tok = s.signup('admin', 'secret1')['token']
    with pytest.raises(AuthError):
        s.change_password('admin', 'wrong', 'newpass1')
    s.change_password('admin', 'secret1', 'newpass1')
    assert s.verify(tok) is None
    assert s.login('admin', 'newpass1')['token']
    s.admin_reset_password('admin', 'another1')
    assert s.login('admin', 'another1')['token']


def test_cli_reset_admin(tmp_path, capsys):
    p = str(tmp_path / 'u.json')
    assert auth._cli([p, 'reset-admin', 'rescue', 'rescue123']) == 0
    s = UserStore(p)
    assert s.login('rescue', 'rescue123')['user']['role'] == 'admin'


def test_permission_table():
    assert required_role('GET', '/api/ping') is None
    assert required_role('POST', '/api/auth/login') is None
    assert required_role('GET', '/api/state') == 'staff'
    assert required_role('POST', '/api/estop', {'on': True}) == 'staff'      # anyone can stop it
    assert required_role('POST', '/api/estop', {'on': False}) == 'operator'  # releasing: the team
    assert required_role('POST', '/api/nav', {'type': 'goto_place'}) == 'staff'
    assert required_role('POST', '/api/nav', {'type': 'command'}) == 'staff'
    assert required_role('POST', '/api/nav', {'type': 'add_place'}) == 'operator'
    assert required_role('POST', '/api/nav', {'type': 'explore_start'}) == 'operator'
    assert required_role('POST', '/api/cmd') == 'operator'                     # driving by hand
    assert required_role('POST', '/api/esp') == 'operator'
    assert required_role('POST', '/api/save_map') == 'operator'
    assert required_role('GET', '/api/users') == 'admin'
    assert required_role('POST', '/api/users/update') == 'admin'


# ---------------------------------------------------------------------- over HTTP
class Backend:
    def __init__(self):
        self.calls = []

    def get_state(self):
        return {'ok': 1}

    def get_map(self, since):
        return None

    def ping(self):
        return {'ok': True, 'api': 2}

    def command(self, v, w):
        self.calls.append(('cmd', v, w))

    def set_estop(self, on):
        self.calls.append(('estop', on))

    def nav_request(self, req):
        self.calls.append(('nav', req))
        return {'ok': True}


def call(base, path, body=None, token=None):
    h = {'Content-Type': 'text/plain'}
    if token:
        h['Authorization'] = 'Bearer ' + token
    req = urllib.request.Request(base + path, data=None if body is None else json.dumps(body).encode(),
                                 method='GET' if body is None else 'POST', headers=h)
    try:
        with urllib.request.urlopen(req) as r:
            return r.status, json.load(r)
    except urllib.error.HTTPError as e:
        return e.code, json.load(e)


def test_http_login_roles_and_admin_flow(tmp_path):
    from lidar_robot.dashboard_server import DashboardServer
    be = Backend()
    users = UserStore(str(tmp_path / 'users.json'))
    srv = DashboardServer(be, host='127.0.0.1', port=0, users=users)
    srv.start()
    base = f'http://127.0.0.1:{srv.httpd.server_address[1]}'
    try:
        code, p = call(base, '/api/ping')                           # public: discovery still works
        assert code == 200 and p['auth'] is True and p['setup_needed'] is True
        assert call(base, '/api/state')[0] == 401                   # everything else needs a login

        code, r = call(base, '/api/auth/signup', {'username': 'arpit', 'password': 'secret1', 'name': 'Arpit'})
        assert code == 200 and r['token'] and 'admin' in r['message']
        admin = r['token']
        assert call(base, '/api/ping')[1]['setup_needed'] is False

        code, r = call(base, '/api/auth/signup', {'username': 'mam', 'password': 'teach123'})
        assert code == 200 and r['token'] is None and 'approve' in r['message']
        assert call(base, '/api/auth/login', {'username': 'mam', 'password': 'teach123'})[0] == 403
        code, me = call(base, '/api/auth/me', token=admin)
        assert me['user']['role'] == 'admin' and me['pending'] == 1
        code, lst = call(base, '/api/users', token=admin)
        assert [u['status'] for u in lst['users']][0] == 'pending'  # waiting ones listed first
        assert call(base, '/api/users/update', {'username': 'mam', 'role': 'staff', 'status': 'active'},
                    token=admin)[0] == 200

        code, r = call(base, '/api/auth/login', {'username': 'mam', 'password': 'teach123'})
        staff = r['token']
        assert code == 200 and r['user']['role'] == 'staff'
        # staff: may send the robot and stop it, not drive by hand / edit / release the stop / manage users
        assert call(base, '/api/state', token=staff)[0] == 200
        assert call(base, '/api/nav', {'type': 'goto_place', 'name': 'Lab 3'}, token=staff)[0] == 200
        assert be.calls[-1] == ('nav', {'type': 'goto_place', 'name': 'Lab 3', 'by': 'mam'})   # who sent it
        assert call(base, '/api/estop', {'on': True}, token=staff)[0] == 200
        code, r = call(base, '/api/estop', {'on': False}, token=staff)
        assert code == 403 and r['need'] == 'operator'
        assert call(base, '/api/cmd', {'v': 0.1, 'w': 0}, token=staff)[0] == 403
        assert call(base, '/api/nav', {'type': 'add_place', 'name': 'x'}, token=staff)[0] == 403
        assert call(base, '/api/users', token=staff)[0] == 403
        # promote to operator: driving works
        call(base, '/api/users/update', {'username': 'mam', 'role': 'operator'}, token=admin)
        assert call(base, '/api/cmd', {'v': 0.1, 'w': 0}, token=staff)[0] == 200
        # bad token / logout
        assert call(base, '/api/state', token='forged')[0] == 401
        call(base, '/api/auth/logout', {}, token=staff)
        assert call(base, '/api/state', token=staff)[0] == 401
        # admin cannot remove themself; CORS preflight allows the Authorization header
        assert call(base, '/api/users/delete', {'username': 'arpit'}, token=admin)[0] == 400
        req = urllib.request.Request(base + '/api/state', method='OPTIONS')
        with urllib.request.urlopen(req) as r:
            assert 'Authorization' in r.headers['Access-Control-Allow-Headers']
    finally:
        srv.stop()
        time.sleep(0.05)
