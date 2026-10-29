"""scripts/mcp_server.py end to end against the simulated robot (tools/dashboard_sim.py) over HTTP,
with a staff login, as Claude Desktop / Claude Code would use it."""
import importlib.util
import json
import os
import sys

HERE = os.path.dirname(__file__)
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..'))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(ROOT, 'tools'))


def _load(path, name, env):
    old = {k: os.environ.get(k) for k in env}
    os.environ.update(env)
    try:
        spec = importlib.util.spec_from_file_location(name, path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    finally:
        for k, v in old.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v


def test_mcp_tools_against_the_simulator(tmp_path):
    import pytest
    if not (os.path.isfile(os.path.join(ROOT, 'scripts', 'mcp_server.py'))
            and os.path.isfile(os.path.join(ROOT, 'tools', 'dashboard_sim.py'))):
        pytest.skip('needs the whole repo (the Docker image only has the ROS package)')
    from lidar_robot.auth import UserStore
    from lidar_robot.dashboard_server import DashboardServer
    sim = _load(os.path.join(ROOT, 'tools', 'dashboard_sim.py'), 'dashboard_sim_t', {})
    users = UserStore(str(tmp_path / 'users.json'))
    users.signup('admin', 'admin123')
    users.signup('teacher', 'teach123')
    users.admin_update('admin', 'teacher', role='staff', status='active')
    srv = DashboardServer(sim.SimBackend(), host='127.0.0.1', port=0, users=users)
    srv.start()
    url = f'http://127.0.0.1:{srv.httpd.server_address[1]}'
    try:
        bad = _load(os.path.join(ROOT, 'scripts', 'mcp_server.py'), 'mcp_nologin',
                    {'ROBOT_URL': url, 'ROBOT_USER': '', 'ROBOT_PASSWORD': ''})
        r = bad.handle({'id': 1, 'method': 'tools/call', 'params': {'name': 'robot_status'}})
        assert r['isError'] and 'ROBOT_USER' in r['content'][0]['text']          # says what is missing

        m = _load(os.path.join(ROOT, 'scripts', 'mcp_server.py'), 'mcp_t',
                  {'ROBOT_URL': url, 'ROBOT_USER': 'teacher', 'ROBOT_PASSWORD': 'teach123'})
        assert m.handle({'id': 1, 'method': 'initialize', 'params': {}})['serverInfo']['name'] == 'pathik'
        names = {t['name'] for t in m.handle({'id': 2, 'method': 'tools/list'})['tools']}
        assert {'robot_status', 'go_to', 'command', 'explain', 'cancel'} <= names

        def call(tool, **args):
            r = m.handle({"id": 3, "method": "tools/call", "params": {"name": tool, "arguments": args}})
            return r['isError'], json.loads(r['content'][0]['text'])

        err, st = call('robot_status')
        assert not err and st['state'] in ('IDLE', 'CHARGING')
        err, pl = call('list_places')
        assert not err and pl['places']
        err, r = call('go_to', name=pl['places'][0])
        assert not err and r['ok'] and r['mission']['stops']
        err, r = call('command', text='go to ' + pl['places'][-1], dry_run=True)
        assert not err and r['plan']['intent'] == 'navigate'
        err, r = call('cancel')
        assert not err and r['ok']
        assert m.handle({'id': 4, 'method': 'tools/call', 'params': {'name': 'go_to'}})['isError']   # name missing
    finally:
        srv.stop()
