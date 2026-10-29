"""scripts/setup_tailscale.sh with fake tailscale/systemctl/curl commands (no Tailscale needed)."""
import json
import os
import shutil
import stat
import subprocess

import pytest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
SCRIPT = os.path.join(ROOT, 'scripts', 'setup_tailscale.sh')

FAKE_TS = """#!/bin/sh
echo "tailscale $*" >> "$FAKE_LOG"
case "$1" in
  status) [ "$2" = "--json" ] && echo '{"BackendState": "Running", "Self": {"DNSName": "slam-robot.tail1234.ts.net."}}' || echo "100.101.1.2 slam-robot";;
  ip) echo 100.101.1.2;;
esac
exit 0
"""


def _bin(d, name, body):
    p = d / name
    p.write_text(body)
    p.chmod(p.stat().st_mode | stat.S_IEXEC)


@pytest.fixture
def env(tmp_path):
    if not os.path.isfile(SCRIPT) or os.geteuid() != 0 or not shutil.which('python3'):
        pytest.skip('needs the repo scripts/ and root (the script checks for sudo)')
    b = tmp_path / 'bin'
    b.mkdir()
    _bin(b, 'tailscale', FAKE_TS)
    _bin(b, 'systemctl', '#!/bin/sh\necho "systemctl $*" >> "$FAKE_LOG"\necho active\n')
    _bin(b, 'curl', '#!/bin/sh\necho \'{"api": 2}\'\n')
    home = tmp_path / 'home'
    (home / 'maps').mkdir(parents=True)
    log = tmp_path / 'log'
    e = dict(os.environ, PATH=f'{b}:{os.environ["PATH"]}', PATHIK_HOME=str(home), FAKE_LOG=str(log),
             SUDO_USER='root')
    e.pop('TS_AUTHKEY', None)
    return e, home, log


def run(e, *args):
    return subprocess.run(['bash', SCRIPT, *args], env=e, capture_output=True, text=True, timeout=60)


def test_up_with_auth_key_from_robot_env(env):
    e, home, log = env
    (home / 'robot.env').write_text('GROQ_API_KEY=x\nTS_AUTHKEY=tskey-auth-secret123\n')
    r = run(e, 'up')
    assert r.returncode == 0, r.stderr
    calls = log.read_text()
    assert 'systemctl enable --now tailscaled' in calls
    assert '--auth-key=tskey-auth-secret123' in calls and '--hostname=slam-robot' in calls
    assert 'tskey' not in r.stdout + r.stderr                            # the key is never printed
    assert 'slam-robot.tail1234.ts.net' in r.stdout and '100.101.1.2' in r.stdout


def test_funnel_needs_an_admin_first_then_writes_the_flag(env):
    e, home, log = env
    r = run(e, 'funnel', 'on')
    assert r.returncode == 1 and 'No admin account' in r.stdout
    assert not (home / 'maps' / '.public_url').exists() and not log.exists()     # refused before calling tailscale
    (home / 'maps' / 'users.json').write_text(json.dumps({'users': {'arpit': {'role': 'admin', 'status': 'active'}}}))
    r = run(e, 'funnel', 'on')
    assert r.returncode == 0, r.stdout + r.stderr
    assert 'tailscale funnel --bg 8080' in log.read_text()
    assert (home / 'maps' / '.public_url').read_text().strip() == 'https://slam-robot.tail1234.ts.net'
    assert 'PUBLIC (Funnel)    : ON' in run(e, 'status').stdout
    r = run(e, 'funnel', 'off')
    assert r.returncode == 0 and not (home / 'maps' / '.public_url').exists()
