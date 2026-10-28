"""Users, sessions and permissions for the robot's HTTP API (stdlib only, unit-tested).

Roles (each includes everything of the ones before it):
    staff     teachers/staff: send or call the robot, stop it, see status and map
    operator  the robot team: drive by hand, edit places/rooms, mapping, tuning, schedules
    admin     everything + approve sign-ups, change roles, remove users

Accounts:
    * The FIRST account created on a robot becomes an active admin (first-run setup).
    * Every later sign-up is 'pending' until an admin approves it (and picks its role).
    * Passwords are stored as PBKDF2-SHA256 hashes with a per-user salt, never in plain text.
    * Sessions are random tokens (only their SHA-256 is stored), valid for SESSION_DAYS and
      kept on disk so a robot restart doesn't log everybody out.

Recovery from the Pi's terminal (forgotten admin password):
    python3 -m lidar_robot.auth ~/maps/users.json reset-admin <username> <new password>
"""
import hashlib
import hmac
import json
import os
import re
import secrets
import tempfile
import threading
import time

ROLES = ('staff', 'operator', 'admin')
RANK = {r: i for i, r in enumerate(ROLES)}
SESSION_DAYS = 30
PBKDF2_ROUNDS = 120_000          # ~0.15 s on a Pi 4: slow for attackers, fine for a login
USERNAME_RE = re.compile(r'^[a-z0-9._-]{3,32}$')
MIN_PASSWORD = 6
MAX_FAILS = 5                    # per username+address, then wait FAIL_WINDOW_S
FAIL_WINDOW_S = 60.0


class AuthError(Exception):
    """Message is safe to show to the user. `code` is the HTTP status to answer with."""

    def __init__(self, msg, code=400):
        super().__init__(msg)
        self.code = code


def _hash_password(password, salt=None, rounds=None):
    salt = salt or secrets.token_hex(16)
    rounds = rounds or PBKDF2_ROUNDS
    h = hashlib.pbkdf2_hmac('sha256', password.encode(), bytes.fromhex(salt), rounds).hex()
    return salt, h, rounds


def _token_id(token):
    return hashlib.sha256(token.encode()).hexdigest()


def at_least(role, needed):
    return RANK.get(role, -1) >= RANK[needed]


class UserStore:
    def __init__(self, path, clock=time.time):
        self.path = path
        self.clock = clock
        self.lock = threading.RLock()
        self.users = {}
        self.sessions = {}            # token sha256 -> {'user', 'exp'}
        self.fails = {}               # (username, addr) -> [t, ...]
        self._load()

    # ------------------------------------------------------------------ storage
    def _load(self):
        if not self.path or not os.path.isfile(self.path):
            return
        with open(self.path) as f:
            d = json.load(f)
        self.users = d.get('users', {})
        now = self.clock()
        self.sessions = {k: v for k, v in d.get('sessions', {}).items() if v.get('exp', 0) > now}

    def _save(self):
        if not self.path:
            return
        d = os.path.dirname(os.path.abspath(self.path))
        os.makedirs(d, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=d, prefix='.users-')
        with os.fdopen(fd, 'w') as f:
            json.dump({'users': self.users, 'sessions': self.sessions}, f, indent=1)
        os.chmod(tmp, 0o600)              # password hashes: owner only
        os.replace(tmp, self.path)

    # ------------------------------------------------------------------ helpers
    @staticmethod
    def public(username, u):
        return {'username': username, 'name': u.get('name') or username, 'role': u['role'],
                'status': u['status'], 'created': u.get('created'), 'last_login': u.get('last_login')}

    def setup_needed(self):
        with self.lock:
            return not any(u['role'] == 'admin' and u['status'] == 'active' for u in self.users.values())

    def _active_admins(self):
        return [n for n, u in self.users.items() if u['role'] == 'admin' and u['status'] == 'active']

    def _check_new(self, username, password):
        username = (username or '').strip().lower()
        if not USERNAME_RE.match(username):
            raise AuthError('Username: 3-32 characters, letters, digits, dot, dash or underscore')
        if len(password or '') < MIN_PASSWORD:
            raise AuthError(f'Password must be at least {MIN_PASSWORD} characters')
        return username

    def _new_session(self, username):
        token = secrets.token_urlsafe(32)
        self.sessions[_token_id(token)] = {'user': username, 'exp': self.clock() + SESSION_DAYS * 86400}
        return token

    # ------------------------------------------------------------------ public API
    def signup(self, username, password, name=''):
        """First account -> active admin + logged in. Later -> pending (no token)."""
        with self.lock:
            username = self._check_new(username, password)
            if username in self.users:
                raise AuthError('That username is taken', 409)
            first = self.setup_needed()
            salt, h, rounds = _hash_password(password)
            self.users[username] = {'name': (name or '').strip()[:60] or username,
                                    'role': 'admin' if first else 'staff',
                                    'status': 'active' if first else 'pending',
                                    'salt': salt, 'hash': h, 'rounds': rounds,
                                    'created': int(self.clock()), 'last_login': None}
            token = None
            if first:
                token = self._new_session(username)
                self.users[username]['last_login'] = int(self.clock())
            self._save()
            return {'user': self.public(username, self.users[username]), 'token': token}

    def login(self, username, password, addr=''):
        with self.lock:
            username = (username or '').strip().lower()
            key = (username, addr)
            now = self.clock()
            recent = [t for t in self.fails.get(key, []) if now - t < FAIL_WINDOW_S]
            if len(recent) >= MAX_FAILS:
                raise AuthError('Too many wrong passwords. Wait a minute and try again.', 429)
            u = self.users.get(username)
            ok = False
            if u:
                _, h, _ = _hash_password(password or '', u['salt'], u.get('rounds'))
                ok = hmac.compare_digest(h, u['hash'])
            if not ok:
                self.fails[key] = recent + [now]
                raise AuthError('Wrong username or password', 401)
            self.fails.pop(key, None)
            if u['status'] == 'pending':
                raise AuthError('Your account is waiting for an admin to approve it', 403)
            if u['status'] == 'disabled':
                raise AuthError('This account has been disabled. Ask an admin.', 403)
            token = self._new_session(username)
            u['last_login'] = int(now)
            self._save()
            return {'user': self.public(username, u), 'token': token}

    def verify(self, token):
        """-> public user dict, or None (unknown/expired token, user removed/disabled)."""
        if not token:
            return None
        with self.lock:
            s = self.sessions.get(_token_id(token))
            if not s or s['exp'] < self.clock():
                return None
            u = self.users.get(s['user'])
            if not u or u['status'] != 'active':
                return None
            return self.public(s['user'], u)

    def logout(self, token):
        with self.lock:
            if self.sessions.pop(_token_id(token or ''), None):
                self._save()

    def change_password(self, username, old, new):
        with self.lock:
            u = self.users.get(username)
            _, h, _ = _hash_password(old or '', u['salt'], u.get('rounds'))
            if not hmac.compare_digest(h, u['hash']):
                raise AuthError('Current password is wrong', 401)
            self._set_password(username, new)

    def _set_password(self, username, new):
        if len(new or '') < MIN_PASSWORD:
            raise AuthError(f'Password must be at least {MIN_PASSWORD} characters')
        u = self.users[username]
        u['salt'], u['hash'], u['rounds'] = _hash_password(new)
        self._drop_sessions(username)
        self._save()

    def _drop_sessions(self, username):
        self.sessions = {k: v for k, v in self.sessions.items() if v['user'] != username}

    # ------------------------------------------------------------------ admin
    def list_users(self):
        with self.lock:
            order = {'pending': 0, 'active': 1, 'disabled': 2}
            return sorted((self.public(n, u) for n, u in self.users.items()),
                          key=lambda x: (order[x['status']], -RANK[x['role']], x['username']))

    def pending_count(self):
        with self.lock:
            return sum(1 for u in self.users.values() if u['status'] == 'pending')

    def admin_update(self, actor, username, role=None, status=None):
        """Approve (status=active), change role, disable/enable. Never leaves the robot without an admin."""
        with self.lock:
            u = self.users.get(username)
            if not u:
                raise AuthError('No such user', 404)
            if role is not None and role not in ROLES:
                raise AuthError('Unknown role')
            if status is not None and status not in ('active', 'disabled'):
                raise AuthError('Unknown status')
            losing_admin = (u['role'] == 'admin' and u['status'] == 'active'
                            and ((role and role != 'admin') or status == 'disabled'))
            if losing_admin and self._active_admins() == [username]:
                raise AuthError('This is the only admin: make someone else admin first', 409)
            if role:
                u['role'] = role
            if status:
                u['status'] = status
                if status == 'disabled':
                    self._drop_sessions(username)
            self._save()
            return self.public(username, u)

    def admin_reset_password(self, username, new):
        with self.lock:
            if username not in self.users:
                raise AuthError('No such user', 404)
            self._set_password(username, new)

    def admin_delete(self, actor, username):
        with self.lock:
            u = self.users.get(username)
            if not u:
                raise AuthError('No such user', 404)
            if u['role'] == 'admin' and u['status'] == 'active' and self._active_admins() == [username]:
                raise AuthError('This is the only admin: make someone else admin first', 409)
            del self.users[username]
            self._drop_sessions(username)
            self._save()


# ---------------------------------------------------------------------- permissions
# /api/nav request types a staff member may send: go somewhere, call/stop, ask, look up
STAFF_NAV = {'goto_place', 'goto_room', 'command', 'preview', 'cancel', 'resume', 'go_home', 'go_dock',
             'confirm', 'explain', 'find', 'search', 'schedule_list', 'mission_log'}


def required_role(method, path, body=None):
    """Minimum role for an API call; None = public (no login needed)."""
    if path in ('/api/ping', '/api/auth/login', '/api/auth/signup'):
        return None
    if path.startswith('/api/users'):
        return 'admin'
    if path in ('/api/state', '/api/map', '/api/camera.jpg', '/api/auth/me', '/api/auth/logout',
                '/api/auth/password'):
        return 'staff'
    if path == '/api/estop':
        # anyone logged in can STOP the robot; releasing the stop needs the robot team
        return 'staff' if (body or {}).get('on', True) else 'operator'
    if path == '/api/nav':
        return 'staff' if (body or {}).get('type') in STAFF_NAV else 'operator'
    return 'operator'          # driving, goals, maps, relocalize, ESP32 tuning, tests


def _cli(argv):
    import getpass
    if len(argv) < 2:
        print(__doc__)
        return 2
    store = UserStore(os.path.expanduser(argv[0]))
    cmd = argv[1]
    if cmd == 'list':
        for u in store.list_users():
            print(f"{u['username']:20} {u['role']:9} {u['status']}")
    elif cmd == 'reset-admin' and len(argv) >= 3:
        name = argv[2].lower()
        pw = argv[3] if len(argv) > 3 else getpass.getpass('New password: ')
        with store.lock:
            if name not in store.users:
                salt, h, rounds = _hash_password(pw)
                store.users[name] = {'name': name, 'role': 'admin', 'status': 'active', 'salt': salt,
                                     'hash': h, 'rounds': rounds, 'created': int(time.time()), 'last_login': None}
                store._save()
            else:
                store.users[name].update(role='admin', status='active')
                store._set_password(name, pw)
        print(f'{name} is now an active admin with the new password.')
    else:
        print(__doc__)
        return 2
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(_cli(sys.argv[1:]))
