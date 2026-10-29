"""Tiny dependency-free HTTP server for the teleop dashboard (stdlib only).

The server knows nothing about ROS. It talks to a `backend` object:
    backend.get_state() -> dict
    backend.get_map(since_version:int) -> dict | None
    backend.command(v, w)
    backend.set_estop(on: bool)
    backend.send_goal(x, y, yaw)
    backend.cancel_goal()
    backend.save_map(name) -> str
    backend.nav_request(dict) -> dict     (navigator: places, routes, missions, commands)
  Tuning Lab / mobile app (optional; 404 if the backend lacks them):
    backend.ping() -> dict
    backend.esp_command(dict) -> dict     ({"op": "tune"|"save"|"defaults"|"telemetry"|"query"})
    backend.get_telemetry(since) -> dict
    backend.test_start(dict) / test_stop() / test_status(since) -> dict
    backend.camera_jpeg() -> bytes | None      (latest annotated camera picture, GET /api/camera.jpg)
The mobile app (web/app/) is served at / ; the classic dashboard at /classic.
Every response carries CORS headers so the APK (file:// origin) can call the API.

Login (auth.py): pass `users=UserStore(...)` and every /api/ call needs
`Authorization: Bearer <token>` with a role that allows it (auth.required_role).
users=None = no login at all (only for unit tests and the classic offline setups).
  POST /api/auth/signup {username, password, name}   first account = admin, later ones wait for approval
  POST /api/auth/login  {username, password}          -> {token, user}
  GET  /api/auth/me                                    -> {user, pending}
  POST /api/auth/logout | /api/auth/password {old, new}
  GET  /api/users        POST /api/users/update {username, role?, status?}
  POST /api/users/reset {username, password}         POST /api/users/delete {username}
The ROS node (dashboard.py) and the offline simulator (tools/dashboard_sim.py)
both implement this interface.
"""
import gzip
import json
import math
import mimetypes
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

from lidar_robot.auth import AuthError, at_least, required_role

MAX_BODY = 4096


def _num(body, key, default=None):
    """A finite float from the JSON body (NaN/inf would reach the motors as garbage)."""
    v = body.get(key, default)
    if v is None:
        raise KeyError(key)
    v = float(v)
    if not math.isfinite(v):
        raise ValueError(f'{key} must be a finite number')
    return v


def _find_index_html():
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [os.path.join(here, 'web', 'index.html')]
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.insert(0, os.path.join(get_package_share_directory('lidar_robot'),
                                          'web', 'index.html'))
    except Exception:
        pass
    for c in candidates:
        if os.path.isfile(c):
            return c
    raise FileNotFoundError('dashboard index.html not found in: ' + ', '.join(candidates))


def _find_app_dir():
    d = os.path.join(os.path.dirname(_find_index_html()), 'app')
    return d if os.path.isfile(os.path.join(d, 'index.html')) else None


STATIC_TYPES = {'.html': 'text/html; charset=utf-8', '.js': 'application/javascript; charset=utf-8',
                '.css': 'text/css; charset=utf-8', '.svg': 'image/svg+xml', '.png': 'image/png',
                '.json': 'application/json', '.webmanifest': 'application/manifest+json',
                '.ico': 'image/x-icon', '.woff2': 'font/woff2'}


def make_handler(backend, index_path, app_dir=None, users=None):
    class Handler(BaseHTTPRequestHandler):
        server_version = 'RobotDashboard/1.0'

        def log_message(self, fmt, *args):     # keep the ROS console clean
            pass

        def _send(self, code, body, ctype='application/json'):
            if isinstance(body, (dict, list)):
                body = json.dumps(body, separators=(',', ':')).encode()
            elif isinstance(body, str):
                body = body.encode()
            zipped = (len(body) > 1200 and 'gzip' in (self.headers.get('Accept-Encoding') or '')
                      and not ctype.startswith(('image/png', 'image/jpeg', 'font/')))
            if zipped:                     # ~4x less data: matters when the phone is on mobile data
                body = gzip.compress(body, compresslevel=5)
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            if zipped:
                self.send_header('Content-Encoding', 'gzip')
                self.send_header('Vary', 'Accept-Encoding')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(body)

        def _json_body(self):
            n = int(self.headers.get('Content-Length') or 0)
            if n <= 0:
                return {}
            if n > MAX_BODY:
                raise ValueError('body too large')
            return json.loads(self.rfile.read(n) or b'{}')

        def do_OPTIONS(self):                    # CORS preflight from the APK
            self.send_response(204)
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
            self.send_header('Access-Control-Allow-Headers', 'Content-Type, Authorization')
            self.send_header('Access-Control-Max-Age', '7200')
            self.send_header('Content-Length', '0')
            self.end_headers()

        def _static(self, rel):
            if not app_dir:
                return self._send(404, {'error': 'app not installed'})
            full = os.path.realpath(os.path.join(app_dir, rel))
            if not full.startswith(os.path.realpath(app_dir) + os.sep) or not os.path.isfile(full):
                return self._send(404, {'error': 'not found'})
            ext = os.path.splitext(full)[1].lower()
            ctype = STATIC_TYPES.get(ext) or mimetypes.guess_type(full)[0] or 'application/octet-stream'
            with open(full, 'rb') as f:
                return self._send(200, f.read(), ctype)

        # ------------------------------------------------------------ login
        def _token(self):
            h = self.headers.get('Authorization') or ''
            return h[7:].strip() if h.lower().startswith('bearer ') else ''

        def _guard(self, method, path, body=None):
            """Sets self.user; returns True if the request may go on (else the error is already sent)."""
            self.user = None
            if users is None or not path.startswith('/api/'):
                return True
            need = required_role(method, path, body)
            self.user = users.verify(self._token())
            if need is None:
                return True
            if self.user is None:
                self._send(401, {'error': 'Please log in again', 'login': True})
                return False
            if not at_least(self.user['role'], need):
                self._send(403, {'error': f'Your role ({self.user["role"]}) cannot do this. Ask an {need}.',
                                 'need': need})
                return False
            return True

        def _auth_post(self, path, body):
            addr = self.client_address[0] if self.client_address else ''
            if path == '/api/auth/login':
                return users.login(body.get('username'), body.get('password'), addr)
            if path == '/api/auth/signup':
                r = users.signup(body.get('username'), body.get('password'), body.get('name', ''))
                r['message'] = ('Welcome! You are the first user, so you are the admin.' if r['token'] else
                                'Account created. An admin has to approve it before you can log in.')
                return r
            if path == '/api/auth/logout':
                users.logout(self._token())
                return {'ok': True}
            if path == '/api/auth/password':
                users.change_password(self.user['username'], body.get('old'), body.get('new'))
                return {'ok': True, 'message': 'Password changed. Log in again on your other devices.'}
            if path == '/api/users/update':
                return {'ok': True, 'user': users.admin_update(self.user['username'], str(body.get('username', '')),
                                                                body.get('role'), body.get('status'))}
            if path == '/api/users/reset':
                users.admin_reset_password(str(body.get('username', '')), body.get('password'))
                return {'ok': True}
            if path == '/api/users/delete':
                if body.get('username') == self.user['username']:
                    raise AuthError('You cannot remove your own account')
                users.admin_delete(self.user['username'], str(body.get('username', '')))
                return {'ok': True}
            return None

        def _optional(self, name, *args):
            fn = getattr(backend, name, None)
            if fn is None:
                return self._send(404, {'error': f'{name} not supported by this backend'})
            return self._send(200, fn(*args))

        def do_GET(self):
            url = urlparse(self.path)
            q = parse_qs(url.query)
            try:
                if not self._guard('GET', url.path):
                    return
                if users is not None and url.path == '/api/auth/me':
                    return self._send(200, {'user': self.user,
                                            'pending': users.pending_count() if self.user['role'] == 'admin' else 0})
                if users is not None and url.path == '/api/users':
                    return self._send(200, {'users': users.list_users()})
                if url.path in ('/', '/index.html'):
                    if app_dir:                          # the app's relative URLs need the /app/ base
                        self.send_response(302)
                        self.send_header('Location', '/app/')
                        self.send_header('Content-Length', '0')
                        self.end_headers()
                        return
                    with open(index_path, 'rb') as f:
                        return self._send(200, f.read(), 'text/html; charset=utf-8')
                if url.path in ('/classic', '/classic/'):
                    with open(index_path, 'rb') as f:
                        return self._send(200, f.read(), 'text/html; charset=utf-8')
                if url.path == '/app':
                    self.send_response(301)
                    self.send_header('Location', '/app/')
                    self.send_header('Content-Length', '0')
                    self.end_headers()
                    return
                if url.path.startswith('/app/'):
                    return self._static(url.path[len('/app/'):] or 'index.html')
                if url.path == '/api/ping':
                    fn = getattr(backend, 'ping', None)
                    if fn is None:
                        return self._send(404, {'error': 'ping not supported by this backend'})
                    return self._send(200, dict(fn(), auth=users is not None,
                                                setup_needed=bool(users and users.setup_needed())))
                if url.path == '/api/telemetry':
                    return self._optional('get_telemetry', int(q.get('since', ['0'])[0]))
                if url.path == '/api/test':
                    return self._optional('test_status', int(q.get('since', ['0'])[0]))
                if url.path == '/api/state':
                    return self._send(200, backend.get_state())
                if url.path == '/api/camera.jpg':
                    fn = getattr(backend, 'camera_jpeg', None)
                    jpg = fn() if fn else None
                    if not jpg:
                        return self._send(404, {'error': 'no camera picture'})
                    return self._send(200, jpg, 'image/jpeg')
                if url.path == '/api/map':
                    since = int(parse_qs(url.query).get('since', ['-1'])[0])
                    m = backend.get_map(since)
                    return self._send(200, m if m is not None else {'unchanged': True})
                return self._send(404, {'error': 'not found'})
            except Exception as e:                       # never kill the server thread
                return self._send(500, {'error': str(e)})

        def do_POST(self):
            url = urlparse(self.path)
            try:
                body = self._json_body()
                if not isinstance(body, dict):
                    raise ValueError('body must be a JSON object')
                if not self._guard('POST', url.path, body):
                    return
                if users is not None and url.path.startswith(('/api/auth/', '/api/users/')):
                    r = self._auth_post(url.path, body)
                    return self._send(200, r) if r is not None else self._send(404, {'error': 'not found'})
                if self.user and url.path == '/api/nav':
                    body['by'] = self.user['username']
                if url.path == '/api/cmd':
                    backend.command(_num(body, 'v', 0.0), _num(body, 'w', 0.0))
                elif url.path == '/api/estop':
                    backend.set_estop(bool(body.get('on', True)))
                elif url.path == '/api/goal':
                    backend.send_goal(_num(body, 'x'), _num(body, 'y'), _num(body, 'yaw', 0.0))
                elif url.path == '/api/cancel':
                    backend.cancel_goal()
                elif url.path == '/api/nav':
                    return self._send(200, backend.nav_request(body))
                elif url.path == '/api/save_map':
                    return self._send(200, {'ok': True, 'saved': backend.save_map(str(body.get('name', 'map')))})
                elif url.path == '/api/relocalize':
                    return self._optional('relocalize', body)
                elif url.path == '/api/esp':
                    return self._optional('esp_command', body)
                elif url.path == '/api/test':
                    if body.get('action') == 'stop':
                        return self._optional('test_stop')
                    return self._optional('test_start', body)
                else:
                    return self._send(404, {'error': 'not found'})
                return self._send(200, {'ok': True})
            except AuthError as e:
                return self._send(e.code, {'error': str(e)})
            except (ValueError, KeyError, TypeError) as e:
                return self._send(400, {'error': str(e)})
            except Exception as e:
                return self._send(500, {'error': str(e)})

    return Handler


class DashboardServer:
    def __init__(self, backend, host='0.0.0.0', port=8080, users=None):
        self.httpd = ThreadingHTTPServer((host, port),
                                         make_handler(backend, _find_index_html(), _find_app_dir(), users))
        self.httpd.daemon_threads = True
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.httpd.shutdown()
        self.httpd.server_close()
