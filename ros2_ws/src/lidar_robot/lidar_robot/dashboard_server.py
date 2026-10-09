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
The mobile app (web/app/) is served at / ; the classic dashboard at /classic.
Every response carries CORS headers so the APK (file:// origin) can call the API.
The ROS node (dashboard.py) and the offline simulator (tools/dashboard_sim.py)
both implement this interface.
"""
import gzip
import json
import mimetypes
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

MAX_BODY = 4096


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


def make_handler(backend, index_path, app_dir=None):
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
                      and not ctype.startswith(('image/png', 'font/')))
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
            self.send_header('Access-Control-Allow-Headers', 'Content-Type')
            self.send_header('Access-Control-Max-Age', '600')
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

        def _optional(self, name, *args):
            fn = getattr(backend, name, None)
            if fn is None:
                return self._send(404, {'error': f'{name} not supported by this backend'})
            return self._send(200, fn(*args))

        def do_GET(self):
            url = urlparse(self.path)
            q = parse_qs(url.query)
            try:
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
                    return self._optional('ping')
                if url.path == '/api/telemetry':
                    return self._optional('get_telemetry', int(q.get('since', ['0'])[0]))
                if url.path == '/api/test':
                    return self._optional('test_status', int(q.get('since', ['0'])[0]))
                if url.path == '/api/state':
                    return self._send(200, backend.get_state())
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
                if url.path == '/api/cmd':
                    backend.command(float(body.get('v', 0.0)), float(body.get('w', 0.0)))
                elif url.path == '/api/estop':
                    backend.set_estop(bool(body.get('on', True)))
                elif url.path == '/api/goal':
                    backend.send_goal(float(body['x']), float(body['y']), float(body.get('yaw', 0.0)))
                elif url.path == '/api/cancel':
                    backend.cancel_goal()
                elif url.path == '/api/nav':
                    return self._send(200, backend.nav_request(body))
                elif url.path == '/api/save_map':
                    return self._send(200, {'ok': True, 'saved': backend.save_map(str(body.get('name', 'map')))})
                elif url.path == '/api/esp':
                    return self._optional('esp_command', body)
                elif url.path == '/api/test':
                    if body.get('action') == 'stop':
                        return self._optional('test_stop')
                    return self._optional('test_start', body)
                else:
                    return self._send(404, {'error': 'not found'})
                return self._send(200, {'ok': True})
            except (ValueError, KeyError, TypeError) as e:
                return self._send(400, {'error': str(e)})
            except Exception as e:
                return self._send(500, {'error': str(e)})

    return Handler


class DashboardServer:
    def __init__(self, backend, host='0.0.0.0', port=8080):
        self.httpd = ThreadingHTTPServer((host, port),
                                         make_handler(backend, _find_index_html(), _find_app_dir()))
        self.httpd.daemon_threads = True
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.httpd.shutdown()
        self.httpd.server_close()
