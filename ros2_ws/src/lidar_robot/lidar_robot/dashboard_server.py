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
The ROS node (dashboard.py) and the offline simulator (tools/dashboard_sim.py)
both implement this interface.
"""
import json
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


def make_handler(backend, index_path):
    class Handler(BaseHTTPRequestHandler):
        server_version = 'RobotDashboard/1.0'

        def log_message(self, fmt, *args):     # keep the ROS console clean
            pass

        def _send(self, code, body, ctype='application/json'):
            if isinstance(body, (dict, list)):
                body = json.dumps(body, separators=(',', ':')).encode()
            elif isinstance(body, str):
                body = body.encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def _json_body(self):
            n = int(self.headers.get('Content-Length') or 0)
            if n <= 0:
                return {}
            if n > MAX_BODY:
                raise ValueError('body too large')
            return json.loads(self.rfile.read(n) or b'{}')

        def do_GET(self):
            url = urlparse(self.path)
            try:
                if url.path in ('/', '/index.html'):
                    with open(index_path, 'rb') as f:
                        return self._send(200, f.read(), 'text/html; charset=utf-8')
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
        self.httpd = ThreadingHTTPServer((host, port), make_handler(backend, _find_index_html()))
        self.httpd.daemon_threads = True
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.httpd.shutdown()
        self.httpd.server_close()
