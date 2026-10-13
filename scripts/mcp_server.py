#!/usr/bin/env python3
"""Pathik MCP server: lets an AI assistant (Claude Desktop, Claude Code, any MCP client) use the robot.

Runs on your LAPTOP (or the Pi) and talks to the robot's HTTP API. No extra packages needed.

    ROBOT_URL=http://slam-robot:8080 python3 scripts/mcp_server.py      # stdio MCP server

Claude Desktop -> Settings -> Developer -> Edit config (claude_desktop_config.json):
    {"mcpServers": {"pathik": {"command": "python3",
        "args": ["/path/to/slam-robot/scripts/mcp_server.py"],
        "env": {"ROBOT_URL": "http://slam-robot:8080"}}}}
Claude Code:  claude mcp add pathik -e ROBOT_URL=http://slam-robot:8080 -- python3 scripts/mcp_server.py

Then ask: "Where is Pathik and how much battery is left?", "Send it to the room with the 3D printer",
"Why did it stop?", "Schedule a patrol of Lab 2 and Lab 3 every weekday at 10".
Safety: the AI can only do what the app can do; the e-stop, RC override, battery rules and the
robot's own obstacle checks still apply. Moving the robot needs the robot on a saved map.
"""
import json
import os
import sys
import urllib.error
import urllib.request

ROBOT = os.environ.get('ROBOT_URL', 'http://10.42.0.1:8080').rstrip('/')
PROTOCOL = '2024-11-05'


def http(path, body=None, timeout=6.0):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(ROBOT + path, data=data, method='POST' if body is not None else 'GET',
                                 headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read().decode() or '{}')
    except urllib.error.HTTPError as e:
        try:
            return json.loads(e.read().decode())
        except ValueError:
            return {'ok': False, 'error': f'HTTP {e.code}'}
    except (urllib.error.URLError, OSError) as e:
        return {'ok': False, 'error': f'robot not reachable at {ROBOT}: {e}'}


def nav(req):
    return http('/api/nav', req)


def status():
    st = http('/api/state')
    if st.get('ok') is False:
        return st
    n = st.get('nav') or {}
    m = (n.get('missions') or {}).get('active')
    return {
        'state': n.get('state'), 'message': n.get('message'), 'room': n.get('room'),
        'pose': st.get('pose'), 'battery': n.get('battery'), 'docked': n.get('docked'),
        'estop': st.get('estop'), 'localized': n.get('localized'),
        'mission': m and {'text': m.get('text'), 'stops': m.get('stops'), 'next_stop': m.get('next_stop')},
        'queued': len((n.get('missions') or {}).get('queue') or []),
        'waiting_for_ok': n.get('confirm'), 'last_failure': n.get('last_failure'),
    }


def places():
    st = http('/api/state')
    p = st.get('places') or {}
    return {'places': [x['name'] for x in p.get('places', [])], 'home_set': bool(p.get('home')),
            'dock_set': bool(p.get('dock')),
            'rooms': [{'name': r['name'], 'contains': r.get('contents', [])} for r in p.get('rooms', [])]}


def go_to(name, priority):
    r = nav({'type': 'goto_place', 'name': name, 'priority': priority})
    if r.get('ok') is False and 'no place matches' in str(r.get('error')):
        r = nav({'type': 'goto_room', 'name': name, 'priority': priority})
    return r


TOOLS = [
    ('robot_status', 'Where the robot is, what it is doing, battery, e-stop, current mission.', {}, lambda a: status()),
    ('list_places', 'Saved places, rooms and what each room contains.', {}, lambda a: places()),
    ('go_to', 'Send the robot to a saved place or room by name (queued after current tasks unless urgent).',
     {'name': {'type': 'string'}, 'urgent': {'type': 'boolean'}},
     lambda a: go_to(a['name'], 3 if a.get('urgent') else 1)),
    ('go_to_thing', 'Send the robot to the room that contains a thing, e.g. "3D printer".',
     {'thing': {'type': 'string'}}, lambda a: nav({'type': 'goto_room', 'thing': a['thing']})),
    ('command', 'A natural-language task, e.g. "take this to Lab 3, wait for OK, then come back". '
                'Set dry_run to only see the plan.',
     {'text': {'type': 'string'}, 'dry_run': {'type': 'boolean'}},
     lambda a: nav({'type': 'command', 'text': a['text'], 'dry_run': bool(a.get('dry_run'))})),
    ('find_thing', 'Which room something is in.', {'thing': {'type': 'string'}},
     lambda a: nav({'type': 'find', 'q': a['thing']})),
    ('explain', 'Why the robot stopped / what went wrong, in plain words.', {},
     lambda a: nav({'type': 'explain'})),
    ('cancel', 'Cancel all missions (the robot stops where it is).', {}, lambda a: nav({'type': 'cancel'})),
    ('go_home', 'Drive back home.', {}, lambda a: nav({'type': 'go_home', 'replace': True})),
    ('charge', 'Drive to the charging dock and back onto it.', {}, lambda a: nav({'type': 'go_dock', 'replace': True})),
    ('confirm_ok', 'Press OK for a "wait for OK" stop.', {}, lambda a: nav({'type': 'confirm'})),
    ('schedule', 'Run a task every day (or on days 0=Mon..6=Sun) at HH:MM robot time. task_text is a command.',
     {'at': {'type': 'string'}, 'task_text': {'type': 'string'}, 'days': {'type': 'array', 'items': {'type': 'integer'}},
      'name': {'type': 'string'}},
     lambda a: nav({'type': 'schedule_add', 'name': a.get('name') or a['task_text'][:40], 'at': a['at'],
                    'days': a.get('days') or [], 'action': {'type': 'command', 'text': a['task_text']}})),
    ('mission_stats', 'Evaluation numbers: success rate, times, distance, replans, recent missions.',
     {'n': {'type': 'integer'}}, lambda a: nav({'type': 'mission_log', 'n': int(a.get('n') or 20)})),
]
REQUIRED = {'go_to': ['name'], 'go_to_thing': ['thing'], 'command': ['text'], 'find_thing': ['thing'],
            'schedule': ['at', 'task_text']}


def tool_list():
    return [{'name': n, 'description': d,
             'inputSchema': {'type': 'object', 'properties': props, 'required': REQUIRED.get(n, [])}}
            for n, d, props, _ in TOOLS]


def handle(msg):
    mid, method, params = msg.get('id'), msg.get('method'), msg.get('params') or {}
    if method == 'initialize':
        return {'protocolVersion': params.get('protocolVersion', PROTOCOL), 'capabilities': {'tools': {}},
                'serverInfo': {'name': 'pathik', 'version': '2.4'},
                'instructions': 'Tools to drive the Pathik campus robot. Check robot_status before sending it '
                                'somewhere; never send it while the e-stop is on.'}
    if method == 'tools/list':
        return {'tools': tool_list()}
    if method == 'tools/call':
        name, args = params.get('name'), params.get('arguments') or {}
        fn = next((f for n, _, _, f in TOOLS if n == name), None)
        if fn is None:
            return {'content': [{'type': 'text', 'text': f'unknown tool {name}'}], 'isError': True}
        missing = [k for k in REQUIRED.get(name, []) if k not in args]
        if missing:
            return {'content': [{'type': 'text', 'text': f'missing: {", ".join(missing)}'}], 'isError': True}
        res = fn(args)
        err = isinstance(res, dict) and res.get('ok') is False
        return {'content': [{'type': 'text', 'text': json.dumps(res, indent=1)[:20000]}], 'isError': err}
    if method == 'ping':
        return {}
    if mid is None:                     # notifications (initialized, cancelled...): no reply
        return None
    raise KeyError(method)


def main():
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            msg = json.loads(line)
        except ValueError:
            continue
        try:
            result = handle(msg)
            if msg.get('id') is None:
                continue
            out = {'jsonrpc': '2.0', 'id': msg['id'], 'result': result}
        except KeyError as e:
            out = {'jsonrpc': '2.0', 'id': msg.get('id'), 'error': {'code': -32601, 'message': f'method not found: {e}'}}
        except Exception as e:          # never crash the client session
            out = {'jsonrpc': '2.0', 'id': msg.get('id'), 'error': {'code': -32603, 'message': str(e)}}
        sys.stdout.write(json.dumps(out) + '\n')
        sys.stdout.flush()


if __name__ == '__main__':
    main()
