"""Map building on a laptop riding on the robot (docs/laptop_slam.md). Plain Python, no ROS: tested alone.

The Pi keeps the sensors, motors, navigator, app and voice. With `slam_host:=laptop` it does NOT start
Cartographer while mapping; a laptop on the same Ethernet cable runs it (scripts/laptop_slam.py), so the Pi's
CPU stays free and the map never tears from overload. Only ONE Cartographer may run: both would publish
map->odom.

The dashboard publishes /slam_host (latched JSON, every 2 s):
    {"host": "laptop", "mode": "mapping", "map": "", "boot": 1794..., "save_dir": "/tmp/pathik_maps",
     "map_dir": "/home/arpit/maps", "lua": "cartographer_odom.lua"}
The laptop starts Cartographer when host=laptop and mode=mapping, restarts it when the Pi restarted (new
boot: the odometry started again from zero) or the map to continue changed, and stops it otherwise (e.g. the
map was frozen: localization runs on the Pi itself).

Saving (Freeze / autosave in the app): the dashboard asks Cartographer to write <save_dir>/<name>.pbstream,
which lands on the LAPTOP; the laptop copies every finished file to the Pi's map_dir (rsync), and the
dashboard waits for it to arrive before it calls the map saved.
"""
import json
import os

HOSTS = ('pi', 'laptop')
DEFAULT_SAVE_DIR = '/tmp/pathik_maps'


def status_message(host, mode, map_file, boot, save_dir, map_dir, use_odom=True):
    """What the dashboard publishes on /slam_host."""
    return json.dumps({'host': host if host in HOSTS else 'pi', 'mode': mode,
                       'map': os.path.basename(map_file or ''), 'boot': int(boot),
                       'save_dir': save_dir or DEFAULT_SAVE_DIR, 'map_dir': map_dir,
                       'lua': 'cartographer_odom.lua' if use_odom else 'cartographer_lidar_only.lua'})


def parse_status(text):
    try:
        d = json.loads(text)
    except (TypeError, ValueError):
        return None
    return d if isinstance(d, dict) and d.get('host') in HOSTS else None


def wanted(status):
    """What the laptop should run for this /slam_host message: None = nothing, else a dict with
    'key' (restart Cartographer when it changes) and 'load' (pbstream name to continue, or '')."""
    if not status or status.get('host') != 'laptop' or status.get('mode') != 'mapping':
        return None
    load = status.get('map') or ''
    if load and not load.endswith('.pbstream'):
        load = ''
    return {'key': (status.get('boot'), load), 'load': load}


def decide(status, running_key, pi_seen_age_s, lost_after_s=15.0):
    """'start' | 'restart' | 'stop' | 'keep' for the laptop's Cartographer.

    pi_seen_age_s: seconds since the last /slam_host message. When the Pi goes quiet (restarting, cable out)
    Cartographer keeps running for lost_after_s so a short blip loses nothing; after that it is stopped,
    because a restarted Pi starts odometry from zero and the old map would not fit it any more.
    """
    if pi_seen_age_s is None or pi_seen_age_s > lost_after_s:
        return 'stop' if running_key is not None else 'keep'
    w = wanted(status)
    if w is None:
        return 'stop' if running_key is not None else 'keep'
    if running_key is None:
        return 'start'
    return 'keep' if running_key == w['key'] else 'restart'


def ready_files(folder, sent, now, settle_s=2.0, exts=('.pbstream',)):
    """Files in `folder` that are new or changed since they were sent, and have not changed for settle_s
    (Cartographer writes a big map for a while). `sent`: {name: (size, mtime)} of files already copied."""
    out = []
    try:
        names = sorted(os.listdir(folder))
    except OSError:
        return out
    for n in names:
        if not n.endswith(exts):
            continue
        p = os.path.join(folder, n)
        try:
            st = os.stat(p)
        except OSError:
            continue
        sig = (st.st_size, int(st.st_mtime))
        if st.st_size > 0 and sent.get(n) != sig and now - st.st_mtime >= settle_s:
            out.append((n, sig))
    return out


def clock_note(offset_s):
    """Laptop clock minus the Pi's (from /odom stamps). Cartographer only uses the Pi's stamps, so a
    skew does not break the map, but recordings and logs are easier to read with synced clocks."""
    if offset_s is None:
        return 'no /odom from the robot yet'
    a = abs(offset_s)
    if a < 0.2:
        return f'clocks agree ({offset_s * 1000:+.0f} ms)'
    return (f'laptop clock is {offset_s:+.1f} s off the Pi: the map still works, but sync them '
            '(both online: sudo timedatectl set-ntp true on both)')


def file_arrived(path, since, min_size=1):
    """The dashboard's check that the laptop's copy has landed on the Pi (newer than the save request)."""
    try:
        st = os.stat(path)
    except OSError:
        return False
    return st.st_size >= min_size and st.st_mtime >= since - 1.0


def rviz_env(env, gpu='auto', has_nvidia=False):
    """Environment for RViz on the laptop. RViz froze on 19 Nov ('rviz2 is not responding', Map display
    GLSL link error on the Intel/Mesa driver under Wayland). Always X11 (xcb); on a laptop with an NVIDIA
    card RViz is sent to it (PRIME offload); 'software' draws on the CPU (slow but never hits a driver bug)."""
    e = dict(env)
    e['QT_QPA_PLATFORM'] = 'xcb'
    e.pop('__NV_PRIME_RENDER_OFFLOAD', None)
    e.pop('__GLX_VENDOR_LIBRARY_NAME', None)
    e.pop('LIBGL_ALWAYS_SOFTWARE', None)
    if gpu == 'nvidia' or (gpu == 'auto' and has_nvidia):
        e['__NV_PRIME_RENDER_OFFLOAD'] = '1'
        e['__GLX_VENDOR_LIBRARY_NAME'] = 'nvidia'
    elif gpu == 'software':
        e['LIBGL_ALWAYS_SOFTWARE'] = '1'
    return e
