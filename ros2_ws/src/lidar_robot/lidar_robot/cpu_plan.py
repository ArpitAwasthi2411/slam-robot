"""Who gets the CPU first on the Pi 4 (4 cores): launch prefixes for the less urgent nodes (docs/compute.md).

nice: higher = lower priority (lowering needs no root). cores: a taskset core list, or None = any core.
Motion-critical programs (esp32_bridge, navigator, Cartographer, the LiDAR driver) are not listed: they keep
nice 0 and every core, so the camera, the app server or the voice can never slow driving down.
"""
import os
import shutil

CPU_PLAN = {
    'vision': (10, '3'),                              # YOLO + camera: at most one core, and only the leftovers
    'cartographer_occupancy_grid_node': (5, None),    # redraws the map picture every few seconds: can wait
    'dashboard': (5, None),                           # the app's web server
    'voice': (5, None),
}


def cpu_prefix(name, enabled=True, which=shutil.which):
    """Launch prefix such as 'taskset -c 3 nice -n 10' for a node, or None (not planned / tools missing)."""
    if not enabled or name not in CPU_PLAN:
        return None
    nice, cores = CPU_PLAN[name]
    parts = []
    if cores is not None and which('taskset'):
        highest = max(int(c) for c in cores.replace('-', ',').split(','))
        if (os.cpu_count() or 1) > highest:
            parts += ['taskset', '-c', cores]
    if nice and which('nice'):
        parts += ['nice', '-n', str(nice)]
    return ' '.join(parts) or None
