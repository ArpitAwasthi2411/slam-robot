"""Save an OccupancyGrid as map_server-compatible .pgm + .yaml (no nav2 needed)."""
import math
import os
import re


def occupancy_to_pgm_bytes(width, height, data, occupied_thresh=65, free_thresh=25):
    """data: row-major int8 list, row 0 = bottom (ROS). Returns P5 bytes, row 0 = top."""
    out = bytearray()
    for row in range(height - 1, -1, -1):
        base = row * width
        for col in range(width):
            v = data[base + col]
            if v < 0:
                out.append(205)          # unknown
            elif v >= occupied_thresh:
                out.append(0)            # occupied
            elif v <= free_thresh:
                out.append(254)          # free
            else:
                out.append(205)
    header = f'P5\n# CREATOR: lidar_robot mapsave\n{width} {height}\n255\n'.encode()
    return header + bytes(out)


def safe_name(name: str) -> str:
    name = re.sub(r'[^A-Za-z0-9_\-]', '_', name.strip()) or 'map'
    return name[:60]


def save_map(directory, name, width, height, resolution, origin_x, origin_y, origin_yaw, data):
    os.makedirs(directory, exist_ok=True)
    name = safe_name(name)
    pgm_path = os.path.join(directory, name + '.pgm')
    yaml_path = os.path.join(directory, name + '.yaml')
    with open(pgm_path, 'wb') as f:
        f.write(occupancy_to_pgm_bytes(width, height, data))
    with open(yaml_path, 'w') as f:
        f.write(
            f'image: {name}.pgm\n'
            f'mode: trinary\n'
            f'resolution: {resolution:.6f}\n'
            f'origin: [{origin_x:.6f}, {origin_y:.6f}, {origin_yaw if math.isfinite(origin_yaw) else 0.0:.6f}]\n'
            f'negate: 0\n'
            f'occupied_thresh: 0.65\n'
            f'free_thresh: 0.25\n')
    return pgm_path, yaml_path
