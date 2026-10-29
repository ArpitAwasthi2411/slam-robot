"""Is the robot on a network people can reach it through? (pure Python + one ping, unit-tested)

"Up" means: a default route exists on a real interface (Wi-Fi / phone hotspot / LAN) and its gateway answers a
ping. That is what the app needs to reach the robot (directly on the hotspot, or through Tailscale).
Groq/internet being down does NOT count as "network down": commands and recovery fall back to offline rules.

A direct laptop cable (169.254.x.x, no default route) is "no network", but the navigator only acts on a network
loss after the network has been up at least once since start, so lab sessions on the cable are not affected.
"""
import socket
import struct
import subprocess

SKIP_IFACES = ('lo', 'docker', 'veth', 'br-', 'virbr', 'tailscale')


def default_gateways(route_text):
    """[(iface, gateway_ip)] from the text of /proc/net/route (default routes only, real interfaces only)."""
    out = []
    for line in route_text.splitlines()[1:]:
        f = line.split()
        if len(f) < 4 or f[1] != '00000000':
            continue
        iface, gw_hex, flags = f[0], f[2], int(f[3], 16)
        if iface.startswith(SKIP_IFACES) or not flags & 0x1:      # RTF_UP
            continue
        try:
            gw = socket.inet_ntoa(struct.pack('<I', int(gw_hex, 16)))
        except (ValueError, struct.error):
            continue
        out.append((iface, gw))
    return out


def ping(host, timeout_s=1):
    try:
        r = subprocess.run(['ping', '-c', '1', '-W', str(int(timeout_s)), host],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=timeout_s + 2)
        return r.returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def network_ok(route_text=None, ping_fn=ping):
    """True if any default gateway answers. route_text / ping_fn are for tests."""
    if route_text is None:
        try:
            with open('/proc/net/route') as f:
                route_text = f.read()
        except OSError:
            return False
    for _iface, gw in default_gateways(route_text):
        if gw == '0.0.0.0' or ping_fn(gw):            # 0.0.0.0: point-to-point link, no gateway to ping
            return True
    return False
