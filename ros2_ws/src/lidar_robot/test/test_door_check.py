"""Camera door check (web/app/doorcheck.js): taught a door closed and open, it must tell them apart under other
lighting, sensor noise and a robot that stops a few pixels off, and say 'unknown' for a different scene."""
import json
import os
import shutil
import subprocess

import pytest

APP = os.path.join(os.path.dirname(__file__), '..', 'lidar_robot', 'web', 'app')

SCENES = r"""
const D = require(process.argv[1]);
let seed = 7; const rnd = () => (seed = (seed * 1103515245 + 12345) % 2147483648) / 2147483648;
const W = 160, H = 120;
function scene(kind, o) {               // o: {gain, bias, noise, dx, dy, seed}
  seed = o.seed || 7;
  const g = new Float32Array(W * H);
  const wall = 170, frame = 90;
  // the room behind an open door: shelves, a desk, a window: lots of edges (fixed layout, own seed)
  const room = []; let s2 = 99; const r2 = () => (s2 = (s2 * 1103515245 + 12345) % 2147483648) / 2147483648;
  for (let k = 0; k < 40; k++) room.push([40 + r2() * 70, 15 + r2() * 95, 4 + r2() * 20, 3 + r2() * 15, 20 + r2() * 200]);
  for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
    const X = x - (o.dx || 0), Y = y - (o.dy || 0);
    let v = wall;
    const inDoor = X >= 45 && X <= 115 && Y >= 12 && Y <= 118;
    const inFrame = X >= 40 && X <= 120 && Y >= 8 && !inDoor;
    if (inFrame) v = frame;
    if (inDoor) {
      if (kind === 'closed') { v = 120 + (X - 45) * 0.15; if (X >= 102 && X <= 108 && Y >= 60 && Y <= 64) v = 230; }
      else if (kind === 'open') {
        v = 60;
        for (const [rx, ry, rw, rh, c] of room) if (X >= rx && X <= rx + rw && Y >= ry && Y <= ry + rh) v = c;
      } else v = 140 + 40 * Math.sin(X / 5) * Math.cos(Y / 7);       // 'other': a notice board, something else
    }
    v = v * (o.gain || 1) + (o.bias || 0) + (o.noise || 0) * (rnd() - 0.5) * 2;
    g[y * W + x] = Math.max(0, Math.min(255, v));
  }
  return D.signature(g, W, H);
}
const refs = { closed: [scene('closed', {}), scene('closed', { gain: 0.8, seed: 3 })],
               open: [scene('open', {}), scene('open', { gain: 1.1, seed: 4 })] };
const cases = JSON.parse(require('fs').readFileSync(0, 'utf8'));
const out = cases.map(c => D.classify(scene(c.kind, c), c.only ? { [c.only]: refs[c.only] } : refs));
out.push(D.classify(scene('closed', {}), { open: [], closed: [] }));
console.log(JSON.stringify(out));
"""


def run(cases):
    node = shutil.which('node')
    if not node:
        pytest.skip('node not installed')
    r = subprocess.run([node, '-e', SCENES, os.path.abspath(os.path.join(APP, 'doorcheck.js'))],
                       input=json.dumps(cases), capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, r.stderr
    return json.loads(r.stdout)


def test_door_open_closed_under_changes():
    variations = [{}, {'gain': 0.55, 'bias': 10}, {'gain': 1.3, 'bias': -15}, {'noise': 25, 'seed': 11},
                  {'dx': 4, 'dy': -3}, {'dx': -6, 'dy': 2, 'gain': 0.7, 'noise': 15, 'seed': 5}]
    cases = [dict(v, kind=k) for k in ('closed', 'open') for v in variations]
    res = run(cases)
    for c, r in zip(cases, res):
        assert r['state'] == c['kind'], (c, r)


def test_unknown_when_it_is_not_that_door_or_not_taught():
    res = run([{'kind': 'other'}])
    assert res[0]['state'] == 'unknown'                  # a notice board is neither the open nor the closed door
    assert res[-1]['state'] == 'unknown' and 'not taught' in res[-1]['why']


def test_only_closed_taught():
    res = run([{'kind': 'closed', 'only': 'closed', 'gain': 0.8}, {'kind': 'open', 'only': 'closed'}])
    assert res[0]['state'] == 'closed'
    assert res[1]['state'] == 'unknown'
