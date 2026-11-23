"""Nothing clickable may hide under the kiosk STOP button or the Drive tab's joystick, readout, speed slider or
tool rail, on 8 phone/tablet sizes upright and sideways (overlap_check.js drives the demo app in Chromium).
Skipped when node or the playwright package is missing."""
import os
import shutil
import subprocess

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
APP = os.path.join(HERE, '..', 'lidar_robot', 'web', 'app')


def test_nothing_under_stop_or_joystick():
    node = shutil.which('node')
    if not node:
        pytest.skip('node not installed')
    try:
        out = subprocess.run([node, os.path.join(HERE, 'overlap_check.js'), os.path.abspath(APP)],
                             capture_output=True, text=True, timeout=500)
    except subprocess.TimeoutExpired:
        pytest.fail('overlap_check.js timed out')
    if "Cannot find module 'playwright'" in out.stderr:
        pytest.skip('playwright not installed')
    lines = out.stdout.strip().splitlines()
    assert lines and lines[-1] == 'problems: 0', out.stdout[-3000:] + out.stderr[-2000:]
