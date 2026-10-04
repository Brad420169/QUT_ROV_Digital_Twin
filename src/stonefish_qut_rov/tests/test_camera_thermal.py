"""Temperature boundaries and heartbeat fail-closed behavior; no hardware."""
import sys
from pathlib import Path
from camera_thermal import VisionGate, VisionPermission


def test_hysteresis():
    g = VisionGate()
    assert g.evaluate(70, 0, 24, 0)[0]
    assert not g.evaluate(80, 0, 24, 0)[0]
    assert not g.evaluate(79, 0, 5, 0)[0]
    assert not g.evaluate(75, 0, 24, 0)[0]
    assert not g.evaluate(74, 0, 5, 0)[0]
    assert g.evaluate(74, 0, 24, 0)[0]


def test_restart_while_cooling_and_unknown():
    g = VisionGate()
    assert not g.evaluate(77, 0, 24, 0)[0]
    assert not g.evaluate(None, 0, 24, 0)[0]
    assert not g.evaluate(70, 9, 24, 0)[0]
    assert not g.evaluate(float('nan'), 0, 24, 0)[0]
    assert not g.evaluate(70, 0, 24, 2)[0]


def test_permission_expiry():
    now = [0.]
    p = VisionPermission(lambda: now[0])
    assert not p.valid()
    p.update(True)
    assert p.valid()
    now[0] = 2.
    assert not p.valid()
    p.update(False)
    assert not p.valid()
