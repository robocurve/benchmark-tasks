"""Synchronized Cartesian demonstration plans for the standard YAM joint policy."""

import os

from valve_layout import MODES, ORIGIN, PIN, TRAY, knob
from yam_arena.constants import FINGER_OPEN as OPEN
from yam_arena.demo import Waypoint


def demo(mode=None):
    mode = mode or os.environ.get("VALVE_CONTROL", "reference")
    if mode not in MODES:
        raise ValueError(mode)
    if mode == "idle":
        return {"left": [], "right": []}
    wheel = "right" if mode == "alternate" else "left"
    pull = "left" if wheel == "right" else "right"
    sign = lambda side: 1 if side == "left" else -1
    home = {side: (-0.078, sign(side) * 0.212, 0.157, OPEN) for side in (wheel, pull)}
    live = home.copy()
    seq = {s: [] for s in home}

    def phase(sec, label, **poses):
        live.update(poses)
        for side, (x, y, z, gap) in live.items():
            p = tuple(a + b for a, b in zip(ORIGIN, (x, y, z)))
            seq[side].append(Waypoint(sec, p, gap, label))

    def handle(deg, gap=-0.002, z=None):
        x, y, h = knob(wheel, deg)
        return x, y, h if z is None else z, gap

    def pin(z, gap):
        return *PIN[:2], z, gap

    phase(1.6, "above station")
    if mode not in ("idle", "loaded_pull"):
        phase(1.4, "approach override", **{wheel: handle(0, -0.023, 0.175)})
        phase(1.0, "straddle knob", **{wheel: handle(0, -0.023)})
        phase(0.7, "grasp override", **{wheel: handle(0)})
        phase(1.2, "take load", **{wheel: handle(3.9 if wheel == "left" else 1.3)})
        phase(0.75, "stabilize override")
    if mode not in ("idle", "unload_only"):
        phase(1.4, "align over pin", **{pull: pin(0.179, -0.022)})
        phase(0.95, "straddle pin", **{pull: pin(0.130, -0.022)})
        phase(0.75, "grasp pin", **{pull: pin(0.130, -0.001)})
        phase(1.65, "lift pin clear", **{pull: pin(0.202, -0.001)})
        phase(0.35, "hold above guide")
        phase(0.45, "settle lifted pin", **{pull: pin(0.191, -0.004)})
        phase(1.2, "carry pin to tray", **{pull: (*TRAY[:2], 0.191, -0.004)})
        phase(1.2, "lower pin to tray", **{pull: (*TRAY[:2], 0.097, -0.004)})
        phase(0.4, "settle pin on tray")
        phase(0.7, "release pin", **{pull: (*TRAY[:2], 0.097, -0.026)})
        phase(0.85, "lift off tray", **{pull: (*TRAY[:2], 0.175, -0.026)})
        phase(0.95, "clear pin hand", **{pull: home[pull]})
    if mode in ("reference", "alternate", "abandon_release"):
        for deg in (-2, -7, -12, -17, -22, -27, -32, -37, -42, -45):
            phase(0.72, "guide return", **{wheel: handle(deg)})
            if mode == "abandon_release" and deg == -12:
                break
        phase(0.65, "release knob", **{wheel: handle(deg, OPEN)})
        wx, wy, _, _ = handle(deg)
        phase(
            0.9,
            "retreat from wheel",
            **{wheel: (wx - 0.035, wy + sign(wheel) * 0.055, 0.144, OPEN)},
        )
    if mode == "unload_only":
        phase(0.9, "reload pin", **{wheel: handle(0)})
        phase(0.75, "release wheel", **{wheel: handle(0, OPEN)})
        phase(0.9, "clear wheel", **{wheel: handle(0, OPEN, 0.178)})
    phase(1.15, "withdraw", **home)
    phase(1.4, "dwell clear")
    return seq
