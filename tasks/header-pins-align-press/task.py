# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Seat a robot controller's add-on board (a shield with two rows of 20 header pins) without bending any pin.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475.

- A controller board (green, 85 x 56 x 1.6 mm) is screwed to a 100 x 70 x 6 mm aluminium carrier plate fixed to the
  table at (0.30, 0.0), its long edge along y. A black 2 x 20 female header (7.62 x 50.8 x 8.5 mm: the two pin rows,
  5.08 mm apart, plus 1.27 mm of plastic each side) stands in the middle of the board, its two rows along y.
- The add-on board (blue, 65 x 56 x 1.6 mm, 30 g with a 25 x 25 x 12 mm aluminium heat sink on top) carries two rows
  of 20 pins (0.64 mm square, 2.54 mm pitch, rows 5.08 mm apart), 6 mm long. It starts level, resting on the pin tips
  on top of the header (underside 6.0 mm above the seat). Pressed through its centre it goes home evenly; pressed
  off-centre the pins on the loaded side enter first, the board rocks, and past 1.5 deg lengthwise or 4 deg across as
  the pins enter the leading pins bend. Mating force is 10 N (0.25 N per pin).

Goal: the add-on board fully seated on the header, no pin bent, both grippers let go.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import linkkit as lk  # noqa: E402
import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
VARIANT = os.environ.get("DEMO_VARIANT", "")

BASE = (0.30, 0.0)
ZSEAT = 0.0161  # underside of the add-on board when seated, above the table top
BOARD_T = 0.0016
PIN_M = 0.006  # pin length: the board rests on the pin tips with its underside this high above the seat
ENGAGE_M = 0.0045  # the pins are 1.5 mm into the sockets below this height above the seat
ROLL_TOL, PITCH_TOL = 1.5, 4.0
SEAT_TOL_M = 0.0003
TILT_HOLD = 6.0  # N m/rad: the pins in the sockets resist rocking
TILT_K = 0.02  # N m/rad: weight restoring the board flat while it rests on the pin tips
PRESS_N = 3.3
GRIP_STEP = 0.001  # m: the pins grip with a 3.3 N force (0.08 N per pin): the board stays put until pushed this far past its latch
GRIP_K = PRESS_N / GRIP_STEP  # N/m
STILL_MM_S, STILL_DEG_S = 3.0, 3.0
TCP_STILL, CLEAR_M, STILL_S, RELEASE_S = 0.005, 0.030, 1.0, 1.0
EPS = 1e-6

GREEN, BLACK, BLUE, ALU = (0.10, 0.50, 0.25), (0.05, 0.05, 0.05), (0.15, 0.30, 0.80), (0.75, 0.76, 0.78)

t = lk.Tree("shield", pos=(BASE[0], BASE[1], TOP), fixed=True, color=(0.6, 0.6, 0.62))
t.link("ctrl", [lk.box((0.070, 0.100, 0.006), (0, 0, 0.003)),
                lk.box((0.056, 0.085, BOARD_T), (0, 0, 0.006 + BOARD_T / 2), color=GREEN),
                lk.box((0.00762, 0.0508, 0.0085), (0, 0, 0.006 + BOARD_T + 0.00425), color=BLACK)], mass=0.4,
       contact_offset=0.001)
CZ = ZSEAT + BOARD_T / 2  # board mid-plane when seated
t.link("carriage", [lk.box((0.004, 0.004, 0.002), (-0.048, 0, 0))], mass=0.02, at=(0, 0, CZ), color=BLUE)
t.link("gimbal", [lk.box((0.004, 0.004, 0.002), (0.048, 0, 0))], mass=0.02, at=(0, 0, CZ), color=BLUE)
t.link("board", [lk.box((0.056, 0.065, BOARD_T), color=BLUE),
                 lk.box((0.025, 0.025, 0.012), (0, 0, BOARD_T / 2 + 0.006), color=ALU)], mass=0.030, at=(0, 0, CZ),
       color=BLUE, contact_offset=0.001)
# Supports (all contact physics, nothing holds the board in the air): above the header the slide and both hinges are
# free and the board falls; on the pin tips the weight restores it flat; the pins' grip (3.3 N: it stays where it was
# pushed unless pushed or pulled by more than that, the weight is 0.3 N) holds it on the tips and in the sockets, and with the pins 1.5 mm in they also
# resist rocking (6 N m/rad: 1.5 deg at 0.16 N m, i.e. a 10 N press 16 mm off centre).
t.prismatic("seat", "ctrl", "carriage", anchor=(0, 0, CZ), axis=(0, 0, 1), limits=(0.0, 0.060),
            init=PIN_M, damping=60.0, brake=lk.Brake(stiffness=GRIP_K, damping=20.0, engaged=True))
t.revolute("roll", "carriage", "gimbal", anchor=(0, 0, CZ), axis=(1, 0, 0), limits_deg=(-20, 20), init_deg=0.0,
           damping=0.01, armature=5e-4, stiffness=TILT_K, brake=lk.Brake(stiffness=TILT_HOLD, damping=0.3))
t.revolute("pitch", "gimbal", "board", anchor=(0, 0, CZ), axis=(0, 1, 0), limits_deg=(-20, 20), init_deg=0.0,
           damping=0.01, armature=5e-4, stiffness=TILT_K, brake=lk.Brake(stiffness=TILT_HOLD, damping=0.3))
SHIELD = t.build()
OBJECTS = [SHIELD]


def bent(m) -> bool:
    return m["z"] < ENGAGE_M and (abs(m["roll"]) > ROLL_TOL or abs(m["pitch"]) > PITCH_TOL)


def seated(m) -> bool:
    return m["z"] <= SEAT_TOL_M


class Record:
    """Whole-run record (a linkkit History, cleared only by the simulator reset) plus the latest sample."""

    def __init__(self):
        self.H = lk.History()
        self.last: dict | None = None
        self.n = 0
        self.trace: list = []

    def reset(self) -> None:
        self.H.reset()
        self.last, self.n, self.trace = None, 0, []

    def update(self, m: dict, t: float | None = None) -> None:
        H = self.H
        if t is not None:
            H._tick(t)
        self.n += 1
        self.last = m
        H.ever("bent", bent(m))
        H.ever("pressed_in", m["z"] <= 0.002)
        still = m["speed"] < STILL_MM_S / 1000 and m["spin"] < STILL_DEG_S
        H.held("steady", seated(m) and still)
        H.held("released", bool(m["clear"]) and min(m["clear"].values()) >= CLEAR_M
               and max(m["tcp_speed"].values()) < TCP_STILL)


def ladder(rec: Record) -> tuple[float, str]:
    H, m = rec.H, rec.last
    if m is None:
        return 0.0, "no steps yet"
    if "bent" in H.evers:
        return 0.0, (f"gate: pins bent at {H.evers['bent']:.2f} s (board entered the holes cocked: "
                     f"roll {m['roll']:.1f} deg, pitch {m['pitch']:.1f} deg at the end)")
    if seated(m):
        ends = {"board still 1 s": H.runs.get("steady", 0.0) >= STILL_S - EPS,
                "both grippers 30 mm clear and still 1 s": H.runs.get("released", 0.0) >= RELEASE_S - EPS}
        if all(ends.values()):
            return 1.0, "board seated, no pin bent, both grippers let go"
        return 0.75, "seated at the end; not yet: " + ", ".join(k for k, ok in ends.items() if not ok)
    if "pressed_in" in H.evers:
        return 0.5, "pressed within 2 mm of the seat, unbent, but not seated at the end"
    return 0.0, "the board was never pressed into the sockets"


REC = Record()
_PREV: dict = {}


@lk.on_reset
def _reset(tick):
    REC.reset()
    _PREV.clear()


@lk.on_step
def _track(tick):
    s = tick.states["shield"]
    b = s.poses["board"]
    m = {"z": s.q["seat"], "roll": s.deg("roll"), "pitch": s.deg("pitch"), "speed": 0.0, "spin": 0.0,
         "clear": {}, "tcp_speed": {}}
    if _PREV:
        dt = max(tick.t - _PREV["t"], 1e-6)
        m["speed"] = float(np.linalg.norm(b[:3, 3] - _PREV["b"][:3, 3])) / dt
        m["spin"] = (abs(m["roll"] - _PREV["roll"]) + abs(m["pitch"] - _PREV["pitch"])) / dt
    _PREV.update(t=tick.t, b=np.array(b, dtype=float), roll=m["roll"], pitch=m["pitch"])
    for side, tool in tick.tools.items():
        m["clear"][side] = lk.clearance(tool, s)
        m["tcp_speed"][side] = tool.speed
    REC.update(m)
    mode = "free" if m["z"] > PIN_M + 0.0010 else ("pinned" if m["z"] < ENGAGE_M else "tips")
    if mode != _PREV.get("mode"):
        _PREV["mode"] = mode
        if mode == "free":  # clear of the header: nothing holds the board
            for j in ("seat", "roll", "pitch"):
                lk.brake("shield", j, False)
                lk.set_spring("shield", j, stiffness=0.0)
        else:  # on the pin tips (or in the sockets): the pins grip the slide where it is
            _PREV["latch"] = m["z"]
            lk.brake("shield", "seat", True)
            for j in ("roll", "pitch"):
                lk.brake("shield", j, mode == "pinned")
    if mode != "free" and (m["z"] < _PREV["latch"] - GRIP_STEP or (m["z"] < SEAT_TOL_M < _PREV["latch"])):  # pushed in
        _PREV["latch"] = m["z"]
        lk.brake("shield", "seat", False)
        lk.brake("shield", "seat", True)
    if REC.n % 5 == 0 or REC.n < 8:
        REC.trace.append((round(tick.t, 2), round(1000 * m["z"], 2), round(m["roll"], 2), round(m["pitch"], 2),
                          [round(1000 * m["clear"][k], 0) for k in ("left", "right")], round(1000 * tick.tools["right"].opening, 1)))


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():
    return {}


# -- scripted demo (camera-observable poses only) ---------------------------------------------------------------------
VARIANTS = ("", "nothing", "end_press", "hover_release", "left_only", "right_only")
LOG: dict = {}
SINK_C = (0.0, 0.0, BOARD_T / 2 + 0.006)  # heat sink centre, board frame
D = 0.0115
PRESS_MM = float(os.environ.get("PRESS_MM", 45.0))


def demo_report() -> dict:
    return {"record_trace": REC.trace[::2], "ladder": ladder(REC), "log": LOG, "variant": VARIANT,
            "peaks": REC.H.peaks, "evers": REC.H.evers, "runs": REC.H.best_runs}


def demo_script(ctx):
    from linkdemo import grip_frame

    LOG.clear()
    LOG["variant"] = VARIANT
    yield from ctx.wait(1.0)
    if VARIANT == "nothing":
        yield from ctx.wait(8.0)
        return
    arm = ctx.left if VARIANT == "left_only" else ctx.right
    if VARIANT == "hover_release":  # the old loophole: lift the board clear of the header and let go
        seen = ctx.observe()
        Rb = seen.pose("shield", "board")[:3, :3]
        P = seen.point("shield", "board", SINK_C)
        r = grip_frame(-Rb[:, 2], Rb[:, 0])
        tcp = P - r @ np.array([0.0, 0.0, D])
        yield from arm.move(tcp - r[:, 2] * 0.06, r, width=0.06, duration=1.5)
        yield from arm.move(tcp, r, duration=1.5)
        yield from arm.grip(0.025 - 0.003, 0.8)
        yield from arm.wait(0.4)
        yield from arm.move(tcp + np.array([0.0, 0.0, 0.025]), r, duration=1.5)
        yield from arm.open(0.6)
        yield from arm.move(arm.pos + np.array([0.0, 0.0, 0.06]), duration=1.0)
        yield from ctx.wait(3.0)
        return
    local = (0.0, 0.027, BOARD_T / 2) if VARIANT == "end_press" else (0.0, 0.0, BOARD_T / 2 + 0.012)
    o = ctx.observe()
    top = o.point("shield", "board", local)
    yield from arm.press(top, PRESS_MM / 1000, hover=0.05, duration=3.0, hold=1.0)
    LOG["seen_seat_mm"] = round(1000 * (ctx.observe().point("shield", "board", (0, 0, 0))[2] - (TOP + CZ)), 2)
    yield from arm.open(0.8)
    yield from arm.move(arm.pos + np.array([0.0, 0.0, 0.08]), duration=1.2)
    yield from ctx.wait(2.5)
