# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Seat a robot controller's add-on board (a shield with two rows of 20 header pins) without bending any pin.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475.

- A controller board (green, 85 x 56 x 1.6 mm) is screwed to a 100 x 70 x 6 mm aluminium carrier plate fixed to the
  table at (0.30, 0.0), its long edge along y. A black 2 x 20 female header (5.08 x 50.8 x 8.5 mm) stands in the middle
  of the board, its two rows along y.
- The add-on board (blue, 65 x 56 x 1.6 mm, 30 g with a 25 x 25 x 12 mm aluminium heat sink on top) carries two rows
  of 20 pins (0.64 mm square, 2.54 mm pitch, rows 5.08 mm apart). It hangs 9.4 mm above the header, resting
  cocked 7 deg lengthwise and 3 deg across, as if set down on the pin tips. The pins enter the holes only when
  the board is within 1.5 deg lengthwise and 4 deg across; pressed in while cocked past that, the leading pins bend.
  Mating force is 10 N (0.25 N per pin).

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
START_MM = 10.0  # hover height above the seat
ROLL0, PITCH0 = 7.0, 3.0  # initial cock, deg (roll about x = lengthwise tilt, pitch about y = across the rows)
ENGAGE_M = 0.004  # pins reach the holes below this height above the seat
ROLL_TOL, PITCH_TOL = 1.5, 4.0
SEAT_TOL_M = 0.0003
LEVEL_ROLL, LEVEL_PITCH = 1.0, 2.0
GRIP_M = 0.0025  # a pad this close to the board or heat sink counts as holding it
PRESS_N = 10.0
STILL_MM_S, STILL_DEG_S = 3.0, 3.0
TCP_STILL, CLEAR_M, STILL_S, RELEASE_S = 0.005, 0.030, 1.0, 1.0
EPS = 1e-6

GREEN, BLACK, BLUE, ALU = (0.10, 0.50, 0.25), (0.05, 0.05, 0.05), (0.15, 0.30, 0.80), (0.75, 0.76, 0.78)

t = lk.Tree("shield", pos=(BASE[0], BASE[1], TOP), fixed=True, color=(0.6, 0.6, 0.62))
t.link("ctrl", [lk.box((0.070, 0.100, 0.006), (0, 0, 0.003)),
                lk.box((0.056, 0.085, BOARD_T), (0, 0, 0.006 + BOARD_T / 2), color=GREEN),
                lk.box((0.00508, 0.0508, 0.0085), (0, 0, 0.006 + BOARD_T + 0.00425), color=BLACK)], mass=0.4,
       contact_offset=0.001)
CZ = ZSEAT + BOARD_T / 2  # board mid-plane when seated
t.link("carriage", [lk.box((0.004, 0.004, 0.002), (0, 0.048, 0))], mass=0.02, at=(0, 0, CZ), color=BLUE)
t.link("gimbal", [lk.box((0.004, 0.004, 0.002), (0, -0.048, 0))], mass=0.02, at=(0, 0, CZ), color=BLUE)
t.link("board", [lk.box((0.056, 0.065, BOARD_T), color=BLUE),
                 lk.box((0.025, 0.025, 0.012), (0, 0, BOARD_T / 2 + 0.006), color=ALU)], mass=0.030, at=(0, 0, CZ),
       color=BLUE, contact_offset=0.001)
t.prismatic("seat", "ctrl", "carriage", anchor=(0, 0, CZ), axis=(0, 0, 1), limits=(0.0, START_MM / 1000),
            init=START_MM / 1000, damping=5.0,
            brake=lk.Brake(stiffness=PRESS_N / (START_MM / 1000), damping=15.0, engaged=True))
t.revolute("roll", "carriage", "gimbal", anchor=(0, 0, CZ), axis=(1, 0, 0), limits_deg=(-20, 20), init_deg=ROLL0,
           damping=0.01, armature=5e-4, brake=lk.Brake(stiffness=2.0, damping=0.1, engaged=True))
t.revolute("pitch", "gimbal", "board", anchor=(0, 0, CZ), axis=(0, 1, 0), limits_deg=(-20, 20), init_deg=PITCH0,
           damping=0.01, armature=5e-4, brake=lk.Brake(stiffness=2.0, damping=0.1, engaged=True))
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
        held = m["held"]
        H.ever("levelled_held", held and m["z"] >= ENGAGE_M and abs(m["roll"]) <= LEVEL_ROLL
               and abs(m["pitch"]) <= LEVEL_PITCH)
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
    if "levelled_held" in H.evers:
        return 0.25, "the board was held and levelled above the pins, never pressed home"
    return 0.0, "the board was never held and levelled above the pins"


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
         "clear": {}, "tcp_speed": {}, "held": False}
    if _PREV:
        dt = max(tick.t - _PREV["t"], 1e-6)
        m["speed"] = float(np.linalg.norm(b[:3, 3] - _PREV["b"][:3, 3])) / dt
        m["spin"] = (abs(m["roll"] - _PREV["roll"]) + abs(m["pitch"] - _PREV["pitch"])) / dt
    _PREV.update(t=tick.t, b=np.array(b, dtype=float), roll=m["roll"], pitch=m["pitch"])
    for side, tool in tick.tools.items():
        m["clear"][side] = lk.clearance(tool, s)
        m["tcp_speed"][side] = tool.speed
        # gripped = a pad on the board with the fingers apart (closed fingertips pressing on top do not grip it)
        m["held"] = m["held"] or (m["clear"][side] <= GRIP_M and tool.opening > 0.010)
    REC.update(m)
    if not _PREV.get("guided"):
        if m["held"] and not _PREV.get("held"):  # a gripper has the board: its tilt is free
            _PREV["held"] = True
            lk.brake("shield", "roll", False)
            lk.brake("shield", "pitch", False)
        elif _PREV.get("held") and not m["held"]:  # let go above the pins: the tips prop it as left
            _PREV["held"] = False
            lk.brake("shield", "roll", True)
            lk.brake("shield", "pitch", True)
    if m["z"] < ENGAGE_M and not _PREV.get("guided"):  # the pins in the holes pull the board square
        _PREV["guided"] = True
        lk.brake("shield", "roll", False)
        lk.brake("shield", "pitch", False)
        lim = math.radians(0.5)
        lk.set_limits("shield", "roll", -lim, lim)
        lk.set_limits("shield", "pitch", -lim, lim)
    if m["z"] < ENGAGE_M and m["z"] < _PREV.get("latch", 1.0) - 1e-4:  # the pins grip: what is pushed in stays in
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
VARIANTS = ("", "nothing", "cocked_press", "cocked_top_press")
LOG: dict = {}
SINK_C = (0.0, 0.0, BOARD_T / 2 + 0.006)  # heat sink centre, board frame
D = 0.0115
SQUEEZE = float(os.environ.get("SQUEEZE", 0.003))


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
    arm = ctx.right
    if VARIANT == "cocked_top_press":  # no levelling: closed fingertips straight down on the cocked board
        o = ctx.observe()
        top = o.point("shield", "board", (0.0, 0.0, BOARD_T / 2 + 0.012))
        yield from arm.press(top, 0.03, hover=0.05, duration=3.0, hold=1.0)
        yield from arm.move(arm.pos + np.array([0.0, 0.0, 0.06]), duration=1.0)
        yield from ctx.wait(1.5)
        return
    seen = ctx.observe()
    Rb = seen.pose("shield", "board")[:3, :3]
    C = seen.point("shield", "board", (0, 0, 0))
    P = seen.point("shield", "board", SINK_C)
    r = grip_frame(-Rb[:, 2], Rb[:, 0])  # approach down the board normal, pads across the heat sink
    tcp = P - r @ np.array([0.0, 0.0, D])
    yield from arm.move(tcp - r[:, 2] * 0.06, r, width=0.06, duration=1.5)
    yield from arm.move(tcp, r, duration=1.5)
    yield from arm.grip(0.025 - SQUEEZE, 0.8)
    yield from arm.wait(0.4)
    st = {"r": r, "tcp": tcp}

    def level():
        for _ in range(3):
            Rb = ctx.observe().pose("shield", "board")[:3, :3]
            dR = Rb.T  # rotation that takes the seen board to level (world frame, about the board centre)
            st["r"] = dR @ st["r"]
            st["tcp"] = C + dR @ (st["tcp"] - C)
            yield from arm.move(st["tcp"], st["r"], duration=1.5)
        o = ctx.observe()
        LOG["seen_roll_pitch"] = (round(o.angle_deg("shield", "roll"), 2), round(o.angle_deg("shield", "pitch"), 2))

    def descend(mm, s=2.5):
        st["tcp"] = st["tcp"] + np.array([0.0, 0.0, -mm / 1000])
        yield from arm.move(st["tcp"], st["r"], duration=s)

    def top_press(mm):
        o = ctx.observe()
        top = o.point("shield", "board", (0.0, 0.0, BOARD_T / 2 + 0.012))
        yield from ctx.right.press(top, mm / 1000, hover=0.05, duration=3.0, hold=1.0)

    if VARIANT == "cocked_press":
        yield from descend(12.5)
        yield from arm.wait(0.6)
    else:
        yield from level()
        yield from arm.open(0.6)
        yield from arm.move(arm.pos + np.array([0.0, 0.0, 0.05]), duration=1.0)
        LOG["seen_roll_pitch_released"] = (round(ctx.observe().angle_deg("shield", "roll"), 2),
                                           round(ctx.observe().angle_deg("shield", "pitch"), 2))
        yield from top_press(float(os.environ.get("PRESS_MM", 30.0)))
    LOG["seen_seat_mm"] = round(1000 * (ctx.observe().point("shield", "board", (0, 0, 0))[2] - (TOP + CZ)), 2)
    yield from arm.open(0.8)
    yield from arm.move(arm.pos + np.array([0.0, 0.0, 0.08]), duration=1.2)
    yield from ctx.wait(2.5)
