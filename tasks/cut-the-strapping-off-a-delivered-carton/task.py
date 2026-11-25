"""On the station table: a delivered carton (12 x 9 x 3 in) stands on two boards, bound by two black polypropylene
straps round its girth. A concealed-blade safety cutter lies flat on the table to the right. The robot must cut both
straps with the cutter, holding each strap down beside the cut, without marking the carton, and put the cutter down.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from strapkit import StrapRig, StrapState, on_reset  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = ("Cut both straps on this carton with the safety cutter, holding each strap down while you cut it. "
               "Do not mark the carton. Then put the cutter down on the table.")

KRAFT, PINE, BLACK, ORANGE, GREY, STEEL = ((0.72, 0.58, 0.40), (0.86, 0.72, 0.50), (0.08, 0.08, 0.09),
                                           (0.95, 0.45, 0.08), (0.30, 0.31, 0.33), (0.78, 0.80, 0.83))
LAYOUT = os.environ.get("STRAP_LOT", "A").upper()
DX = {"A": 0.0, "B": -0.012}[LAYOUT]            # layout B: carton, boards and straps 12 mm nearer the robot

# -- the carton on its boards ------------------------------------------------------------------------------------------
BOARD_T, BOARD_W, BOARD_L = 0.019, 0.089, 0.30   # a 1x4 board: 19 x 89 mm
CARTON = (0.3048, 0.2286, 0.0762)                # 12 x 9 x 3 in
XC = 0.37 + DX
CARTON_C = (XC, 0.0, TOP + BOARD_T + CARTON[2] / 2)
ZT = TOP + BOARD_T + CARTON[2]                   # the carton's top
BOARD_DX = 0.125                                 # the boards' middle lines either side of the carton's middle
BOARDS = (((XC - BOARD_DX, 0.0, TOP + BOARD_T / 2), (BOARD_W, BOARD_L, BOARD_T), PINE),
          ((XC + BOARD_DX, 0.0, TOP + BOARD_T / 2), (BOARD_W, BOARD_L, BOARD_T), PINE))
STRAP_X = (XC - 0.065, XC + 0.065)               # the two straps' middle lines
STRAP_W, STRAP_T = 0.0127, 0.001
TENSION, SPRING_K = float(os.environ.get("STRAP_TENSION_N", 50.0)), 5000.0

# -- the cutter (tool frame: origin at the toe's tip on the sole side, mid thickness; x to the head, z sole to spine) ---
T_THICK = 0.013
TOOL_BOXES = (((-0.035, 0.0, 0.041), (0.140, T_THICK, 0.022), ORANGE),      # handle
              ((0.027, 0.0, 0.0181), (0.046, T_THICK, 0.0238), ORANGE),     # upper jaw: its underside is 6.2 mm up
              ((0.045, 0.0, 0.0042), (0.010, T_THICK, 0.0040), GREY),       # the channel's closed end
              ((0.036, 0.0, 0.0011), (0.028, T_THICK, 0.0022), GREY))       # sole behind the toe
TOE = tuple((x, y, z) for x, z in ((0.0, 0.0), (0.0, 0.0003), (0.022, 0.0), (0.022, 0.0022)) for y in (-0.0065, 0.0065))
BLADE_C, BLADE = (0.031, 0.0, 0.0042), (0.002, 0.011, 0.0036)
GRIP_T = np.array([0.027, 0.0, 0.046])            # the tool point on the handle, over the blade, tool frame
# -- where the cutter waits: in the 15 mm gap between two 1x4 offcuts lying flat, toe tip and handle end on the table --
def rot_y(a: float) -> np.ndarray:
    """Rotation about env y; positive tips the +x end down."""
    c, sn = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, sn], [0.0, 1.0, 0.0], [-sn, 0.0, c]])


BETA = math.atan2(0.030, 0.105)                  # resting on its toe tip and its handle's end, it sits nose up 16 deg
HOLD_R = rot_y(-BETA)
TOOL0 = (0.30, -0.26, TOP + 0.0005)              # where its origin (the toe's tip) rests at the start
GUIDE = (0.19, BOARD_W, BOARD_T)                 # a 19 cm offcut of the same 1x4 board, lying flat
GAP = 0.015
GUIDES = tuple(((TOOL0[0] - 0.03, TOOL0[1] + sgn * (GAP + GUIDE[1]) / 2, TOP + GUIDE[2] / 2), GUIDE, PINE) for sgn in (-1, 1))
CUT_N, CUT_HOLD = 5.0, 24
DIG_N = 15.0         # the cutter pressed on the carton this hard for 0.1 s: the carton is marked

RIG = StrapRig(name="strapped", pos=CARTON_C, size=CARTON, color=KRAFT, carton_mass=2.5, carton_color=KRAFT,
               fixed_boxes=BOARDS + GUIDES, strap_x=STRAP_X, strap_w=STRAP_W, strap_t=STRAP_T,
               n_top=int(os.environ.get("STRAP_NTOP", 9)), n_side=int(os.environ.get("STRAP_NSIDE", 3)),
               link_mass=float(os.environ.get("STRAP_LINK_G", 4)) / 1000, link_inertia=float(os.environ.get("STRAP_LINK_I", 2e-6)), strap_colors=(BLACK,), spring_k=SPRING_K, tension=TENSION,
               slack=float(os.environ.get("STRAP_SLACK_MM", 5)) / 1000,
               tool_pos=TOOL0, tool_R=tuple(map(tuple, HOLD_R)), tool_boxes=TOOL_BOXES, toe_points=TOE, toe_color=GREY,
               tool_mass=0.046, blade_centre=BLADE_C, blade_size=BLADE, blade_mass=0.002, blade_color=STEEL,
               cut_n=CUT_N, cut_hold=CUT_HOLD, dig_n=DIG_N, dig_hold=CUT_HOLD)
OBJECTS = [RIG]

# -- scoring --------------------------------------------------------------------------------------------------------
HOLD_N = 1.0         # N: the other gripper pressed on the strap at least this hard for the 0.1 s of the cut ...
HOLD_M = 0.12        # ... within this distance of the cut
SLID_M = 0.03        # the carton ended this far from where it started: dragged
LIFT_M = 0.05        # the cutter picked up: 5 cm above where it lay
CLEAR_M = 0.03       # both grippers this far from the carton and the cutter
DOWN_Z = TOP + 0.03  # the cutter put down: its origin (the toe's tip) below this
REST_V = 0.02

_STATE: dict = {}


def _reset_state(environment=None) -> None:
    _STATE.update(lifted=False)


_reset_state()
on_reset(_reset_state)


def box_gap(p, centre, R, size) -> float:
    """Distance from point ``p`` to a box (0 inside it)."""
    q = np.asarray(R, float).T @ (np.asarray(p, float) - np.asarray(centre, float))
    return float(np.linalg.norm(np.maximum(np.abs(q) - np.asarray(size, float) / 2, 0.0)))


def tool_gap(p, tool_pos, tool_R) -> float:
    """Distance from ``p`` to the cutter (its handle and head as boxes)."""
    return min(box_gap(p, np.asarray(tool_pos) + np.asarray(tool_R) @ np.asarray(c), tool_R, s) for c, s, _ in TOOL_BOXES)


def verdict(c: StrapState) -> dict:
    """Everything the score depends on, by name."""
    _STATE["lifted"] |= bool(c.tool_pos[2] >= TOOL0[2] + LIFT_M)
    straps = sorted({cut.strap for cut in c.cuts})
    watched = all(cut.watched for cut in c.cuts)
    unheld = [cut for cut in c.cuts if cut.held_n < HOLD_N or cut.held_m > HOLD_M]
    slid = float(np.linalg.norm(np.asarray(c.carton_pos)[:2] - np.asarray(CARTON_C)[:2]))
    tipped = float(np.asarray(c.carton_R)[2, 2]) < math.cos(math.radians(10))
    gaps = [min(box_gap(p, c.carton_pos, c.carton_R, CARTON), tool_gap(p, c.tool_pos, c.tool_R)) for p in c.tcp.values()]
    down = bool(c.tool_pos[2] <= DOWN_Z and float(np.linalg.norm(np.asarray(c.tool_vel)[:3])) < REST_V)
    still = bool(float(np.linalg.norm(np.asarray(c.carton_vel)[:3])) < REST_V)
    return dict(lifted=_STATE["lifted"], straps=straps, watched=watched, unheld=len(unheld), marked=bool(c.dug),
                slid=slid, tipped=tipped, clear=min(gaps) >= CLEAR_M, down=down, still=still)


def _score(v: dict) -> float:
    if v["unheld"] or v["marked"] or v["slid"] > SLID_M or v["tipped"]:
        return 0.0
    if len(v["straps"]) == 2:
        return 1.0 if v["down"] and v["clear"] and v["still"] else 0.75
    if len(v["straps"]) == 1:
        return 0.5
    return 0.25 if v["lifted"] else 0.0


def score(objects) -> float:
    """0 if a strap was cut while the other gripper was not pressing on it (at least 1 N for the 0.1 s of the cut,
    within 12 cm of the cut), if the cutter pressed on the carton with 15 N or more for 0.1 s, or if the carton ends
    more than 3 cm from its start or tipped. Otherwise 1.0 both straps cut, the cutter put down on the table and at rest, both grippers 3 cm
    clear of carton and cutter, the carton at rest; 0.75 both straps cut; 0.5 one strap cut; 0.25 the cutter was lifted
    5 cm; else 0."""
    return _score(verdict(objects["strapped"]))


def success(objects) -> bool:
    """Full score, held for 1 s by the runner."""
    return score(objects) == 1.0


# -- scripted demo (pure NumPy plan; demo() wraps it in yam_arena Waypoints) -----------------------------------------
OPEN, SHUT = -0.04695, 0.0           # yam_arena.constants.FINGER_OPEN / FINGER_CLOSED
HOLD_GAP = 0.0                       # fingers sent shut on the 13 mm handle: 5 N/mm x 6.5 mm = 32.5 N a finger
GRIP = -HOLD_GAP / 2
TIP = 0.0127                         # the closed fingertips' end beyond the tool point
Y_CUT, Y_HOLD = -0.02, 0.07          # where the cutter crosses the strap, and where the other hand presses it
PRESS_M = float(os.environ.get("DEMO_PRESS_MM", 6)) / 1000     # the holding fingertips are sent this far into the strap
SOLE_M = float(os.environ.get("DEMO_SOLE_MM", 2)) / 1000       # the sole is sent this far into the carton's top
START_DX, END_DX = 0.020, float(os.environ.get("DEMO_END_CM", -7.5)) / 100   # toe tip from 2 cm beyond the strap ...
PULL_S = float(os.environ.get("DEMO_PULL_S", 3.5))


def frame(z, x) -> np.ndarray:
    """Gripper rotation with approach axis ``z`` and finger-closing axis ``x`` (world)."""
    z = np.asarray(z, float) / np.linalg.norm(z)
    x = np.asarray(x, float) - z * np.dot(x, z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


DOWN_Y = frame((0, 0, -1), (0, -1, 0))                                  # straight down, fingers closing along y
USE_LEAN = math.radians(30)                                             # in use the hand leans forward 30 degrees
USE_R = np.eye(3)                                                       # the cutter in use: sole flat, head away
USE_HAND = rot_y(-USE_LEAN) @ DOWN_Y
PICK_HAND = rot_y(-(USE_LEAN + BETA)) @ DOWN_Y                          # ... and 46 degrees to take it nose up as it waits
PRESS_LEAN = math.radians(35)
PRESS = frame((0, -math.sin(PRESS_LEAN), -math.cos(PRESS_LEAN)), (-1, 0, 0))   # left hand, leaning out to its own side
PICK = np.asarray(TOOL0) + HOLD_R @ GRIP_T                              # tool point on the handle as it waits


def tool_tcp(origin) -> tuple:
    """Tool point that puts the cutter's origin (toe tip) at ``origin`` with its sole flat."""
    return tuple(np.asarray(origin, float) + USE_R @ GRIP_T)


def _right(order=(1, 0), wait=(0.0, 0.0), keep: bool = False):
    """(duration, pos, rot, finger, label). Pick the cutter up, cut the far strap then the near one, put it down."""
    px, py, pz = PICK
    back = -PICK_HAND[:, 2]                      # away from the cutter, along the hand
    over = tuple(PICK + back * 0.09)
    w = [(2.5, over, PICK_HAND, OPEN, "over the cutter"),
         (1.5, (px, py, pz), PICK_HAND, OPEN, "astride the handle"), (0.8, (px, py, pz), PICK_HAND, GRIP, "grip"),
         (1.2, (px, py, pz + 0.10), PICK_HAND, GRIP, "lift")]
    low = ZT - SOLE_M
    for n, s in enumerate(order):
        x = STRAP_X[s]
        w += [(3.0 if n == 0 else 1.5, tool_tcp((x + START_DX, Y_CUT, ZT + 0.025)), USE_HAND, GRIP, f"beyond strap {s}, sole level"),
              (1.5, tool_tcp((x + START_DX, Y_CUT, low)), USE_HAND, GRIP, "sole on the carton"),
              (wait[n], tool_tcp((x + START_DX, Y_CUT, low)), USE_HAND, GRIP, "wait for the holding hand"),
              (PULL_S, tool_tcp((x + END_DX, Y_CUT, low)), USE_HAND, GRIP, "draw the toe under the strap and through"),
              (0.4, tool_tcp((x + END_DX, Y_CUT, low)), USE_HAND, GRIP, "hold"),
              (0.8, tool_tcp((x + END_DX, Y_CUT, ZT + 0.03)), USE_HAND, GRIP, "lift clear")]
    if keep:                                     # the control that never puts the cutter down
        return w + [(4.0, w[-1][1], USE_HAND, GRIP, "keep hold of the cutter")]
    w += [(3.0, (px, py, pz + 0.10), PICK_HAND, GRIP, "back over its place"),
          (1.5, (px, py, pz + 0.004), PICK_HAND, GRIP, "set it down between the guides"),
          (0.6, (px, py, pz + 0.004), PICK_HAND, OPEN, "let go"), (1.2, over, PICK_HAND, OPEN, "clear")]
    return w


def _press_point(s: int, depth: float) -> tuple:
    tip = np.array([STRAP_X[s], Y_HOLD, ZT + STRAP_T - depth])
    return tuple(tip - PRESS[:, 2] * TIP)


def plan(hold: bool = True, order=(1, 0), stop_after: int = 2, press_m: float | None = None, keep: bool = False) -> dict[str, list]:
    """Both arms' waypoints. ``hold`` False leaves the left arm still (the control with nobody holding the strap);
    ``stop_after`` 1 cuts only the first strap."""
    order = tuple(order)[:stop_after]
    press = PRESS_M if press_m is None else press_m
    base = _right(order, (0.0,) * len(order), keep)
    # when does the right hand have its sole down at each strap, and when is each cut over?
    t, ready, done = 0.0, [], []
    for d, _, _, _, label in base:
        if label == "wait for the holding hand":
            ready.append(t)
        t += d
        if label == "lift clear":
            done.append(t)
    if not hold or not order:
        return {"right": base}
    left, tl, waits = [], 0.0, []
    for n, s in enumerate(order):
        up, on = _press_point(s, -0.05), _press_point(s, press)
        seg = [(3.0 if n == 0 else 1.5, up, PRESS, SHUT, f"over strap {s}"), (1.2, on, PRESS, SHUT, "press the strap down")]
        arrive = tl + sum(x[0] for x in seg)
        shift = sum(waits)
        wait = max(0.3, arrive + 0.3 - (ready[n] + shift))
        waits.append(wait)
        cut_over = done[n] + shift + wait
        seg += [(cut_over - arrive, on, PRESS, SHUT, "hold it while it is cut"), (0.8, up, PRESS, SHUT, "let go")]
        left += seg
        tl = arrive + (cut_over - arrive) + 0.8
    right = _right(order, tuple(waits), keep)
    left.append((sum(x[0] for x in right) - tl if sum(x[0] for x in right) > tl else 0.5,
                 (0.25, 0.25, ZT + 0.10), PRESS, SHUT, "back"))
    return {"left": left, "right": right}


def demo():
    """The right arm takes the cutter by its handle from above, levels its sole on the carton and draws its toe under
    each strap in turn, far strap first; the left arm presses each strap down beside the cut while it is cut. DEMO_NO_HOLD=1 leaves
    the left arm still; DEMO_STRAPS=1 stops after one strap and DEMO_STRAPS=0 only picks the cutter up and puts it
    back; DEMO_KEEP=1 never puts the cutter down; DEMO_SOLE_MM sets how hard the sole is pressed on the carton;
    DEMO_STILL=1 moves nothing."""
    from yam_arena.demo import Waypoint

    if os.environ.get("DEMO_STILL"):
        return {}
    out = plan(hold=not os.environ.get("DEMO_NO_HOLD"), stop_after=int(os.environ.get("DEMO_STRAPS", 2)),
               keep=bool(os.environ.get("DEMO_KEEP")))
    return {arm: [Waypoint(d, tuple(p), f, label, r) for d, p, r, f, label in wps] for arm, wps in out.items()}
