"""On the station table: a portable inkjet printer with its back toward the robot and a card jammed in the feed slot
of its rear tray. The card stands in the slot with its top 5 cm sticking out, and the printer's two feed rollers, seen
at the slot's mouth, still grip it near both edges. The OK button is on the printer's top.

Task (instruction): clear the paper jam and resume printing. The instruction does not say how. The procedure the task
checks is the printer maker's (Canon GX5000 series online manual, "What to Do When Paper Is Jammed", "Removing Jammed
Paper through Feed Slot of Rear Tray"): "Hold jammed paper firmly with both hands and pull it out slowly." "Slowly pull
out the paper so as not to tear it." Then "Press printer's OK button. The printer resumes printing." With the rollers
gripping both edges, a card pulled up by one edge, or by its middle, skews in the slot and tears beside a roller;
pulled up by both edges at once it comes out whole.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- printer: fixed, x 0.22 to 0.40 (its back, at x 0.22, toward the robot), y -0.15 to 0.15, 7 cm tall (the size of a
  portable inkjet printer). The rear tray's feed slot: x 0.240 to 0.246, y -0.106 to 0.106, 5 cm deep (its floor 2 cm
  above the table).
- card: a DL card (210 x 99 mm) fed landscape, printed, 3 mm thick here (thicker and stiffer than real 0.4 mm card
  stock), standing on the slot's floor against its far wall, its top 11.9 cm above the table. It is three strips joined
  along two seams, at y -0.045 and 0.045. A seam lets its two strips slide past each other only up and down, on a
  spring of 1 N per mm: sheared 1.5 mm, the card has torn.
- feed rollers: light grey, one over each edge strip (y 0.05 to 0.10 and -0.10 to -0.05) at the slot's mouth (their
  tops 3 mm above the printer's top), each on a spring arm that presses the card against the slot's far wall with
  about 4 N.
- OK button: on the printer's top at x 0.31, y -0.11, 1.6 cm across, standing 5 mm proud; pushed down 3 mm it is
  pressed.

Score: 0 for the rest of the run once the card has torn (a seam sheared 1.5 mm or more). Otherwise the best stage
reached: 0.25 the card raised 1.5 cm or more; 0.5 out of the printer (its bottom edge above the printer's top); 0.75
put down on the table in front of the printer (its lowest point on the table, every strip's middle in front of the
printer's back, at rest, no gripper holding it). Full score: that, with the OK button pressed after the card came out,
both grippers 6 cm or more from the card and from the button, still for 1 s.
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, Joint, allow_matrix_grips, as_numpy, on_reset, on_step  # noqa: E402
from bodykit import rotation  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
# The file keeps the best stage reached itself and a broken rule latches 0, so the score is read at the end of the run.
SCORE_OVER = "end"

# -- layout (world frame, heights above the table) --------------------------------------------------------------------
PR_X0, PR_X1, PR_HW, PR_H = 0.22, 0.40, 0.15, 0.07  # printer: back (toward the robot), front, half width, height
SLOT_X0, SLOT_X1, SLOT_HW, SLOT_FLOOR = 0.240, 0.246, 0.106, 0.02  # feed slot: near wall, far wall, half length, floor
# The jammed sheet is a DL card (210 x 99 mm) fed landscape, 3 mm thick here (real 300 g/m2 card is ~0.4 mm)
SH_T, SH_HW, SH_H = 0.003, 0.105, 0.099  # sheet: thickness, half width (y), height
SEAM_Y = 0.045  # the seams between the middle strip and the two edge strips, at y -SEAM_Y and +SEAM_Y
# Each seam lets its two strips slide past each other up and down only, on a stiff spring (1 mm per N); everything
# else across the seam is rigid. Sheared 1.5 mm (1.5 N) the sheet has torn (the score's test); past 4 mm, or under
# 6 N across it, the seam comes apart.
SEAM_K, SEAM_C, SEAM_TRAVEL = 1000.0, 3.0, 0.004
SEAM_N = float(os.environ.get("PAPER_SEAM_N", "6.0"))
SH_X = SLOT_X1 - SH_T / 2  # the sheet's middle plane: against the slot's far wall
SH_KG = 0.012  # per strip
ROLL_R, ROLL_Z, ROLL_LEN = 0.005, 0.068, 0.05  # feed rollers at the slot's mouth: radius, axis height, length (y)
ROLL_Y = (SEAM_Y + SH_HW) / 2  # each roller over the middle of an edge strip
ROLL_N = float(os.environ.get("PAPER_ROLL_N", "4.0"))  # each roller presses the sheet this hard (N)
PIVOT_X, PIVOT_Z = SLOT_X1 - SH_T - ROLL_R, 0.042  # the roller arms' pivots, below the rollers, in the back wall
ARM_L = ROLL_Z - PIVOT_Z
ROLL_DEG = 6.0  # the arm's spring aims this far past the sheet
ROLL_K = ROLL_N * ARM_L / math.radians(ROLL_DEG)  # N m/rad
OK_X, OK_Y, OK_R, OK_UP, OK_TALL = 0.31, -0.11, 0.008, 0.005, 0.012  # OK button: middle, radius, proud, height
OK_K, OK_PRESS = 300.0, 0.003

STRIPS = ("sheet_left", "sheet_middle", "sheet_right")
STRIP_Y = {"sheet_left": (SEAM_Y, SH_HW), "sheet_middle": (-SEAM_Y, SEAM_Y), "sheet_right": (-SH_HW, -SEAM_Y)}
TEXT_Y = (-SH_HW + 0.012, SH_HW - 0.012)  # the printed lines' left and right margins

# A black printer (as most portable inkjets are), so that the white sheet stands out against it.
LIGHT, DARK, SLOTC, RUBBER, PAPER, INK, KEY = ((0.13, 0.13, 0.15), (0.38, 0.39, 0.42), (0.03, 0.03, 0.03),
                                                (0.66, 0.67, 0.70), (0.96, 0.96, 0.94), (0.30, 0.32, 0.40),
                                                (0.88, 0.89, 0.90))  # rollers light grey: seen against the black case


def _box(x0, x1, y0, y1, z0, z1, color, origin, collide=True) -> BoxShape:
    """A box from world corners (heights above the table), in the frame of a body whose origin is ``origin``."""
    return BoxShape(((x0 + x1) / 2 - origin[0], (y0 + y1) / 2 - origin[1], (z0 + z1) / 2 - origin[2]),
                    (x1 - x0, y1 - y0, z1 - z0), color, collide=collide)


PR_O = ((PR_X0 + PR_X1) / 2, 0.0, 0.0)


def _printer_shapes() -> list:
    """The printer in the frame of the middle of its base: the block round the feed slot (below it, behind it, in
    front of it and at its two ends), the slot's dark inside, a seam line round the case and the operation panel."""
    o = PR_O
    out = [_box(PR_X0, PR_X1, -PR_HW, PR_HW, 0.0, SLOT_FLOOR, LIGHT, o),
           _box(PR_X0, SLOT_X0, -PR_HW, PR_HW, SLOT_FLOOR, PR_H, LIGHT, o),
           _box(SLOT_X1, PR_X1, -PR_HW, PR_HW, SLOT_FLOOR, PR_H, LIGHT, o)]
    for s in (-1.0, 1.0):
        y0, y1 = sorted((s * SLOT_HW, s * PR_HW))
        out.append(_box(SLOT_X0, SLOT_X1, y0, y1, SLOT_FLOOR, PR_H, LIGHT, o))
    # visual only: the slot's dark mouth round the sheet, the rear tray's grey frame round it, a case seam and the panel
    out.append(_box(SLOT_X0 - 0.0005, SLOT_X1 + 0.0005, -SLOT_HW, SLOT_HW, PR_H - 0.0006, PR_H + 0.0002, SLOTC, o,
                    collide=False))
    for x0, x1 in ((SLOT_X0 - 0.010, SLOT_X0 - 0.0005), (SLOT_X1 + 0.0005, SLOT_X1 + 0.010)):
        out.append(_box(x0, x1, -SLOT_HW - 0.008, SLOT_HW + 0.008, PR_H, PR_H + 0.0008, DARK, o, collide=False))
    for s in (-1.0, 1.0):  # the dark windows the two feed rollers sit in, at the slot's mouth
        y0, y1 = sorted((s * (ROLL_Y - ROLL_LEN / 2 - 0.003), s * (ROLL_Y + ROLL_LEN / 2 + 0.003)))
        out.append(_box(PIVOT_X - ROLL_R - 0.003, SLOT_X0 + 0.0002, y0, y1, PR_H - 0.0004, PR_H + 0.0009, SLOTC, o,
                        collide=False))
    out.append(_box(PR_X0 - 0.0004, PR_X0, -PR_HW + 0.01, PR_HW - 0.01, 0.040, 0.042, DARK, o, collide=False))
    out.append(_box(OK_X - 0.030, OK_X + 0.035, OK_Y - 0.030, OK_Y + 0.030, PR_H, PR_H + 0.0008, DARK, o,
                    collide=False))
    out.append(_box(OK_X + 0.016, OK_X + 0.028, OK_Y - 0.012, OK_Y + 0.012, PR_H + 0.0008, PR_H + 0.0014,
                    (0.25, 0.55, 0.85), o, collide=False))  # the panel's little screen
    out.append(_box(PR_X1, PR_X1 + 0.0004, -0.10, 0.10, 0.025, 0.030, SLOTC, o, collide=False))  # the output slot
    return out


def _roller_shapes() -> list:
    """A feed roller on its arm, in the frame of the arm's pivot: the arm standing up inside the printer's back wall
    (touching nothing) and the rubber roller at its top, poking into the slot to press the sheet."""
    return [BoxShape((0.0, 0.0, ARM_L / 2), (0.004, ROLL_LEN - 0.01, ARM_L), DARK, collide=False),
            CylShape((0.0, 0.0, ARM_L), ROLL_R, ROLL_LEN, RUBBER, axis="Y")]


def _strip_shapes(name: str) -> list:
    """A strip of the sheet in the frame of its bottom middle: the paper and, visual only, printed lines on the face
    toward the robot (the lines run on across the seams, as on one sheet)."""
    y0, y1 = STRIP_Y[name]
    yc = (y0 + y1) / 2
    out = [BoxShape((0.0, 0.0, SH_H / 2), (SH_T, y1 - y0, SH_H), PAPER)]
    face = -SH_T / 2 - 0.0003
    # Seen from the robot, +y is on the left: lines start at the left margin (+y) and end raggedly toward -y.
    rows = [(SH_H - 0.012, TEXT_Y[0] + 0.06, 0.004)]  # a heading, then body text
    for k in range(8):
        rows.append((SH_H - 0.024 - 0.0075 * k, TEXT_Y[0] + (0.0, 0.03, 0.01, 0.05, 0.0, 0.02, 0.07, 0.04)[k], 0.0022))
    for z, end, tall in rows:
        a, b = max(y0 + 0.0005, end), min(y1 - 0.0005, TEXT_Y[1])
        if b - a > 0.002:
            out.append(BoxShape((face, (a + b) / 2 - yc, z), (0.0006, b - a, tall), INK, collide=False))
    return out


def _strip_pos(name: str) -> tuple:
    y0, y1 = STRIP_Y[name]
    return (SH_X, (y0 + y1) / 2, TOP + SLOT_FLOOR)


PRINTER = Body.build("printer", pos=(PR_O[0], 0.0, TOP), mass=5.0, color=LIGHT, fixed=True, friction=(0.3, 0.25),
                     shapes=_printer_shapes())
ROLLERS = []
for _name, _s in (("roller_left", 1.0), ("roller_right", -1.0)):
    _pivot = (PIVOT_X, _s * ROLL_Y, TOP + PIVOT_Z)
    ROLLERS.append(Body.build(_name, pos=_pivot, mass=0.02, color=RUBBER, friction=(1.0, 0.9), solver_iterations=32,
                              shapes=_roller_shapes(),
                              joint=Joint(anchor=_pivot, kind="revolute", axis="Y", parent="printer", limits=(-12, 12),
                                          stiffness=ROLL_K, damping=0.004, target=ROLL_DEG)))
OK_TOP0 = np.array([OK_X, OK_Y, TOP + PR_H + OK_UP])
OK_BUTTON = Body.build("ok_button", pos=tuple(float(v) for v in OK_TOP0), mass=0.005, color=KEY, friction=(0.8, 0.7),
                       solver_iterations=32, shapes=[CylShape((0.0, 0.0, -OK_TALL / 2), OK_R, OK_TALL, KEY)],
                       joint=Joint(anchor=tuple(float(v) for v in OK_TOP0), kind="slide", axis="Z", parent="printer",
                                   axial=(OK_K, 1.0, 0.006)))
SHEET = [Body.build("sheet_left", pos=_strip_pos("sheet_left"), mass=SH_KG, color=PAPER, friction=(0.5, 0.45),
                    solver_iterations=32, shapes=_strip_shapes("sheet_left")),
         Body.build("sheet_middle", pos=_strip_pos("sheet_middle"), mass=SH_KG, color=PAPER, friction=(0.5, 0.45),
                    solver_iterations=32, shapes=_strip_shapes("sheet_middle"),
                    joint=Joint(anchor=(SH_X, SEAM_Y, TOP + SLOT_FLOOR + SH_H / 2), kind="slide", axis="Z",
                                parent="sheet_left", axial=(SEAM_K, SEAM_C, SEAM_TRAVEL), break_force=SEAM_N)),
         Body.build("sheet_right", pos=_strip_pos("sheet_right"), mass=SH_KG, color=PAPER, friction=(0.5, 0.45),
                    solver_iterations=32, shapes=_strip_shapes("sheet_right"),
                    joint=Joint(anchor=(SH_X, -SEAM_Y, TOP + SLOT_FLOOR + SH_H / 2), kind="slide", axis="Z",
                                parent="sheet_middle", axial=(SEAM_K, SEAM_C, SEAM_TRAVEL), break_force=SEAM_N))]
OBJECTS = [PRINTER, *ROLLERS, OK_BUTTON, *SHEET]

# -- scoring ----------------------------------------------------------------------------------------------------------
RAISED = 0.015
TORN_MM = 1.5  # a seam's two sides this far apart: the sheet has torn
DOWN_Z = 0.012  # put down: the sheet's lowest point this close to the table
HELD_R, WITH_MM = 0.07, 2.0
REST_MM, REST_STEPS = 3.0, 3
CLEAR = 0.06
STILL_STEPS, STILL_MM = 20, 1.5
_DEBUG = os.environ.get("PAPER_DEBUG") == "1"
_DEBUG_EVERY = int(os.environ.get("PAPER_DEBUG_EVERY", "10"))
START = {n: np.array(_strip_pos(n)) for n in STRIPS}
_CORNERS = {n: np.array([[sx * SH_T / 2, y - (STRIP_Y[n][0] + STRIP_Y[n][1]) / 2, z]
                         for sx in (-1.0, 1.0) for y in STRIP_Y[n] for z in (0.0, SH_H)]) for n in STRIPS}
_SEAMS = (("sheet_left", "sheet_middle", SEAM_Y), ("sheet_middle", "sheet_right", -SEAM_Y))
_EP: dict = {}
_TCP: dict = {}
_ARM_IDS: dict = {}


@on_reset
def _reset() -> None:
    _EP.update(step=0, tick=0, counted=-1, best=0.0, failed="", recent=[], hist=[], offsets={},
               pose={n: (START[n].copy(), np.eye(3)) for n in STRIPS}, rise=0.0, low=SLOT_FLOOR, gap=0.0,
               out=False, came_out=False, down=False, held=False, ok_drop=0.0, ok=False, early_ok=False,
               ok_top=OK_TOP0.copy())
    _TCP.clear()


_reset()


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("PAPER_TRACEBACK\n" + traceback.format_exc(), flush=True)
        raise


def _middle(n: str) -> np.ndarray:
    pos, rot = _EP["pose"][n]
    return pos + rot @ np.array([0.0, 0.0, SH_H / 2])


def _track_inner(env) -> None:
    """Each gripper's centre (TCP) from its joints; each strip's pose: how far the sheet has risen, whether a seam has
    opened, whether the sheet is out, held, put down; how far the OK button is pushed down."""
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS

    _EP["tick"] += 1
    base = getattr(env, "unwrapped", env)
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        if side not in _ARM_IDS:
            _ARM_IDS[side] = list(robot.find_joints(ARM_JOINTS, preserve_order=True)[0])
        q = as_numpy(robot.data.joint_pos)[0]
        _TCP[side] = np.asarray(ik.tcp_pose(q[_ARM_IDS[side]], station.BASES[side])[0], float)
    origin = as_numpy(base.scene.env_origins[0])
    for n in STRIPS:
        p = as_numpy(base.scene[n].data.root_link_pose_w)[0]
        _EP["pose"][n] = (p[:3] - origin, rotation(p[3:7]))
    lows, rises = [], []
    for n in STRIPS:
        pos, rot = _EP["pose"][n]
        lows.append(float((pos + _CORNERS[n] @ rot.T)[:, 2].min()))
        rises.append(float(pos[2] - START[n][2]))
    gap = 0.0
    for a, b, y in _SEAMS:
        for z in (0.0, SH_H):
            pa, ra = _EP["pose"][a]
            pb, rb = _EP["pose"][b]
            wa = pa + ra @ np.array([0.0, y - (STRIP_Y[a][0] + STRIP_Y[a][1]) / 2, z])
            wb = pb + rb @ np.array([0.0, y - (STRIP_Y[b][0] + STRIP_Y[b][1]) / 2, z])
            gap = max(gap, float(np.linalg.norm(wa - wb)))
    if gap * 1000 > TORN_MM and not _EP["failed"]:
        _EP["failed"] = "the card tore: a seam sheared 1.5 mm or more (one side pulled harder than the other)"
    rolls = []
    for n in ("roller_left", "roller_right"):
        rp = as_numpy(base.scene[n].data.root_link_pose_w)[0]
        r = rotation(rp[3:7])
        rolls.append(math.degrees(math.atan2(r[0, 2], r[2, 2])))
    _EP["rolls"] = rolls
    bp = as_numpy(base.scene["ok_button"].data.root_link_pose_w)[0][:3] - origin
    ok_drop = float(OK_TOP0[2] - bp[2])
    low = min(lows)
    out = low - TOP > PR_H + 0.003
    if out:
        _EP["came_out"] = True
    if ok_drop >= OK_PRESS:
        if _EP["came_out"]:
            _EP["ok"] = True
        else:
            _EP["early_ok"] = True
    mids = {n: _middle(n) for n in STRIPS}
    held = False
    for side, tcp in _TCP.items():
        for n in STRIPS:
            off = mids[n] - tcp
            old = _EP["offsets"].get((side, n))
            if np.linalg.norm(off) < HELD_R and old is not None and np.linalg.norm(off - old) * 1000 < WITH_MM:
                held = True
            _EP["offsets"][(side, n)] = off
    _EP["hist"].append(np.concatenate([mids[n] for n in STRIPS]))
    del _EP["hist"][:-REST_STEPS]
    resting = (len(_EP["hist"]) == REST_STEPS
               and max(np.abs(h - _EP["hist"][-1]).max() for h in _EP["hist"]) * 1000 < REST_MM)
    max_z = max(float((_EP["pose"][n][0] + _CORNERS[n] @ _EP["pose"][n][1].T)[:, 2].max()) for n in STRIPS)
    in_front = all(mids[n][0] < PR_X0 - 0.002 for n in STRIPS) and max_z - TOP < SH_H + 0.01  # lying or standing
    down = bool(_EP["came_out"] and low - TOP < DOWN_Z and in_front and resting and not held)
    _EP.update(rise=min(rises), low=low, gap=gap, out=out, down=down, held=held, ok_drop=ok_drop,
               ok_top=OK_TOP0 - np.array([0.0, 0.0, ok_drop]))
    stage = 0.75 if down else 0.5 if out else 0.25 if min(rises) >= RAISED else 0.0
    if not _EP["failed"]:
        _EP["best"] = max(_EP["best"], stage)


def _tally(o) -> dict:
    recent = _EP["recent"]
    if _EP["counted"] != _EP["tick"]:
        _EP["counted"] = _EP["tick"]
        recent.append(np.concatenate([_middle(n) for n in STRIPS]))
        del recent[:-STILL_STEPS]
    still = len(recent) == STILL_STEPS and all(np.abs(p - recent[-1]).max() * 1000 < STILL_MM for p in recent)
    near = 0.0
    if len(_TCP) == 2:
        targets = [_middle(n) for n in STRIPS] + [_EP["ok_top"]]
        near = min(min(np.linalg.norm(v - t) for t in targets) for v in _TCP.values())
    return dict(still=still, near=float(near), done=bool(_EP["down"] and _EP["ok"] and near >= CLEAR and still))


def _debug(t) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    rises = " ".join(f"{(_EP['pose'][n][0][2] - START[n][2]) * 1000:5.1f}" for n in STRIPS)
    print(f"DBG step={_EP['step']} rise_mm(L M R)={rises} low={(_EP['low'] - TOP) * 100:.1f}cm "
          f"seam_gap={_EP['gap'] * 1000:.1f}mm out={_EP['out']} came_out={_EP['came_out']} held={_EP['held']} "
          f"down={_EP['down']} ok_drop={_EP['ok_drop'] * 1000:.1f}mm ok={_EP['ok']} early_ok={_EP['early_ok']} "
          f"near={t['near']:.3f} still={t['still']} best={_EP['best']} failed={_EP['failed'] or '-'} "
          f"rollers_deg={' '.join(f'{a:.2f}' for a in _EP.get('rolls', []))}", flush=True)
    if len(_TCP) == 2:
        print("DBG   " + " | ".join(f"{k} tcp=({v[0]:.3f},{v[1]:.3f},{v[2] - TOP:.3f})" for k, v in _TCP.items()),
              flush=True)


def score(objects) -> float:
    """The best stage reached (0 for the rest of the run once the sheet has torn): 0.25 the sheet raised 1.5 cm or
    more; 0.5 out of the printer; 0.75 put down on the table in front of it. Full score (1.0) is success()."""
    t = _tally(objects)
    if _DEBUG:
        _debug(t)
    return 0.0 if _EP["failed"] else _EP["best"]


def success(objects) -> bool:
    """The sheet out and put down in front of the printer, whole, the OK button pressed after the sheet came out, both
    grippers 6 cm or more from the sheet and the button, still for 1 s."""
    t = _tally(objects)
    return bool(t["done"] and not _EP["failed"])


# -- scripted demo ----------------------------------------------------------------------------------------------------
# As the manual says: each gripper takes the sheet by its top corner above a roller, and together they pull it
# straight up out of the slot, slowly; they carry it toward the robot, clear of the printer, lower it until it nearly
# touches the table and let it go, and it falls over onto the table. The left arm moves away; the right presses the OK
# button and moves away. PAPER_ONE=1 runs it the wrong way: the right gripper alone pulls the sheet by its right corner.

WRONG = os.environ.get("PAPER_ONE") == "1"
GRIP = {"left": grip_rotation(-math.pi / 2), "right": grip_rotation(math.pi / 2)}  # down, fingers closing along x
OPEN_C, CLOSED, LET_GO = -0.025, 0.0, -0.008
FINGERTIP = 0.013  # the fingertips stand this far below the gripper's centre
TCP_Z = 0.108  # the gripper's centre on the sheet, above the table (the pads cover 1.8 cm below the top 7 mm)
GRAB_Y = {"left": 0.11, "right": -0.11}  # 22 cm apart: closer, the two wrists meet once the arms draw back
PULL = float(os.environ.get("PAPER_PULL", "0.071"))  # how far the grippers rise: the sheet's bottom clears the top
DROP_X = float(os.environ.get("PAPER_DROP_X", "0.195"))  # just behind the printer: nearer the robot the arms meet
T_GRIP = 3.0 + 1.2 + 0.5 + 1.0 + 0.5
T_UP = T_GRIP + 3.0 + 0.5
T_DOWN = T_UP + 2.0 + 0.4 + 1.5 + 0.4  # lowered, about to let go


def _good_ik(arm: str, pos, rot):
    from yam_arena import ik, station

    best = None
    for sd in itertools.product((-0.4, 0.0, 0.4), (0.8, 1.3), (0.6, 1.2), (-0.8, 0.8), (-1.2, 0.0, 1.2),
                                (-1.5, 0.0, 1.5)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.asarray(sd, float),
                             base_pos=station.BASES[arm], iters=150)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
                if room > 0.3:
                    break
    if best is None:
        raise ValueError(f"no IK solution for the {arm} arm at {pos}")
    return best[0]


def _lead_in(arm: str, pos, rot, finger: float, label: str, seconds: float = 3.0, steps: int = 15) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, steps + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / steps, station.BASES[arm])
        out.append(Waypoint(seconds / steps, tuple(float(v) for v in p), finger, label, r))
    return out


def _w(t, p, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in p), finger, label, rot)


def _stay(arm: str, seconds: float) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return [Waypoint(seconds, tuple(float(v) for v in pos), OPEN_C, "stay put", rot)]


def _pull(arm: str) -> list:
    """Take the sheet by the top corner on this arm's side and, with the other arm, pull it out, carry it toward the
    robot, lower it and let it go."""
    rot = GRIP[arm]
    grab = np.array([SH_X, GRAB_Y[arm], TOP + TCP_Z])
    up = grab + [0.0, 0.0, PULL]
    over = np.array([DROP_X, grab[1], up[2]])
    low = np.array([DROP_X, grab[1], TOP + (TCP_Z - SLOT_FLOOR) + 0.003])
    out = _lead_in(arm, grab + [0.0, 0.0, 0.05], rot, OPEN_C, "above the sheet's top corner")
    out += [_w(1.2, grab, OPEN_C, "fingers either side of the sheet", rot),
            _w(0.5, grab, OPEN_C, "settle", rot),
            _w(1.0, grab, CLOSED, "take the sheet", rot),
            _w(0.5, grab, CLOSED, "settle", rot),
            _w(3.0, up, CLOSED, "pull it straight up, slowly, together with the other arm", rot),
            _w(0.5, up, CLOSED, "settle", rot)]
    if WRONG:
        return out + [_w(4.0, up, CLOSED, "hold", rot)]
    out += [_w(2.0, over, CLOSED, "carry it toward the robot, clear of the printer", rot),
            _w(0.4, over, CLOSED, "settle", rot),
            _w(1.5, low, CLOSED, "lower it until it nearly touches the table", rot),
            _w(0.4, low, CLOSED, "settle", rot),
            _w(0.3, low, LET_GO, "let go", rot),
            _w(0.5, low + [0.0, 0.0, 0.05], LET_GO, "rise", rot)]
    return out


def _left() -> list:
    if WRONG:
        return _stay("left", 20.0)
    rot = GRIP["left"]
    away = np.array([0.16, 0.24, TOP + 0.16])
    return _pull("left") + [_w(1.4, away, OPEN_C, "move away", rot), _w(10.0, away, OPEN_C, "stay clear", rot)]


def _right() -> list:
    out = _pull("right")
    if WRONG:
        return out
    rot = GRIP["right"]
    above = np.array([OK_X, OK_Y, TOP + 0.14])
    touch = np.array([OK_X, OK_Y, OK_TOP0[2] + 0.003 + FINGERTIP])
    press = touch - [0.0, 0.0, 0.003 + OK_PRESS + 0.003]
    away = np.array([0.16, -0.24, TOP + 0.16])
    out += [_w(0.6, out[-1].pos, CLOSED, "close the fingers", rot),
            _w(1.5, above, CLOSED, "over the OK button", rot),
            _w(0.8, touch, CLOSED, "down onto the OK button", rot),
            _w(0.6, press, CLOSED, "press OK", rot),
            _w(0.5, press, CLOSED, "hold it", rot),
            _w(0.6, above, CLOSED, "let it go", rot),
            _w(1.2, away, CLOSED, "move away", rot),
            _w(4.0, away, CLOSED, "stay clear", rot)]
    return out


def demo():
    return {"left": _left(), "right": _right()}
