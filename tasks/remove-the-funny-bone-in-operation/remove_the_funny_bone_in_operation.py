"""On the station table: a classic Operation game (Hasbro), its 40.0 x 25.1 cm board fixed in front of the robot with
Cavity Sam printed on it; the Funny Bone ailment lies in its bone-shaped cavity at Sam's elbow. The game's wired
tweezers stand tips down in a tweezer stand beside the board, and the Funny Bone ailment card lies face up on the
table. Task (instruction): take the Funny Bone out of Sam without getting buzzed and lay it on its card, then put the
tweezers down. The instruction does not say how.

The rules (Hasbro, Classic Operation B2176 game guide): "Use the tweezers to remove the ailment without touching the
metal sides." "If you touch the metal sides, you'll get buzzed, and Cavity Sam's nose will light up!" The tweezers and
the metal edge of every opening are the two ends of a battery circuit: any touch closes it, the buzzer sounds and the
nose lights.

The hand skill: one gripper holds the light sprung tweezers by their upper arms and works them through its fingers,
squeezing to close the tips and easing off to let their spring open them. The opening is only 3.5 mm wider than the
bone on every side, and each tip is 1 mm thick, so the tips opened to about 9 mm clear the metal edge by about 0.9 mm
a side (and the bone by 1.6 mm): they must come straight down, centred to about a millimetre, open just enough, pinch the
2 g bone across its 6 mm shaft and lift it straight out without swinging.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- board: fixed (taped down), 0.251 (x) x 0.400 (y) x 0.041 m, middle at (0.245, 0.020); cardboard top; Cavity Sam lies along y,
  head at +y. The Funny Bone cavity at (0.165, -0.005), its long axis along y: a 1 mm steel edge band (3 mm wide) round
  the opening, flush with the top; under it a plastic pocket of the same outline, 6.5 mm deep.
- Funny Bone: loose, white plastic, 2 g: a 6 x 18 mm shaft between two knuckle ends (two 7 mm round lobes each), 33 mm
  long, 13 mm across the ends, 5 mm thick, lying flat in its pocket.
- tweezers: loose, steel, 14 g, 110 mm: a joined end (7 x 4 x 10 mm) and two arms, 1 mm thick, 7 mm wide tapering to
  3 mm at the tips, on hinges at the joined end with springs (2 N m/rad each: 1.2 N per tip to close them fully);
  tips 12 mm apart (inside) when free; arms apart along x. The wire to the board is not modelled.
- tweezer stand: loose pine block, 50 (x) x 40 (y) x 35 mm, 45 g, at (0.155, -0.235), with a 22 x 9 mm slot 30 mm
  deep; the tweezers stand in it tips down.
- ailment card: loose, 88 (x) x 63 (y) x 1 mm, 3 g, middle at (0.290, -0.235).

Score (the game's own rule: a buzz loses the turn and the ailment stays):
- a buzz is a touch of either tweezer arm on the steel edge with a contact force of 0.05 N or more (PhysX contact
  force, every control step); Sam's nose lights while a touch lasts. The bone is clear of the opening once its middle
  is outside the opening's outline or its lowest point is 2 mm or more above the board's top.
- 0 for the rest of the run: a buzz while the bone is not clear of the opening (still in it, or being lifted through
  it). A buzz once the bone is clear does not count.
- 0: the bone is in its cavity at the end (never out, or dropped back in).
- 1.0 (success, held 1 s): the bone lies on its card (middle over the card, lowest point within 3 mm of its top), the
  tweezers are put down (nothing of either gripper within 15 mm of them), and both are still (under 1 mm in 1 s).
- 0.3: the bone is out without a buzz but the turn is not finished (on the board or the table, still in the tweezers,
  or the tweezers still held).
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, Link, Linkage, SphereShape, allow_matrix_grips  # noqa: E402
from bodykit import as_numpy, on_reset, on_step, plate, rotation  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
MM = 0.001

# -- colours ---------------------------------------------------------------------------------------------------------
BOARD_BLUE = (0.16, 0.36, 0.70)
BOARD_EDGE = (0.10, 0.22, 0.48)
SKIN = (0.92, 0.60, 0.44)
STEEL = (0.72, 0.74, 0.78)
DARK = (0.10, 0.08, 0.10)
BONE = (0.97, 0.96, 0.90)
NOSE_DIM = (0.45, 0.06, 0.06)
NOSE_LIT = (1.0, 0.10, 0.08)
PINE = (0.86, 0.72, 0.50)
CARD = (0.98, 0.93, 0.55)
RED = (0.80, 0.10, 0.10)
GREEN = (0.20, 0.65, 0.30)

# -- the layout ------------------------------------------------------------------------------------------------------
BOARD_C = np.array([0.245, 0.020])
BOARD_X, BOARD_Y, BOARD_H = 0.251, 0.400, 0.041
SURF = TOP + BOARD_H  # the board's top
CAV = np.array([float(v) for v in os.environ.get("OPER_CAV", "0.165,-0.005").split(",")])  # the Funny Bone's cavity
DEPTH = 0.0065  # the pocket's floor below the top
RIM_T = 0.001  # the steel edge band's thickness
RIM_W = 0.003  # and width
CLEAR = 0.0035  # the opening is this much wider than the bone on every side

# the bone (local frame: long axis along y): a shaft and two knuckle ends of two round lobes each
SHAFT_HX, SHAFT_HY = 0.003, 0.009
LOBE_R, LOBE_X, LOBE_Y = 0.0035, 0.003, 0.0125
BONE_T = 0.005
BONE_HX = LOBE_X + LOBE_R  # 6.5 mm: half the width across the ends
BONE_HY = LOBE_Y + LOBE_R  # 16 mm: half the length
# the opening: the bone's outline grown by CLEAR (the lobes as rectangles), as a union of rectangles
# (x0, x1, y0, y1) in the cavity's frame
HOLE = [(-SHAFT_HX - CLEAR, SHAFT_HX + CLEAR, -BONE_HY - CLEAR, BONE_HY + CLEAR),
        (-BONE_HX - CLEAR, BONE_HX + CLEAR, SHAFT_HY - CLEAR, BONE_HY + CLEAR),
        (-BONE_HX - CLEAR, BONE_HX + CLEAR, -BONE_HY - CLEAR, -SHAFT_HY + CLEAR)]
OUTER = [(a - RIM_W, b + RIM_W, c - RIM_W, d + RIM_W) for a, b, c, d in HOLE]

# the tweezers: standing tips down in the stand's slot, arms apart along x
STAND_C = np.array([0.155, -0.235])
STAND_X, STAND_Y, STAND_H = 0.050, 0.040, 0.035
SLOT_X, SLOT_Y, SLOT_D = 0.022, 0.009, 0.030
TW_L = 0.110  # joined end to tips
TW_JOIN = 0.010  # the joined block's height (the hinges at its bottom)
TW_T = 0.001  # arm thickness
TW_GAP = 0.012  # inside gap between the tips when free
TW_K = 2.0  # N m/rad: each arm's spring
TW_CLOSE_DEG = math.degrees(math.atan((TW_GAP / 2) / (TW_L - TW_JOIN)))  # an arm's swing to close the tips
TW_TOP0 = TOP + (STAND_H - SLOT_D) + TW_L + 0.0015  # the joined end's top, standing in the stand
HINGE0 = np.array([STAND_C[0], STAND_C[1], TW_TOP0 - TW_JOIN])
ARM_LEN = TW_L - TW_JOIN

CARD_C = np.array([0.290, -0.235])
CARD_X, CARD_Y, CARD_T = 0.088, 0.063, 0.001

# Cavity Sam's nose (lights while the tweezers touch the edge)
NOSE = np.array([0.245, 0.170 + 0.000])


def _v(p) -> tuple:
    return tuple(float(x) for x in p)


def _cells(rects_in, rects_out, bounds):
    """Axis-aligned cells (x0, x1, y0, y1) covering the part of ``bounds`` inside some rect of ``rects_in`` (all of
    ``bounds`` when None) and outside every rect of ``rects_out``, merged along y."""
    xs = sorted({bounds[0], bounds[1]} | {v for r in (rects_in or []) + rects_out for v in r[:2]})
    ys = sorted({bounds[2], bounds[3]} | {v for r in (rects_in or []) + rects_out for v in r[2:]})
    xs = [x for x in xs if bounds[0] <= x <= bounds[1]]
    ys = [y for y in ys if bounds[2] <= y <= bounds[3]]
    inside = lambda rs, x, y: any(r[0] < x < r[1] and r[2] < y < r[3] for r in rs)  # noqa: E731
    out = []
    for x0, x1 in zip(xs[:-1], xs[1:]):
        run = None
        for y0, y1 in zip(ys[:-1], ys[1:]):
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            keep = (rects_in is None or inside(rects_in, cx, cy)) and not inside(rects_out, cx, cy)
            if keep:
                run = [x0, x1, y0 if run is None else run[2], y1]
            elif run is not None:
                out.append(tuple(run))
                run = None
        if run is not None:
            out.append(tuple(run))
    return out


def _board() -> Body:
    """The board: its lower part solid, the top layer (down to the pocket's floor) cut round the opening's outer edge
    (the steel band sits in the cut), and Cavity Sam printed on it (visual only)."""
    cx, cy = CAV - BOARD_C  # the cavity in the board's frame (origin: the board's middle, on the table)
    hx, hy = BOARD_X / 2, BOARD_Y / 2
    low_h = BOARD_H - DEPTH
    sh = [BoxShape((0.0, 0.0, low_h / 2), (BOARD_X, BOARD_Y, low_h), BOARD_BLUE)]
    # the top layer round the opening: cardboard everywhere outside the steel band's outer edge ...
    outer = [(a + cx, b + cx, c + cy, d + cy) for a, b, c, d in OUTER]
    hole = [(a + cx, b + cx, c + cy, d + cy) for a, b, c, d in HOLE]
    for x0, x1, y0, y1 in _cells(None, outer, (-hx, hx, -hy, hy)):
        sh.append(BoxShape(((x0 + x1) / 2, (y0 + y1) / 2, low_h + DEPTH / 2), (x1 - x0, y1 - y0, DEPTH), BOARD_BLUE))
    # ... and the plastic pocket's walls under the band (the band's outline minus the opening)
    wall_h = DEPTH - RIM_T
    for x0, x1, y0, y1 in _cells(outer, hole, (-hx, hx, -hy, hy)):
        sh.append(BoxShape(((x0 + x1) / 2, (y0 + y1) / 2, low_h + wall_h / 2), (x1 - x0, y1 - y0, wall_h), DARK))
    # the pocket's floor shows dark
    sh.append(BoxShape((cx, cy, low_h + 0.0002), (2 * (BONE_HX + CLEAR) - 0.0005, 2 * (BONE_HY + CLEAR) - 0.0005,
                                                    0.0004), DARK, collide=False))
    z = BOARD_H + 0.0002
    vis = lambda c, s, col: sh.append(BoxShape((c[0], c[1], z), (s[0], s[1], 0.0003), col, collide=False))  # noqa
    # the board's border, and Cavity Sam: head (+y), body, legs, the arm nearer the robot with the cavity at the elbow
    for c, s in [((hx - 0.006, 0.0), (0.012, BOARD_Y)), ((-hx + 0.006, 0.0), (0.012, BOARD_Y)),
                 ((0.0, hy - 0.006), (BOARD_X, 0.012)), ((0.0, -hy + 0.006), (BOARD_X, 0.012))]:
        vis(c, s, BOARD_EDGE)
    sam = BOARD_C  # Sam's frame = the board's
    head = np.array([NOSE[0], NOSE[1]]) - sam
    sh.append(CylShape((head[0], head[1], z), 0.040, 0.0003, SKIN, collide=False))
    sh.append(CylShape((head[0], head[1] + 0.020, z - 0.0001), 0.024, 0.0003, (0.85, 0.55, 0.20), collide=False))
    vis((0.0, 0.065), (0.100, 0.110), SKIN)  # body
    vis((0.0, 0.005), (0.080, 0.020), (0.95, 0.95, 0.95))  # shorts
    vis((-0.022, -0.085), (0.034, 0.170), SKIN)  # legs
    vis((0.022, -0.085), (0.034, 0.170), SKIN)
    ex = cx  # the near arm runs along y at the cavity's x, with a gap round the opening
    band = 0.0005
    top_y, bot_y = 0.115, -0.090
    y_hi = cy + BONE_HY + CLEAR + RIM_W + band
    y_lo = cy - BONE_HY - CLEAR - RIM_W - band
    vis((ex, (top_y + y_hi) / 2), (0.030, top_y - y_hi), SKIN)
    vis((ex, (y_lo + bot_y) / 2), (0.030, y_lo - bot_y), SKIN)
    vis((0.075, 0.020), (0.030, 0.190), SKIN)  # the far arm
    # the other ailments' openings (painted here: only the Funny Bone's is modelled)
    for c, s, col in [((0.0, 0.085), (0.030, 0.026), RED), ((0.0, 0.040), (0.040, 0.018), GREEN),
                      ((0.030, 0.105), (0.018, 0.016), (0.90, 0.80, 0.30)), ((-0.022, -0.050), (0.020, 0.026), BONE),
                      ((0.022, -0.130), (0.020, 0.022), (0.55, 0.35, 0.80)), ((0.075, -0.050), (0.018, 0.026), BONE)]:
        vis(c, (s[0] + 0.007, s[1] + 0.007), STEEL)
        sh.append(BoxShape((c[0], c[1], z + 0.0002), (s[0], s[1], 0.0003), col, collide=False))
    sh.append(CylShape((head[0], head[1], z + 0.0004), 0.0095, 0.0006, NOSE_DIM, collide=False))
    return Body.build("board", pos=(BOARD_C[0], BOARD_C[1], TOP), mass=0.45, color=BOARD_BLUE, fixed=True,
                      friction=(0.5, 0.4), shapes=sh, contact_offset=0.001)


def _rim() -> Body:
    """The steel edge band round the opening (the circuit's other end), flush with the board's top."""
    sh = [BoxShape(((x0 + x1) / 2, (y0 + y1) / 2, -RIM_T / 2), (x1 - x0, y1 - y0, RIM_T), STEEL)
          for x0, x1, y0, y1 in _cells(OUTER, HOLE, (-0.05, 0.05, -0.05, 0.05))]
    return Body.build("rim", pos=(CAV[0], CAV[1], SURF), mass=0.02, color=STEEL, fixed=True, friction=(0.3, 0.25),
                      shapes=sh, contact_offset=0.001)


def _bone() -> Body:
    sh = [BoxShape((0.0, 0.0, 0.0), (2 * SHAFT_HX, 2 * SHAFT_HY + 0.002, BONE_T), BONE)]
    for sy in (-1, 1):
        for sx in (-1, 1):
            sh.append(CylShape((sx * LOBE_X, sy * LOBE_Y, 0.0), LOBE_R, BONE_T, BONE))
    return Body.build("funny_bone", pos=(CAV[0], CAV[1], SURF - DEPTH + BONE_T / 2 + 0.0005), mass=0.002, color=BONE,
                      friction=(0.6, 0.5), shapes=sh, contact_offset=0.001, solver_iterations=32)


def _arm_shapes(side: int) -> list:
    """One arm (``side`` +1: the far arm, +x; -1: the near arm), from the hinge down to its tip, in the tweezers'
    frame (origin at the hinge): 1 mm thick across x, tapering from 7 to 3 mm wide (along y)."""
    x_top = side * (0.0015 + TW_T / 2)
    x_tip = side * (TW_GAP / 2 + TW_T / 2)
    pt = lambda s: np.array([x_top + (x_tip - x_top) * s, 0.0, -ARM_LEN * s])  # noqa: E731
    out = []
    for s0, s1, w in ((0.0, 0.40, 0.007), (0.40, 0.80, 0.005), (0.80, 1.0, 0.003)):
        out.append(plate(_v(pt(s0)), _v(pt(s1)), w, TW_T, (1.0, 0.0, 0.0), STEEL))
    return out


def _tweezers() -> Linkage:
    base = [BoxShape((0.0, 0.0, TW_JOIN / 2), (0.004, 0.007, TW_JOIN), STEEL),
            CylShape((0.0, 0.0, TW_JOIN + 0.004), 0.0025, 0.008, RED, collide=False)]  # the wire's sleeve
    links = [Link("arm_far", tuple(_arm_shapes(+1)), mass=0.005, anchor=_v(HINGE0), axis="Y", stiffness=TW_K,
                  spring_damping=0.002, limits=(-0.3, TW_CLOSE_DEG - 0.15), report_contacts=True),
             Link("arm_near", tuple(_arm_shapes(-1)), mass=0.005, anchor=_v(HINGE0), axis="Y", stiffness=TW_K,
                  spring_damping=0.002, limits=(-(TW_CLOSE_DEG - 0.15), 0.3), report_contacts=True)]
    return Linkage.build("tweezers", pos=_v(HINGE0), mass=0.004, color=STEEL, floating=True, friction=(0.7, 0.6),
                         shapes=base, links=links, contact_offset=0.001, solver_iterations=48)


def _stand() -> Body:
    hx, hy = STAND_X / 2, STAND_Y / 2
    fl = STAND_H - SLOT_D
    sh = [BoxShape((0.0, 0.0, fl / 2), (STAND_X, STAND_Y, fl), PINE)]
    slot = [(-SLOT_X / 2, SLOT_X / 2, -SLOT_Y / 2, SLOT_Y / 2)]
    for x0, x1, y0, y1 in _cells(None, slot, (-hx, hx, -hy, hy)):
        sh.append(BoxShape(((x0 + x1) / 2, (y0 + y1) / 2, fl + SLOT_D / 2), (x1 - x0, y1 - y0, SLOT_D), PINE))
    return Body.build("stand", pos=(STAND_C[0], STAND_C[1], TOP + 0.0005), mass=0.045, color=PINE,
                      friction=(0.6, 0.5), shapes=sh)


def _card() -> Body:
    sh = [BoxShape((0.0, 0.0, 0.0), (CARD_X, CARD_Y, CARD_T), CARD),
          BoxShape((0.0, 0.0, CARD_T / 2 + 0.0001), (0.050, 0.020, 0.0002), BONE, collide=False),
          BoxShape((-0.030, 0.0, CARD_T / 2 + 0.0001), (0.012, 0.045, 0.0002), GREEN, collide=False)]
    return Body.build("card", pos=(CARD_C[0], CARD_C[1], TOP + CARD_T / 2 + 0.0005), mass=0.003, color=CARD,
                      friction=(0.6, 0.5), shapes=sh)


BOARD = _board()
RIM = _rim()
BONE_BODY = _bone()
TWEEZERS = _tweezers()
STAND = _stand()
CARD_BODY = _card()
# Sam's nose lit (visual only): kept under the table, laid on the nose while the tweezers touch the edge
HIDE = np.array([0.245, 0.170, TOP - 0.30])
LIT = Body.build("nose_lit", pos=_v(HIDE), mass=0.001, color=NOSE_LIT, gravity=False, damping=(5.0, 5.0),
                 shapes=[CylShape((0.0, 0.0, 0.0), 0.0095, 0.002, NOSE_LIT, collide=False)])
OBJECTS = [BOARD, RIM, BONE_BODY, TWEEZERS, STAND, CARD_BODY, LIT]

# -- tracking ------------------------------------------------------------------------------------------------------
BUZZ_N = 0.05  # N: a touch this hard closes the circuit
BUZZ_GAP_STEPS = 5  # touches less than 0.25 s apart (5 steps at 20 Hz) are one buzz
ON_CARD_MM = 3.0
HELD_MM = 15.0
STILL_STEPS, STILL_MM = 20, 1.0
_DEBUG = os.environ.get("OPER_DEBUG") == "1"
_DEBUG_EVERY = int(os.environ.get("OPER_DEBUG_EVERY", "5"))
_EP: dict = {}
_IDS: dict = {}
_CV: dict = {}

TIP_REL = {+1: np.array([+(TW_GAP / 2 + TW_T / 2), 0.0, -ARM_LEN]), -1: np.array([-(TW_GAP / 2 + TW_T / 2), 0.0,
                                                                                    -ARM_LEN])}
ARM_SAMPLES = {s: np.array([[s * (0.002 + (TW_GAP / 2) * k), 0.0, -ARM_LEN * k] for k in np.linspace(0, 1, 12)])
               for s in (+1, -1)}
BASE_SAMPLES = np.array([[0.0, 0.0, z] for z in np.linspace(0.0, TW_JOIN, 4)])
BONE_SAMPLES = np.array([[sx * BONE_HX, sy * BONE_HY, sz * BONE_T / 2] for sx in (-1, 0, 1) for sy in (-1, 0, 1)
                         for sz in (-1, 1)])


@on_reset
def _reset() -> None:
    _EP.update(tick=0, step=0, counted=-1, touching=False, quiet=99, buzzes=0, force=0.0, peak=0.0, recent=[],
               out_ever=False, lost=False, lost_step=-1, lit=None, tcp={}, gaps={})
    _CV.pop("view", None)


_reset()


def _finger_points() -> dict:
    import struct
    import xml.etree.ElementTree as ET

    from yam_arena import YAM_URDF
    from yam_arena.urdf import rpy_matrix

    root = pathlib.Path(YAM_URDF).parent
    out = {}
    for link in ET.parse(YAM_URDF).getroot().iter("link"):
        name = link.get("name")
        if name not in ("tip_left", "tip_right"):
            continue
        pts = []
        for col in link.findall("collision"):
            o = col.find("origin")
            xyz = np.array([float(v) for v in (o.get("xyz") or "0 0 0").split()]) if o is not None else np.zeros(3)
            rpy = np.array([float(v) for v in (o.get("rpy") or "0 0 0").split()]) if o is not None else np.zeros(3)
            b = (root / col.find("geometry").find("mesh").get("filename")).read_bytes()
            n = struct.unpack("<I", b[80:84])[0]
            tri = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
            v = tri["v"].reshape(-1, 3).astype(float)
            pts.append(np.unique(np.round(v, 4), axis=0) @ rpy_matrix(rpy).T + xyz)
        out[name] = np.concatenate(pts)
    return out


def _rim_force(base) -> float:
    """The largest contact force (N) between either tweezer arm and the steel edge band, from PhysX's pair-wise
    contact forces."""
    from isaaclab_physx.physics import PhysxManager

    sim = PhysxManager.get_physics_sim_view()
    view = _CV.get("view") if _CV.get("sim") is sim else None
    if view is None:
        env = base.scene.env_prim_paths[0]
        view = sim.create_rigid_contact_view([f"{env}/tweezers/arm_far", f"{env}/tweezers/arm_near"],
                                             filter_patterns=[[f"{env}/rim/body"], [f"{env}/rim/body"]],
                                             max_contact_data_count=64)
        _CV.update(sim=sim, view=view, dt=float(base.physics_dt))
    forces = view.get_contact_force_matrix(dt=_CV["dt"])
    a = forces.numpy() if hasattr(forces, "numpy") else as_numpy(forces)
    return float(np.linalg.norm(np.asarray(a, float).reshape(-1, 3), axis=-1).max())


def _show_nose(base, origin, lit: bool) -> None:
    if _EP["lit"] is lit:
        return
    import torch

    _EP["lit"] = lit
    p = np.array([NOSE[0], NOSE[1], SURF + 0.0015]) if lit else HIDE
    p = p + origin
    pose = np.array([[p[0], p[1], p[2], 0.0, 0.0, 0.0, 1.0]])
    obj = base.scene["nose_lit"]
    obj.write_root_link_pose_to_sim_index(root_pose=torch.tensor(pose, dtype=torch.float32, device=base.device))
    obj.write_root_com_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), dtype=torch.float32,
                                                                       device=base.device))


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("OPER_TRACEBACK\n" + traceback.format_exc(), flush=True)
        raise


def _in_hole(p) -> bool:
    d = np.asarray(p[:2]) - CAV
    return any(r[0] <= d[0] <= r[1] and r[2] <= d[1] <= r[3] for r in HOLE)


def _track_inner(env) -> None:
    from scipy.spatial import cKDTree

    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    _EP["tick"] += 1
    base = getattr(env, "unwrapped", env)
    origin = as_numpy(base.scene.env_origins[0])
    if "fingers" not in _IDS:
        _IDS["fingers"] = _finger_points()
    fingers = {}
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        if side not in _IDS:
            _IDS[side] = list(robot.find_joints(ARM_JOINTS, preserve_order=True)[0])
            _IDS[side + "_f"] = list(robot.find_joints(FINGER_JOINTS, preserve_order=True)[0])
            _IDS[side + "_tips"] = [robot.body_names.index(n) for n in ("tip_left", "tip_right")]
        q = as_numpy(robot.data.joint_pos)[0]
        _EP["tcp"][side] = np.asarray(ik.tcp_pose(q[_IDS[side]], station.BASES[side])[0], float)
        _EP["gaps"][side] = float(-q[_IDS[side + "_f"]].sum())
        poses = as_numpy(robot.data.body_link_pose_w)[0]
        fingers[side] = np.concatenate([_IDS["fingers"][n] @ rotation(poses[i][3:7]).T + poses[i][:3] - origin
                                        for n, i in zip(("tip_left", "tip_right"), _IDS[side + "_tips"])])
    tw = base.scene["tweezers"]
    if "bodies" not in _IDS:
        names, jn = list(tw.body_names), list(tw.joint_names)
        _IDS["bodies"] = {k: names.index(k) for k in ("base", "arm_far", "arm_near")}
        _IDS["joints"] = {k: jn.index(f"{k}_turn") for k in ("arm_far", "arm_near")}
    bp = as_numpy(tw.data.body_link_pose_w)[0]
    ib = _IDS["bodies"]
    hp, hr = bp[ib["base"]][:3] - origin, rotation(bp[ib["base"]][3:7])
    tips, pts = {}, [hp + BASE_SAMPLES @ hr.T]
    for s, k in ((+1, "arm_far"), (-1, "arm_near")):
        lp, lr = bp[ib[k]][:3] - origin, rotation(bp[ib[k]][3:7])
        tips[s] = lp + lr @ TIP_REL[s]
        pts.append(lp + ARM_SAMPLES[s] @ lr.T)
    jp = as_numpy(tw.data.joint_pos)[0]
    gap = float(np.linalg.norm(tips[+1] - tips[-1])) - TW_T  # inside gap between the tips
    tree = cKDTree(np.concatenate(pts))
    near = min(float(tree.query(fp, k=1)[0].min()) for fp in fingers.values())
    # the bone
    bs = as_numpy(base.scene["funny_bone"].data.root_link_pose_w)[0]
    bpos, brot = bs[:3] - origin, rotation(bs[3:7])
    bpts = bpos + BONE_SAMPLES @ brot.T
    low = float(bpts[:, 2].min())
    in_cavity = _in_hole(bpos) and low < SURF - 0.0005
    clear = not in_cavity and (low > SURF + 0.002 or not _in_hole(bpos))  # clear of the opening
    if clear:
        _EP["out_ever"] = True
    cs = as_numpy(base.scene["card"].data.root_link_pose_w)[0]
    cpos, crot = cs[:3] - origin, rotation(cs[3:7])
    bc = crot.T @ (bpos - cpos)
    on_card = abs(bc[0]) <= CARD_X / 2 and abs(bc[1]) <= CARD_Y / 2 and \
        low - (cpos[2] + CARD_T / 2) <= ON_CARD_MM * MM and low >= cpos[2] - 0.002
    # the circuit
    f = _rim_force(base)
    touching = f >= BUZZ_N
    if touching and not _EP["touching"] and _EP["quiet"] >= BUZZ_GAP_STEPS:
        _EP["buzzes"] += 1
    _EP["quiet"] = 0 if touching else _EP["quiet"] + 1
    if touching and not clear and not _EP["lost"]:  # buzzed while the bone is in or passing the opening: turn lost
        _EP["lost"] = True
        _EP["lost_step"] = _EP["tick"]
    _EP["touching"] = touching
    _show_nose(base, origin, touching)
    _EP.update(force=f, peak=max(_EP["peak"], f), hp=hp, tips=tips, gap=gap, near=near, bpos=bpos, low=low,
               in_cavity=in_cavity, on_card=on_card, still_pts=np.concatenate([hp, bpos]),
               joints=(math.degrees(jp[_IDS["joints"]["arm_far"]]), math.degrees(jp[_IDS["joints"]["arm_near"]])))


def _tally(o) -> dict:
    recent = _EP["recent"]
    if _EP["counted"] != _EP["tick"] and "hp" in _EP:
        _EP["counted"] = _EP["tick"]
        recent.append(np.asarray(_EP["still_pts"], float).copy())
        del recent[:-STILL_STEPS]
    still = len(recent) == STILL_STEPS and all(np.abs(p - recent[-1]).max() * 1000 < STILL_MM for p in recent)
    out = "hp" in _EP and not _EP["in_cavity"] and _EP["out_ever"]
    put_down = _EP.get("near", 0.0) * 1000 >= HELD_MM
    done = bool(out and _EP.get("on_card") and put_down and still)
    return dict(still=still, out=out, put_down=put_down, done=done)


def _value(t) -> float:
    if _EP["lost"] or not t["out"]:
        return 0.0
    return 1.0 if t["done"] else 0.3


def _debug(t) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY or "hp" not in _EP:
        return
    tips, b = _EP["tips"], _EP["bpos"]
    mid = (tips[+1] + tips[-1]) / 2
    print(f"DBG step={_EP['step']} f={_EP['force']:.3f}N peak={_EP['peak']:.3f} buzzes={_EP['buzzes']} lost={int(_EP['lost'])} "
          f"tipmid-cav=({(mid[0] - CAV[0]) * 1000:+.2f},{(mid[1] - CAV[1]) * 1000:+.2f})mm "
          f"tip_z={(min(tips[1][2], tips[-1][2]) - SURF) * 1000:+.1f}mm gap={_EP['gap'] * 1000:.2f}mm "
          f"far=({(tips[1][0] - CAV[0]) * 1000:+.2f}) near=({(tips[-1][0] - CAV[0]) * 1000:+.2f}) "
          f"j=({_EP['joints'][0]:+.2f},{_EP['joints'][1]:+.2f}) top=({_EP['hp'][0]:.3f},{_EP['hp'][1]:.3f},"
          f"{_EP['hp'][2] - TOP:.3f}) bone=({b[0]:.3f},{b[1]:.3f},{_EP['low'] - TOP:.4f}) incav={int(_EP['in_cavity'])} "
          f"out={int(t['out'])} card={int(_EP['on_card'])} near={_EP['near'] * 1000:.0f}mm still={int(t['still'])} "
          f"score={_value(t):.2f} | "
          + " ".join(f"{k} tcp=({v[0]:.3f},{v[1]:.3f},{v[2] - TOP:.3f}) gap={_EP['gaps'][k] * 1000:.1f}"
                     for k, v in _EP["tcp"].items()), flush=True)


def score(objects) -> float:
    t = _tally(objects)
    if _DEBUG:
        _debug(t)
    return _value(t)


def success(objects) -> bool:
    t = _tally(objects)
    return bool(t["done"] and not _EP["lost"])


# -- scripted demo --------------------------------------------------------------------------------------------------
# Open loop, from the known layout; the left arm stays at the ready pose. The right gripper (fingers closing along x,
# across the tweezers' arms) comes down round the tweezers standing in their stand and squeezes them shut 45 mm below
# their top, lifts them out and carries them over the Funny Bone's cavity. It eases its fingers so the tweezers' spring
# opens the tips to about 9 mm, lowers them straight down past the steel edge either side of the bone's shaft,
# squeezes (the tips pinch the shaft), lifts the bone straight up out of the opening, carries it over its card, lowers
# it and eases the fingers so the tips let it drop onto the card. It squeezes the tweezers shut again, puts them back
# in their stand, opens and rises clear.
# OPER_MODE=sloppy aims the tips OPER_SLOP mm (default 2.5) off to one side (they touch the edge: buzzes); OPER_MODE=fingers leaves the
# tweezers in their stand and tries to pinch the bone out with the gripper's own fingers (they do not fit the opening).
OPEN_C = -0.047
ARM = "right"
MODE = os.environ.get("OPER_MODE", "demo")
GRIP_DOWN = float(os.environ.get("OPER_GRIP_DOWN", "0.045"))  # the TCP this far below the tweezers' top
SHUT = float(os.environ.get("OPER_SHUT", "-0.0021"))  # squeeze the tweezers shut
EASE = float(os.environ.get("OPER_EASE", "-0.0038"))  # eased: the tips open to about 9 mm
TIP_DOWN = TW_L - GRIP_DOWN  # the tips this far below the TCP
AIM = np.array([float(v) for v in os.environ.get("OPER_AIM", "0,0").split(",")]) * MM  # tip aim correction (x, y)
PINCH_Z = float(os.environ.get("OPER_PINCH_Z", "0.0012"))  # the tips' ends this far above the pocket's floor
YAW = math.pi / 2  # fingers closing along x
SLOP = float(os.environ.get("OPER_SLOP", "2.5")) * MM  # OPER_MODE=sloppy: the tips aimed this far off along x


def _good_ik(arm: str, pos, rot):
    from yam_arena import ik, station

    best = None
    b = station.BASES[arm]
    j1 = math.atan2(float(pos[1]) - b[1], float(pos[0]) - b[0])
    for sd in itertools.product((j1,), (0.8, 1.3), (0.6, 1.2), (-0.8, 0.8), (0.0,), (-1.8, -0.9, 0.0, 0.9, 1.8)):
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


def _lead_in(arm: str, pos, rot, finger: float, label: str, seconds: float = 3.0, steps: int = 15) -> list:
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


def _stay(arm: str, t: float = 120.0) -> list:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return [Waypoint(t, tuple(float(v) for v in pos), OPEN_C, "stay put", rot)]


def _tcp_for_tips(xy, tip_z) -> np.ndarray:
    """The TCP that puts the tips' middle at ``xy`` with their ends at height ``tip_z`` (world)."""
    return np.array([xy[0], xy[1], tip_z + TIP_DOWN])


def _fingers_path(r) -> list:
    """The wrong way: pinch the bone out with the gripper's own fingers."""
    p = np.array([CAV[0], CAV[1], SURF + 0.010])
    out = _lead_in(ARM, p + np.array([0.0, 0.0, 0.06]), r, -0.012, "above the bone, fingers 24 mm apart")
    out += [_w(3.0, p, -0.012, "lower the fingers onto the opening", r),
            _w(2.0, p - np.array([0.0, 0.0, 0.012]), -0.012, "push down", r),
            _w(1.5, p - np.array([0.0, 0.0, 0.012]), 0.0, "pinch", r),
            _w(3.0, p + np.array([0.0, 0.0, 0.05]), 0.0, "lift", r),
            _w(20.0, p + np.array([0.0, 0.0, 0.05]), 0.0, "wait", r)]
    return out


def _arm_path() -> list:
    r = grip_rotation(YAW)
    if MODE == "fingers":
        return _fingers_path(r)
    grip = np.array([STAND_C[0], STAND_C[1], TW_TOP0 - GRIP_DOWN])
    up = np.array([0.0, 0.0, 1.0])
    out = _lead_in(ARM, grip + 0.06 * up, r, -0.015, "above the tweezers, fingers 30 mm apart")
    out += [_w(2.5, grip, -0.015, "lower round the tweezers' arms", r),
            _w(0.5, grip, -0.015, "settle", r),
            _w(1.2, grip, SHUT, "squeeze them shut", r),
            _w(2.5, grip + 0.060 * up, SHUT, "lift them out of the stand", r)]
    aim = CAV + AIM + (np.array([SLOP, 0.0]) if MODE == "sloppy" else 0.0)
    above = _tcp_for_tips(aim, SURF + 0.025)
    floor = SURF - DEPTH
    at = _tcp_for_tips(aim, floor + PINCH_Z)
    out += [_w(4.0, above + 0.02 * up, SHUT, "carry them over the Funny Bone", r),
            _w(1.5, above, SHUT, "lower the closed tips to 25 mm over the opening", r),
            _w(1.0, above, SHUT, "settle", r),
            _w(1.2, above, EASE, "ease the fingers: the tips open", r),
            _w(0.8, above, EASE, "settle", r),
            _w(4.0, at, EASE, "lower the tips straight down either side of the shaft", r),
            _w(0.8, at, EASE, "settle", r),
            _w(1.2, at, SHUT, "squeeze: the tips pinch the shaft", r),
            _w(0.6, at, SHUT, "hold", r),
            _w(4.0, at + 0.040 * up, SHUT, "lift the bone straight up out of the opening", r),
            _w(0.6, at + 0.040 * up, SHUT, "settle", r)]
    card_z = TOP + CARD_T
    drop = _tcp_for_tips(CARD_C, card_z + 0.006)
    out += [_w(4.0, drop + 0.08 * up, SHUT, "carry it over its card", r),
            _w(1.5, drop, SHUT, "lower it to just over the card", r),
            _w(0.6, drop, SHUT, "settle", r),
            _w(1.2, drop, EASE, "ease the fingers: the bone drops onto the card", r),
            _w(1.0, drop + 0.04 * up, EASE, "rise", r),
            _w(1.0, drop + 0.04 * up, SHUT, "squeeze them shut", r)]
    back = grip + np.array([0.0, 0.0, 0.004])
    out += [_w(4.0, back + 0.060 * up, SHUT, "carry them back over the stand", r),
            _w(2.5, back, SHUT, "lower them into the slot", r),
            _w(0.6, back, SHUT, "settle", r),
            _w(1.2, back, -0.015, "let go", r),
            _w(2.5, back + 0.070 * up, -0.015, "rise clear", r),
            _w(20.0, back + 0.070 * up + np.array([-0.03, 0.03, 0.0]), -0.015, "stay clear", r)]
    return out


def demo():
    other = "left" if ARM == "right" else "right"
    return {ARM: _arm_path(), other: _stay(other)}
